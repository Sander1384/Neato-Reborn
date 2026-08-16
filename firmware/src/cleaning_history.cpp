#include "cleaning_history.h"
#include "json_fields.h"
#include "neato_serial.h"
#include "system_manager.h"
#include <SPIFFS.h>
#include <cmath>

CleaningHistory::CleaningHistory(NeatoSerial& neato, DataLogger& logger, SystemManager& sysMgr) :
    LoopTask(HISTORY_INTERVAL_IDLE_MS), neato(neato), dataLogger(logger), systemManager(sysMgr) {
    TaskRegistry::add(this);
}

void CleaningHistory::notifyCleanStart() {
    if (collecting)
        return; // Already recording
    setInterval(HISTORY_INTERVAL_ACTIVE_MS);
}

void CleaningHistory::tick() {
    // Run incremental compression when a session just finished
    if (compressing) {
        if (compressStep()) {
            // Compression done - remove raw source and cache metadata
            compressSrc.close();
            compressDst.close();
            SPIFFS.remove(compressSrcPath);
            LOG("HIST", "Compression done: %s", compressDstPath.c_str());

            // Cache session/summary from memory (avoids decompressing on next list request)
            String hsName = compressDstPath;
            int lastSlash = hsName.lastIndexOf('/');
            if (lastSlash >= 0)
                hsName = hsName.substring(lastSlash + 1);
            metaCache[hsName] = {pendingSessionJson, pendingSummaryJson};
            pendingSessionJson = "";
            pendingSummaryJson = "";

            compressing = false;
            setInterval(HISTORY_INTERVAL_IDLE_MS);
        }
        return;
    }

    if (fetchPending)
        return;

    if (collecting) {
        // Periodically flush buffered pose snapshots to disk
        if (!writeBuffer.empty() && millis() - lastFlushMs >= HISTORY_FLUSH_INTERVAL_MS) {
            flushWriteBuffer();
        }
        collectSnapshot();
    } else {
        checkState();
        enforceLimits();
    }
}

// -- State watching (idle mode) ----------------------------------------------

void CleaningHistory::checkState() {
    fetchPending = true;
    neato.getState([this](bool ok, const RobotState& state) {
        fetchPending = false;
        if (!ok)
            return;

        bool wasCleaning = isCleaningState(prevUiState);
        bool nowCleaning = isCleaningState(state.uiState);

        // CLEANINGSUSPENDED + Charging_Cleaning means the robot is on the dock
        // recharging mid-clean and will resume automatically — treat as active
        bool isMidCleanRecharge = isSuspendedState(state.uiState) && state.robotState.indexOf("Charging_Cleaning") >= 0;

        // First poll after boot: if the robot is idle, finalize any orphan sessions
        // left by a previous crash/reboot so they don't get merged into the next clean
        if (prevUiState.isEmpty() && !nowCleaning && !isMidCleanRecharge && !recoveryAttempted) {
            recoveryAttempted = true;
            finalizeOrphanSessions();
        }

        if (!wasCleaning && (nowCleaning || isMidCleanRecharge) && !recoverCollection(state.uiState)) {
            startCollection(state.uiState);
        }

        prevUiState = state.uiState;
    });
}

bool CleaningHistory::isCleaningState(const String& uiState) {
    return uiState.indexOf("HOUSECLEANINGRUNNING") >= 0 || uiState.indexOf("HOUSECLEANINGPAUSED") >= 0 ||
           uiState.indexOf("SPOTCLEANINGRUNNING") >= 0 || uiState.indexOf("SPOTCLEANINGPAUSED") >= 0 ||
           uiState.indexOf("MANUALCLEANING") >= 0;
}

bool CleaningHistory::isPausedState(const String& uiState) {
    return uiState.indexOf("CLEANINGPAUSED") >= 0;
}

bool CleaningHistory::isDockingState(const String& uiState) {
    return uiState.indexOf("DOCKING") >= 0;
}

bool CleaningHistory::isSuspendedState(const String& uiState) {
    return uiState.indexOf("CLEANINGSUSPENDED") >= 0;
}

String CleaningHistory::cleanModeFromState(const String& uiState) {
    if (uiState.indexOf("HOUSECLEANING") >= 0)
        return "house";
    if (uiState.indexOf("SPOTCLEANING") >= 0)
        return "spot";
    if (uiState.indexOf("MANUALCLEANING") >= 0)
        return "manual";
    return "unknown";
}

void CleaningHistory::resetSession() {
    snapshotCount = 0;
    rechargeCount = 0;
    totalDistance = 0.0f;
    totalRotation = 0.0f;
    maxDistFromOrigin = 0.0f;
    errorsDuringClean = 0;
    prevHadError = false;
    hasPrevPose = false;
    prevX = 0.0f;
    prevY = 0.0f;
    prevTheta = 0.0f;
    originX = 0.0f;
    originY = 0.0f;
    visitedCells.clear();
    cleanMode = "";
    sessionStartTime = 0;
    batteryStart = -1;
    recharging = false;
    pendingSessionJson = "";
    pendingSummaryJson = "";
}

void CleaningHistory::startCollection(const String& uiState) {
    collecting = true;
    resetSession();

    cleanMode = cleanModeFromState(uiState);
    sessionStartTime = systemManager.now();

    // Create session file: /history/<epoch>.jsonl
    activeFilePath = String(HISTORY_DIR) + "/" + String(static_cast<long>(sessionStartTime)) + ".jsonl";
    activeFile = SPIFFS.open(activeFilePath, FILE_WRITE);
    if (!activeFile) {
        LOG("HIST", "Failed to create session file: %s", activeFilePath.c_str());
        collecting = false;
        return;
    }

    // Fetch battery level for session metadata, then write header
    neato.getCharger([this](bool ok, const ChargerData& charger) {
        if (ok) {
            batteryStart = charger.fuelPercent;
        }
        writeSessionHeader();
    });

    setInterval(HISTORY_INTERVAL_ACTIVE_MS);
    LOG("HIST", "Collection started (mode: %s, file: %s)", cleanMode.c_str(), activeFilePath.c_str());
    dataLogger.logGenericEvent("history_start", {{"mode", cleanMode, FIELD_STRING}});
}

void CleaningHistory::stopCollection() {
    // Flush any buffered snapshots before writing summary
    flushWriteBuffer();

    // Discard sessions that are too short to produce a useful map
    if (snapshotCount < HISTORY_MIN_SNAPSHOTS) {
        if (activeFile) {
            activeFile.close();
        }
        SPIFFS.remove(activeFilePath);

        LOG("HIST", "Session discarded (%u snapshots < %d minimum)", snapshotCount, HISTORY_MIN_SNAPSHOTS);
        dataLogger.logGenericEvent("history_discard", {{"snapshots", String(snapshotCount), FIELD_INT}});

        collecting = false;
        recharging = false;
        setInterval(HISTORY_INTERVAL_IDLE_MS);
        return;
    }

    // Fetch final battery level for summary
    neato.getCharger([this](bool ok, const ChargerData& charger) {
        int batteryEnd = ok ? charger.fuelPercent : -1;

        writeSessionSummary(batteryEnd);

        // Close the raw file
        if (activeFile) {
            activeFile.close();
        }

        collecting = false;
        recharging = false;
        setInterval(HISTORY_INTERVAL_IDLE_MS);

        float areaCovered = static_cast<float>(visitedCells.size()) * HISTORY_AREA_CELL_M * HISTORY_AREA_CELL_M;

        LOG("HIST", "Collection stopped (%u snapshots, %.1fm², %d recharges)", snapshotCount, areaCovered,
            rechargeCount);
        dataLogger.logGenericEvent("history_stop", {{"snapshots", String(snapshotCount), FIELD_INT},
                                                    {"area_m2", String(areaCovered, 1), FIELD_FLOAT},
                                                    {"recharges", String(rechargeCount), FIELD_INT},
                                                    {"battery_end", String(batteryEnd), FIELD_INT}});

        // Start non-blocking compression: raw .jsonl -> .jsonl.hs
        compressSrcPath = activeFilePath;
        compressDstPath = activeFilePath + ".hs";
        compressSrc = SPIFFS.open(compressSrcPath, FILE_READ);
        compressDst = SPIFFS.open(compressDstPath, FILE_WRITE);
        if (compressSrc && compressDst) {
            heatshrink_encoder_reset(&compressEncoder);
            compressInputDone = false;
            compressing = true;
            setInterval(HISTORY_COMPRESS_INTERVAL_MS);
            LOG("HIST", "Starting compression: %s -> %s", compressSrcPath.c_str(), compressDstPath.c_str());
        } else {
            // Compression failed - keep raw file
            if (compressSrc)
                compressSrc.close();
            if (compressDst)
                compressDst.close();
            LOG("HIST", "Compression setup failed, keeping raw file");
        }
    });
}

// -- Incremental compression (called from tick) ------------------------------

bool CleaningHistory::compressStep() {
    static const size_t CHUNK_SIZE = 512;
    uint8_t inBuf[CHUNK_SIZE];
    uint8_t outBuf[CHUNK_SIZE];

    if (!compressInputDone) {
        int bytesRead = compressSrc.read(inBuf, CHUNK_SIZE);
        if (bytesRead <= 0) {
            compressInputDone = true;
        } else {
            size_t offset = 0;
            while (offset < static_cast<size_t>(bytesRead)) {
                size_t sunk = 0;
                HSE_sink_res sres =
                        heatshrink_encoder_sink(&compressEncoder, inBuf + offset, bytesRead - offset, &sunk);
                if (sres < 0) {
                    LOG("HIST", "Heatshrink sink error");
                    compressSrc.close();
                    compressDst.close();
                    SPIFFS.remove(compressDstPath);
                    compressing = false;
                    return true;
                }
                offset += sunk;

                size_t outSz = 0;
                HSE_poll_res pres;
                do {
                    pres = heatshrink_encoder_poll(&compressEncoder, outBuf, CHUNK_SIZE, &outSz);
                    if (pres < 0) {
                        LOG("HIST", "Heatshrink poll error");
                        compressSrc.close();
                        compressDst.close();
                        SPIFFS.remove(compressDstPath);
                        compressing = false;
                        return true;
                    }
                    if (outSz > 0) {
                        compressDst.write(outBuf, outSz);
                    }
                } while (pres == HSER_POLL_MORE);
            }
        }
        return false;
    }

    // Input exhausted — finish encoding
    HSE_finish_res fres = heatshrink_encoder_finish(&compressEncoder);
    if (fres < 0) {
        LOG("HIST", "Heatshrink finish error");
        compressSrc.close();
        compressDst.close();
        SPIFFS.remove(compressDstPath);
        compressing = false;
        return true;
    }

    size_t outSz = 0;
    HSE_poll_res pres;
    do {
        pres = heatshrink_encoder_poll(&compressEncoder, outBuf, CHUNK_SIZE, &outSz);
        if (pres < 0) {
            LOG("HIST", "Heatshrink poll error during finish");
            compressSrc.close();
            compressDst.close();
            SPIFFS.remove(compressDstPath);
            compressing = false;
            return true;
        }
        if (outSz > 0) {
            compressDst.write(outBuf, outSz);
        }
    } while (pres == HSER_POLL_MORE);

    return (fres == HSER_FINISH_DONE);
}

// -- Session header/summary --------------------------------------------------

void CleaningHistory::writeSessionHeader() {
    std::vector<Field> fields = {{"type", "session", FIELD_STRING}, {"mode", cleanMode, FIELD_STRING}};
    if (sessionStartTime > 0)
        fields.push_back({"time", String(static_cast<long>(sessionStartTime)), FIELD_INT});
    if (batteryStart >= 0)
        fields.push_back({"battery", String(batteryStart), FIELD_INT});
    String json = fieldsToJson(fields);
    pendingSessionJson = json;
    writeLine(json);
}

void CleaningHistory::writeSessionSummary(int batteryEnd) {
    time_t endTime = systemManager.now();
    long duration =
            (sessionStartTime > 0 && endTime > sessionStartTime) ? static_cast<long>(endTime - sessionStartTime) : 0;
    float areaCovered = static_cast<float>(visitedCells.size()) * HISTORY_AREA_CELL_M * HISTORY_AREA_CELL_M;
    std::vector<Field> fields = {{"type", "summary", FIELD_STRING}};
    if (endTime > 0)
        fields.push_back({"time", String(static_cast<long>(endTime)), FIELD_INT});
    fields.push_back({"duration", String(duration), FIELD_INT});
    fields.push_back({"mode", cleanMode, FIELD_STRING});
    fields.push_back({"recharges", String(rechargeCount), FIELD_INT});
    fields.push_back({"snapshots", String(static_cast<int>(snapshotCount)), FIELD_INT});
    fields.push_back({"distanceTraveled", String(totalDistance, 2), FIELD_FLOAT});
    fields.push_back({"maxDistFromOrigin", String(maxDistFromOrigin, 2), FIELD_FLOAT});
    fields.push_back({"totalRotation", String(totalRotation, 1), FIELD_FLOAT});
    fields.push_back({"areaCovered", String(areaCovered, 2), FIELD_FLOAT});
    fields.push_back({"errorsDuringClean", String(errorsDuringClean), FIELD_INT});
    if (batteryStart >= 0)
        fields.push_back({"batteryStart", String(batteryStart), FIELD_INT});
    if (batteryEnd >= 0)
        fields.push_back({"batteryEnd", String(batteryEnd), FIELD_INT});
    String json = fieldsToJson(fields);
    pendingSummaryJson = json;
    writeLine(json);
}

// -- Buffered file writing ---------------------------------------------------
// Pose snapshots accumulate in writeBuffer; flushed every HISTORY_FLUSH_INTERVAL_MS
// or when the session ends (stopCollection / recovery). This reduces flash wear
// from one write every 2s to one write every 30s during a cleaning session.
// Session header, summary, and recharge markers flush immediately since they
// are rare one-shot writes where crash safety matters.

void CleaningHistory::writeLine(const String& line) {
    if (!activeFile)
        return;
    activeFile.println(line);
    activeFile.flush();
}

void CleaningHistory::bufferLine(const String& line) {
    if (!activeFile)
        return;
    writeBuffer.push_back(line);
}

void CleaningHistory::flushWriteBuffer() {
    if (writeBuffer.empty() || !activeFile)
        return;
    // Build a single string and write once to minimize SPIFFS COW metadata updates
    String batch;
    size_t total = 0;
    for (const auto& line: writeBuffer) {
        total += line.length() + 1;
    }
    batch.reserve(total);
    for (const auto& line: writeBuffer) {
        batch += line;
        batch += '\n';
    }
    activeFile.write(reinterpret_cast<const uint8_t *>(batch.c_str()), batch.length());
    activeFile.flush();
    writeBuffer.clear();
    lastFlushMs = millis();
}

// -- Snapshot collection (active mode) ---------------------------------------

static bool parsePose(const String& raw, float& x, float& y, float& theta, float& time) {
    int xPos = raw.indexOf("X=");
    int yPos = raw.indexOf("Y=");
    int tPos = raw.indexOf("Theta=");
    int tmPos = raw.indexOf("Time=");
    if (xPos < 0 || yPos < 0 || tPos < 0 || tmPos < 0)
        return false;

    x = raw.substring(xPos + 2).toFloat();
    y = raw.substring(yPos + 2).toFloat();
    theta = raw.substring(tPos + 6).toFloat();
    time = raw.substring(tmPos + 5).toFloat();
    return true;
}

// Parse a single JSONL line and update session accumulators (header, recharge, pose).
// Returns true if the line was a session header.
bool CleaningHistory::replayLine(const String& line) {
    auto fields = fieldsFromJson(line);
    if (fields.empty())
        return false;

    const Field *typeField = findField(fields, "type");
    if (typeField && typeField->value == "session") {
        const Field *modeField = findField(fields, "mode");
        if (modeField && !modeField->value.isEmpty())
            cleanMode = modeField->value;

        const Field *timeField = findField(fields, "time");
        if (timeField) {
            long t = timeField->value.toInt();
            if (t > 0)
                sessionStartTime = static_cast<time_t>(t);
        }

        const Field *battField = findField(fields, "battery");
        if (battField)
            batteryStart = battField->value.toInt();
        return true;
    }

    if (typeField && typeField->value == "recharge") {
        rechargeCount++;
        return false;
    }

    // Pose snapshot line
    const Field *xf = findField(fields, "x");
    const Field *yf = findField(fields, "y");
    const Field *tf = findField(fields, "t");
    if (xf && yf && tf) {
        updateAccumulators(xf->value.toFloat(), yf->value.toFloat(), tf->value.toFloat());
        snapshotCount++;
    }
    return false;
}

bool CleaningHistory::recoverCollection(const String& uiState) {
    // Only attempt recovery once after boot — avoid scanning filesystem on every clean start
    if (recoveryAttempted)
        return false;
    recoveryAttempted = true;

    File root = SPIFFS.open(HISTORY_DIR);
    if (!root || !root.isDirectory())
        return false;

    // Collect all history filenames (both .jsonl and .jsonl.hs) and sort descending
    // (newest first). Then walk from newest to oldest: collect orphan .jsonl files
    // until we hit a compressed .jsonl.hs — that marks a completed session boundary,
    // so everything older belongs to previous cleans and should not be touched.
    // Also clean up any raw .jsonl that has a .jsonl.hs counterpart (stale leftover).
    std::vector<String> allFiles;
    File entry = root.openNextFile();
    while (entry) {
        String path = String(entry.path());
        if (path.endsWith(".jsonl") || path.endsWith(".jsonl.hs"))
            allFiles.push_back(path);
        entry = root.openNextFile();
    }

    std::sort(allFiles.begin(), allFiles.end(), [](const String& a, const String& b) { return a > b; });

    std::vector<String> orphans;
    for (const auto& path: allFiles) {
        if (path.endsWith(".jsonl.hs")) {
            // Hit a compressed file — this is the boundary of a completed session.
            // Stop collecting; everything older is from previous cleans.
            break;
        }
        // Raw .jsonl — check if a compressed copy exists (stale leftover)
        if (SPIFFS.exists(path + ".hs")) {
            SPIFFS.remove(path);
            LOG("HIST", "Removed stale raw file: %s (compressed copy exists)", path.c_str());
            continue;
        }
        // Check if it has a summary (completed but not yet compressed)
        String firstLine, lastLine;
        readFirstLastLines(path, false, firstLine, lastLine);
        if (lastLine.indexOf("\"type\":\"summary\"") >= 0)
            continue;
        orphans.push_back(path);
    }

    if (orphans.empty())
        return false;

    // orphans are newest-first; reverse to get chronological order (oldest first)
    std::reverse(orphans.begin(), orphans.end());

    // The oldest orphan becomes the target; newer ones are merged into it then deleted
    String targetPath = orphans[0];

    if (orphans.size() > 1) {
        File target = SPIFFS.open(targetPath, FILE_APPEND);
        if (target) {
            for (size_t i = 1; i < orphans.size(); i++) {
                File src = SPIFFS.open(orphans[i], FILE_READ);
                if (!src)
                    continue;
                while (src.available()) {
                    String line = src.readStringUntil('\n');
                    line.trim();
                    if (line.isEmpty())
                        continue;
                    // Skip duplicate session headers from newer orphans
                    if (line.indexOf("\"type\":\"session\"") >= 0)
                        continue;
                    target.println(line);
                }
                src.close();
                SPIFFS.remove(orphans[i]);
                LOG("HIST", "Merged orphan %s into %s", orphans[i].c_str(), targetPath.c_str());
            }
            target.flush();
            target.close();
        }
    }

    // Now replay the merged file to rebuild accumulators
    resetSession();
    cleanMode = cleanModeFromState(uiState);
    activeFilePath = targetPath;

    File recoveryFile = SPIFFS.open(targetPath, FILE_READ);
    if (!recoveryFile)
        return false;

    bool hasSessionHeader = false;
    while (recoveryFile.available()) {
        String line = recoveryFile.readStringUntil('\n');
        line.trim();
        if (line.isEmpty())
            continue;
        if (replayLine(line))
            hasSessionHeader = true;
    }
    recoveryFile.close();

    // Fall back to extracting start time from the filename
    if (sessionStartTime <= 0) {
        String name = targetPath;
        int slash = name.lastIndexOf('/');
        if (slash >= 0)
            name = name.substring(slash + 1);
        int dot = name.indexOf('.');
        if (dot > 0)
            sessionStartTime = static_cast<time_t>(name.substring(0, dot).toInt());
    }

    activeFile = SPIFFS.open(activeFilePath, FILE_APPEND);
    if (!activeFile) {
        activeFilePath = "";
        resetSession();
        return false;
    }

    collecting = true;
    setInterval(HISTORY_INTERVAL_ACTIVE_MS);
    LOG("HIST", "Recovered session: %s (%u snapshots, %zu orphans merged)", activeFilePath.c_str(), snapshotCount,
        orphans.size());
    dataLogger.logGenericEvent("history_recover", {{"path", activeFilePath, FIELD_STRING},
                                                   {"snapshots", String(snapshotCount), FIELD_INT},
                                                   {"orphans", String(static_cast<int>(orphans.size())), FIELD_INT}});

    if (!hasSessionHeader) {
        writeSessionHeader();
    }
    return true;
}

void CleaningHistory::finalizeOrphanSessions() {
    // Called once at boot when the robot is idle. Finds orphan .jsonl files
    // (no summary line, no compressed counterpart) and finalizes them: replay
    // to compute summary stats, append summary line, then start compression.
    File root = SPIFFS.open(HISTORY_DIR);
    if (!root || !root.isDirectory())
        return;

    std::vector<String> allFiles;
    File entry = root.openNextFile();
    while (entry) {
        String path = String(entry.path());
        if (path.endsWith(".jsonl") || path.endsWith(".jsonl.hs"))
            allFiles.push_back(path);
        entry = root.openNextFile();
    }

    std::sort(allFiles.begin(), allFiles.end(), [](const String& a, const String& b) { return a > b; });

    std::vector<String> orphans;
    for (const auto& path: allFiles) {
        if (path.endsWith(".jsonl.hs"))
            break; // Completed session boundary
        if (SPIFFS.exists(path + ".hs")) {
            SPIFFS.remove(path);
            continue;
        }
        String firstLine, lastLine;
        readFirstLastLines(path, false, firstLine, lastLine);
        if (lastLine.indexOf("\"type\":\"summary\"") >= 0)
            continue;
        orphans.push_back(path);
    }

    if (orphans.empty())
        return;

    // Finalize each orphan: replay to rebuild stats, write summary, compress
    for (const auto& orphanPath: orphans) {
        resetSession();

        // Replay to rebuild accumulators
        File f = SPIFFS.open(orphanPath, FILE_READ);
        if (!f)
            continue;
        while (f.available()) {
            String line = f.readStringUntil('\n');
            line.trim();
            if (line.isEmpty())
                continue;
            replayLine(line);
        }
        f.close();

        // Extract start time from filename if not found in header
        if (sessionStartTime <= 0) {
            String name = orphanPath;
            int slash = name.lastIndexOf('/');
            if (slash >= 0)
                name = name.substring(slash + 1);
            int dot = name.indexOf('.');
            if (dot > 0)
                sessionStartTime = static_cast<time_t>(name.substring(0, dot).toInt());
        }

        // Open for append and write summary
        activeFilePath = orphanPath;
        activeFile = SPIFFS.open(orphanPath, FILE_APPEND);
        if (!activeFile)
            continue;

        writeSessionSummary(-1); // Battery end unknown for interrupted sessions
        activeFile.close();

        LOG("HIST", "Finalized orphan: %s (%u snapshots)", orphanPath.c_str(), snapshotCount);
        dataLogger.logGenericEvent("history_finalize", {{"path", orphanPath, FIELD_STRING},
                                                        {"snapshots", String(snapshotCount), FIELD_INT}});

        // Queue compression (only one at a time — first orphan wins, rest will
        // be picked up on next boot or left as uncompressed)
        if (!compressing) {
            compressSrcPath = orphanPath;
            compressDstPath = orphanPath + ".hs";
            compressSrc = SPIFFS.open(compressSrcPath, FILE_READ);
            compressDst = SPIFFS.open(compressDstPath, FILE_WRITE);
            if (compressSrc && compressDst) {
                heatshrink_encoder_reset(&compressEncoder);
                compressInputDone = false;
                compressing = true;
                setInterval(HISTORY_COMPRESS_INTERVAL_MS);
            } else {
                if (compressSrc)
                    compressSrc.close();
                if (compressDst)
                    compressDst.close();
            }
        }
    }

    activeFilePath = "";
    activeFile = File();
}

void CleaningHistory::collectSnapshot() {
    fetchPending = true;

    neato.getState([this](bool stateOk, const RobotState& state) {
        if (stateOk) {
            prevUiState = state.uiState;

            bool isDocking = isDockingState(state.uiState);
            bool isCleaning = isCleaningState(state.uiState);
            bool isSuspended = isSuspendedState(state.uiState);
            bool isChargingMidClean = state.robotState.indexOf("Charging_Cleaning") >= 0;

            // Mid-clean recharge: robot returns to base to charge before resuming.
            // The UI state can be DOCKING (on the way back) or CLEANINGSUSPENDED
            // (already on the dock and charging). Both combined with the
            // Charging_Cleaning robot state indicate a recharge-and-resume cycle.
            if ((isDocking || isSuspended) && isChargingMidClean) {
                if (!recharging) {
                    recharging = true;
                    rechargeCount++;
                    LOG("HIST", "Recharge #%d detected — pausing collection", rechargeCount);
                    dataLogger.logGenericEvent("history_recharge_start", {{"count", String(rechargeCount), FIELD_INT}});

                    if (hasPrevPose) {
                        writeLine(fieldsToJson({{"type", "recharge", FIELD_STRING},
                                                {"x", String(prevX, 3), FIELD_FLOAT},
                                                {"y", String(prevY, 3), FIELD_FLOAT}}));
                    }
                }
                fetchPending = false;
                return;
            }

            if (recharging && isCleaning) {
                recharging = false;
                LOG("HIST", "Recharge done — resuming collection");
                dataLogger.logGenericEvent("history_recharge_end", {});
            }

            if (!isCleaning && !isDocking && !isSuspended) {
                fetchPending = false;
                stopCollection();
                return;
            }

            // Paused — keep session open but skip snapshot collection
            if (isPausedState(state.uiState)) {
                fetchPending = false;
                return;
            }
        }

        neato.getErr([this](bool errOk, const ErrorData& err) {
            if (errOk) {
                if (err.hasError && !prevHadError) {
                    errorsDuringClean++;
                }
                prevHadError = err.hasError;
            }

            if (recharging) {
                fetchPending = false;
                return;
            }

            neato.getRobotPos(true, [this](bool posOk, const RobotPosData& pos) {
                fetchPending = false;
                if (!posOk)
                    return;

                float x, y, theta, time;
                if (!parsePose(pos.raw, x, y, theta, time)) {
                    LOG("HIST", "Failed to parse pose");
                    return;
                }

                writeSnapshot(x, y, theta, time);
            });
        });
    });
}

void CleaningHistory::updateAccumulators(float x, float y, float theta) {
    if (!hasPrevPose) {
        originX = x;
        originY = y;
        prevX = x;
        prevY = y;
        prevTheta = theta;
        hasPrevPose = true;
    } else {
        float dx = x - prevX;
        float dy = y - prevY;
        totalDistance += sqrtf(dx * dx + dy * dy);

        float dTheta = theta - prevTheta;
        if (dTheta > 180.0f)
            dTheta -= 360.0f;
        if (dTheta < -180.0f)
            dTheta += 360.0f;
        totalRotation += fabsf(dTheta);

        prevX = x;
        prevY = y;
        prevTheta = theta;
    }

    float dox = x - originX;
    float doy = y - originY;
    float distFromOrigin = sqrtf(dox * dox + doy * doy);
    if (distFromOrigin > maxDistFromOrigin) {
        maxDistFromOrigin = distFromOrigin;
    }

    int ix = static_cast<int>(floorf(x / HISTORY_AREA_CELL_M));
    int iy = static_cast<int>(floorf(y / HISTORY_AREA_CELL_M));
    uint32_t cellKey = (static_cast<uint32_t>(ix & 0xFFFF) << 16) | static_cast<uint32_t>(iy & 0xFFFF);
    visitedCells.insert(cellKey);
}

void CleaningHistory::writeSnapshot(float x, float y, float theta, float time) {
    // Localization resets to origin right before session ends — drop if the
    // robot was far from origin (genuine return-to-base passes through gradually)
    if (hasPrevPose && fabsf(x) < 0.001f && fabsf(y) < 0.001f && fabsf(theta) < 0.1f) {
        float prevDist = sqrtf(prevX * prevX + prevY * prevY);
        if (prevDist > 1.0f) {
            return;
        }
    }

    String line = "{\"x\":" + String(x, 3) + ",\"y\":" + String(y, 3) + ",\"t\":" + String(theta, 1) +
                  ",\"ts\":" + String(time, 1) + "}";

    updateAccumulators(x, y, theta);
    bufferLine(line);
    snapshotCount++;

    if (snapshotCount % 10 == 0) {
        LOG("HIST", "Snapshot #%u (%.1fm traveled, pose: %.2f,%.2f,%.0f)", snapshotCount, totalDistance, x, y, theta);
    }
}

// -- Saved floorplans ---------------------------------------------------------

bool CleaningHistory::isStableHistoryFilename(const String& filename) {
    if (filename.isEmpty() || filename.indexOf('/') >= 0 || filename.indexOf('\\') >= 0 ||
        filename.indexOf("..") >= 0) {
        return false;
    }
    return filename.endsWith(".jsonl") || filename.endsWith(".jsonl.hs");
}

void CleaningHistory::ensureFloorplansLoaded() {
    if (floorplansLoaded)
        return;

    floorplansLoaded = true;
    floorplans.clear();

    // Recover the previous registry if power was lost between the two rename steps.
    if (!SPIFFS.exists(FLOORPLAN_REGISTRY_PATH) && SPIFFS.exists(FLOORPLAN_REGISTRY_BAK_PATH)) {
        SPIFFS.rename(FLOORPLAN_REGISTRY_BAK_PATH, FLOORPLAN_REGISTRY_PATH);
    }

    File file = SPIFFS.open(FLOORPLAN_REGISTRY_PATH, FILE_READ);
    if (!file)
        return;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.isEmpty())
            continue;

        auto fields = fieldsFromJson(line);
        const Field *sessionField = findField(fields, "session");
        const Field *nameField = findField(fields, "name");

        if (!sessionField || !nameField)
            continue;

        String session = sessionField->value;
        String name = nameField->value;
        name.trim();

        if (!isStableHistoryFilename(session) || name.isEmpty())
            continue;

        // Only load live references. A formatted/corrupt filesystem cannot leave
        // a phantom floorplan entry pointing at missing map bytes.
        String path = String(HISTORY_DIR) + "/" + session;
        if (!SPIFFS.exists(path))
            continue;

        floorplans[session] = name;
    }

    file.close();
}

bool CleaningHistory::persistFloorplans() {
    floorplanError = "";

    SPIFFS.remove(FLOORPLAN_REGISTRY_TMP_PATH);

    File tmp = SPIFFS.open(FLOORPLAN_REGISTRY_TMP_PATH, FILE_WRITE);
    if (!tmp) {
        floorplanError = "registry_open_failed";
        return false;
    }

    for (const auto& item: floorplans) {
        tmp.println(fieldsToJson({{"session", item.first, FIELD_STRING}, {"name", item.second, FIELD_STRING}}));
    }

    tmp.flush();
    tmp.close();

    SPIFFS.remove(FLOORPLAN_REGISTRY_BAK_PATH);

    bool hadOld = SPIFFS.exists(FLOORPLAN_REGISTRY_PATH);
    if (hadOld && !SPIFFS.rename(FLOORPLAN_REGISTRY_PATH, FLOORPLAN_REGISTRY_BAK_PATH)) {
        SPIFFS.remove(FLOORPLAN_REGISTRY_TMP_PATH);
        floorplanError = "registry_backup_failed";
        return false;
    }

    if (!SPIFFS.rename(FLOORPLAN_REGISTRY_TMP_PATH, FLOORPLAN_REGISTRY_PATH)) {
        if (hadOld && SPIFFS.exists(FLOORPLAN_REGISTRY_BAK_PATH)) {
            SPIFFS.rename(FLOORPLAN_REGISTRY_BAK_PATH, FLOORPLAN_REGISTRY_PATH);
        }
        floorplanError = "registry_commit_failed";
        return false;
    }

    SPIFFS.remove(FLOORPLAN_REGISTRY_BAK_PATH);
    return true;
}

bool CleaningHistory::isFloorplan(const String& filename) {
    ensureFloorplansLoaded();
    return floorplans.find(filename) != floorplans.end();
}

String CleaningHistory::listFloorplansJson() {
    ensureFloorplansLoaded();

    String json = "[";
    bool first = true;

    for (const auto& item: floorplans) {
        String path = String(HISTORY_DIR) + "/" + item.first;
        if (!SPIFFS.exists(path))
            continue;

        File file = SPIFFS.open(path, FILE_READ);
        size_t size = file ? file.size() : 0;
        if (file)
            file.close();

        String firstLine;
        String lastLine;
        readFirstLastLines(path, item.first.endsWith(".hs"), firstLine, lastLine);

        if (!first)
            json += ",";
        first = false;

        json += "{\"id\":\"" + jsonEscape(item.first) + "\",\"session\":\"" + jsonEscape(item.first) +
                "\",\"name\":\"" + jsonEscape(item.second) + "\",\"size\":" +
                String(static_cast<unsigned long>(size)) + ",\"compressed\":" +
                String(item.first.endsWith(".hs") ? "true" : "false");

        if (lastLine.indexOf("\"type\":\"summary\"") >= 0) {
            json += ",\"summary\":" + lastLine;
        } else {
            json += ",\"summary\":null";
        }

        json += "}";
    }

    json += "]";
    return json;
}

bool CleaningHistory::saveFloorplan(const String& filename, const String& requestedName) {
    ensureFloorplansLoaded();
    floorplanError = "";

    if (!isStableHistoryFilename(filename)) {
        floorplanError = "invalid_session";
        return false;
    }

    String name = requestedName;
    name.trim();

    if (name.isEmpty()) {
        floorplanError = "name_required";
        return false;
    }

    if (name.length() > FLOORPLAN_NAME_MAX_BYTES) {
        floorplanError = "name_too_long";
        return false;
    }

    String path = String(HISTORY_DIR) + "/" + filename;
    if (!SPIFFS.exists(path)) {
        floorplanError = "session_not_found";
        return false;
    }

    // Never pin a recording or a file that is still being compressed.
    if ((collecting && activeFilePath == path) ||
        (compressing && (compressSrcPath == path || compressDstPath == path))) {
        floorplanError = "session_busy";
        return false;
    }

    String firstLine;
    String lastLine;
    readFirstLastLines(path, filename.endsWith(".hs"), firstLine, lastLine);
    if (lastLine.indexOf("\"type\":\"summary\"") < 0) {
        floorplanError = "session_not_complete";
        return false;
    }

    auto existing = floorplans.find(filename);
    bool isNew = existing == floorplans.end();

    if (isNew && floorplans.size() >= FLOORPLAN_MAX_COUNT) {
        floorplanError = "floorplan_limit";
        return false;
    }

    String oldName;
    if (!isNew)
        oldName = existing->second;

    floorplans[filename] = name;

    if (!persistFloorplans()) {
        if (isNew) {
            floorplans.erase(filename);
        } else {
            floorplans[filename] = oldName;
        }
        return false;
    }

    LOG("HIST", "Floorplan saved: %s -> %s", filename.c_str(), name.c_str());
    return true;
}

bool CleaningHistory::deleteFloorplan(const String& filename) {
    ensureFloorplansLoaded();
    floorplanError = "";

    auto it = floorplans.find(filename);
    if (it == floorplans.end()) {
        floorplanError = "floorplan_not_found";
        return false;
    }

    String oldName = it->second;
    floorplans.erase(it);

    if (!persistFloorplans()) {
        floorplans[filename] = oldName;
        return false;
    }

    LOG("HIST", "Floorplan unpinned: %s", filename.c_str());
    return true;
}

// -- Storage enforcement (mirrors DataLogger::enforceLimits) -----------------

void CleaningHistory::enforceLimits() {
    ensureFloorplansLoaded();

    // Count session files, sum directory size, and find the oldest unpinned
    // session in one pass. Saved floorplans remain part of the real byte/file
    // budget, but are never selected as eviction candidates.
    int fileCount = 0;
    size_t histDirBytes = 0;
    String oldest;
    File root = SPIFFS.open(HISTORY_DIR);
    if (!root || !root.isDirectory())
        return;

    File entry = root.openNextFile();
    while (entry) {
        String name = String(entry.name());
        histDirBytes += entry.size();
        if (name.endsWith(".jsonl") || name.endsWith(".jsonl.hs")) {
            fileCount++;
            if (!isFloorplan(name) && (oldest.isEmpty() || name < oldest)) {
                oldest = name;
            }
        }
        entry = root.openNextFile();
    }

    if (oldest.isEmpty())
        return;

    // History budget: total filesystem cap minus non-history data, floored at minimum reserve
    size_t total = SPIFFS.totalBytes();
    size_t globalCap = (total * HISTORY_MAX_FS_PERCENT) / 100;
    size_t nonHistBytes = SPIFFS.usedBytes() > histDirBytes ? SPIFFS.usedBytes() - histDirBytes : 0;
    size_t available = globalCap > nonHistBytes ? globalCap - nonHistBytes : 0;
    size_t minReserved = (total * HISTORY_MIN_FS_PERCENT) / 100;
    size_t histBudget = available > minReserved ? available : minReserved;

    if (histDirBytes > histBudget || fileCount > HISTORY_MAX_FILES) {
        String fullPath = String(HISTORY_DIR) + "/" + oldest;
        LOG("HIST", "Limit: deleting %s (files=%d, histBytes=%u/%u)", fullPath.c_str(), fileCount, histDirBytes,
            histBudget);
        SPIFFS.remove(fullPath);
        metaCache.erase(oldest);
    }
}

// -- File management (for API) -----------------------------------------------

void CleaningHistory::readFirstLastLines(const String& path, bool compressed, String& firstLine, String& lastLine) {
    firstLine = "";
    lastLine = "";

    if (compressed) {
        // Stream-decompress keeping only the first and last lines in memory
        // to avoid buffering the entire file (large sessions can exceed 100KB
        // decompressed which exhausts ESP32-C3 heap).
        File f = SPIFFS.open(path, FILE_READ);
        if (!f)
            return;
        CompressedLogReader reader(std::move(f));
        uint8_t buf[256];
        size_t n;
        bool firstDone = false;
        // Current incomplete line being assembled from chunks
        String current;
        while ((n = reader.read(buf, sizeof(buf))) > 0) {
            for (size_t i = 0; i < n; i++) {
                char c = static_cast<char>(buf[i]);
                if (c == '\n') {
                    current.trim();
                    if (current.length() > 0) {
                        if (!firstDone) {
                            firstLine = current;
                            firstDone = true;
                        }
                        // Always overwrite lastLine with the most recent non-empty line
                        lastLine = current;
                    }
                    current = "";
                } else {
                    current += c;
                }
            }
        }
        // Handle final chunk without trailing newline
        current.trim();
        if (current.length() > 0) {
            if (!firstDone) {
                firstLine = current;
            }
            lastLine = current;
        }
    } else {
        // Plain .jsonl — read first line directly, seek backward for last line
        File f = SPIFFS.open(path, FILE_READ);
        if (!f)
            return;
        // Read first line
        firstLine = f.readStringUntil('\n');
        firstLine.trim();
        // Seek backward from end to find last line
        size_t fileSize = f.size();
        if (fileSize < 2) {
            f.close();
            return;
        }
        int pos = static_cast<int>(fileSize) - 2; // Skip trailing newline
        while (pos >= 0) {
            f.seek(pos);
            char c = static_cast<char>(f.read());
            if (c == '\n') {
                break;
            }
            pos--;
        }
        // pos is at the newline before last line, or -1 if only one line
        if (pos < 0) {
            f.close();
            return; // Only one line
        }
        f.seek(pos + 1);
        lastLine = f.readStringUntil('\n');
        lastLine.trim();
        f.close();
    }
}

std::vector<HistorySessionInfo> CleaningHistory::listSessions() {
    std::vector<HistorySessionInfo> result;

    File root = SPIFFS.open(HISTORY_DIR);
    if (!root || !root.isDirectory())
        return result;

    File entry = root.openNextFile();
    while (entry) {
        String fullPath = String(entry.path());
        String name = fullPath;
        int lastSlash = name.lastIndexOf('/');
        if (lastSlash >= 0)
            name = name.substring(lastSlash + 1);

        if (name.endsWith(".jsonl") || name.endsWith(".jsonl.hs")) {
            // During compression both raw .jsonl and partial .jsonl.hs exist —
            // skip both until compression finishes and the raw source is deleted
            if (compressing && (fullPath == compressSrcPath || fullPath == compressDstPath)) {
                entry = root.openNextFile();
                continue;
            }

            // Skip the actively collecting file during disk enumeration.
            // We will manually append it at the end to avoid thread-safety /
            // concurrent modification issues with filesystem iterators.
            if (collecting && fullPath == activeFilePath) {
                entry = root.openNextFile();
                continue;
            }

            HistorySessionInfo info;
            info.name = name;
            info.size = entry.size();
            info.compressed = name.endsWith(".hs");

            // Use cached metadata if available (avoids decompressing .hs files)
            auto cached = metaCache.find(name);
            if (cached != metaCache.end()) {
                info.session = cached->second.session;
                info.summary = cached->second.summary;
            } else {
                String firstLine, lastLine;
                readFirstLastLines(fullPath, info.compressed, firstLine, lastLine);
                info.session = firstLine;
                if (lastLine.indexOf("\"type\":\"summary\"") >= 0) {
                    info.summary = lastLine;
                }
                // Cache for subsequent requests
                metaCache[name] = {info.session, info.summary};
            }

            result.push_back(info);
        }
        entry = root.openNextFile();
    }

    // Always append the active session from memory to ensure exactly one entry
    if (collecting && !activeFilePath.isEmpty()) {
        String name = activeFilePath;
        int lastSlash = name.lastIndexOf('/');
        if (lastSlash >= 0)
            name = name.substring(lastSlash + 1);

        HistorySessionInfo info;
        info.name = name;
        info.size = activeFile ? activeFile.size() : 0;
        info.compressed = false;
        info.recording = true;

        String sessionJson = "{\"type\":\"session\",\"mode\":\"" + cleanMode + "\"";
        if (sessionStartTime > 0)
            sessionJson += ",\"time\":" + String(static_cast<long>(sessionStartTime));
        if (batteryStart >= 0)
            sessionJson += ",\"battery\":" + String(batteryStart);
        sessionJson += "}";
        info.session = sessionJson;
        // No summary for active session

        result.push_back(info);
    }

    return result;
}

std::shared_ptr<LogReader> CleaningHistory::readSession(const String& filename) {
    String path = String(HISTORY_DIR) + "/" + filename;

    // Refuse to serve files involved in compression (partial .hs is corrupt)
    if (compressing && (path == compressSrcPath || path == compressDstPath))
        return nullptr;

    if (!SPIFFS.exists(path))
        return nullptr;

    File f = SPIFFS.open(path, FILE_READ);
    if (!f)
        return nullptr;

    if (filename.endsWith(".hs")) {
        return std::make_shared<CompressedLogReader>(std::move(f));
    }
    return std::make_shared<PlainLogReader>(std::move(f));
}

bool CleaningHistory::deleteSession(const String& filename) {
    if (isFloorplan(filename)) {
        floorplanError = "floorplan_protected";
        return false;
    }

    String path = String(HISTORY_DIR) + "/" + filename;
    if (!SPIFFS.exists(path))
        return false;
    metaCache.erase(filename);
    return SPIFFS.remove(path);
}

void CleaningHistory::deleteAllSessions() {
    ensureFloorplansLoaded();

    File root = SPIFFS.open(HISTORY_DIR);
    if (!root || !root.isDirectory())
        return;

    std::vector<String> paths;
    File entry = root.openNextFile();
    while (entry) {
        paths.push_back(String(entry.path()));
        entry = root.openNextFile();
    }

    size_t deleted = 0;
    size_t protectedCount = 0;

    for (const auto& p: paths) {
        String name = p;
        int slash = name.lastIndexOf('/');
        if (slash >= 0)
            name = name.substring(slash + 1);

        if (isFloorplan(name)) {
            protectedCount++;
            continue;
        }

        if (SPIFFS.remove(p)) {
            metaCache.erase(name);
            deleted++;
        }
    }

    LOG("HIST", "Deleted %u history files, preserved %u floorplans", deleted, protectedCount);
}

// -- Session import (compress-on-write from browser upload) -------------------

bool CleaningHistory::beginImport(const String& filename) {
    importError = "";

    if (importing) {
        importError = "Import already in progress";
        return false;
    }
    if (collecting) {
        importError = "Cannot import while recording";
        return false;
    }
    if (compressing) {
        importError = "Cannot import while compressing";
        return false;
    }

    // Validate filename pattern: <digits>.jsonl
    if (!filename.endsWith(".jsonl")) {
        importError = "Invalid filename";
        return false;
    }

    // Check for duplicate (both raw and compressed)
    String rawPath = String(HISTORY_DIR) + "/" + filename;
    String hsPath = rawPath + ".hs";
    if (SPIFFS.exists(rawPath) || SPIFFS.exists(hsPath)) {
        importError = "Session already exists";
        return false;
    }

    // Open destination file for compressed output
    importFilePath = hsPath;
    importFile = SPIFFS.open(importFilePath, FILE_WRITE);
    if (!importFile) {
        importError = "Failed to create file";
        importFilePath = "";
        return false;
    }

    heatshrink_encoder_reset(&importEncoder);
    importBytesReceived = 0;
    importing = true;
    LOG("HIST", "Import started: %s", importFilePath.c_str());
    return true;
}

bool CleaningHistory::writeImportChunk(const uint8_t *data, size_t len) {
    if (!importing || !importFile) {
        importError = "No import in progress";
        return false;
    }

    importBytesReceived += len;
    if (importBytesReceived > HISTORY_IMPORT_MAX_BYTES) {
        importError = "File too large";
        importFile.close();
        SPIFFS.remove(importFilePath);
        importing = false;
        return false;
    }

    static const size_t OUT_BUF_SIZE = 512;
    uint8_t outBuf[OUT_BUF_SIZE];
    size_t offset = 0;

    while (offset < len) {
        size_t sunk = 0;
        HSE_sink_res sres = heatshrink_encoder_sink(&importEncoder, data + offset, len - offset, &sunk);
        if (sres < 0) {
            importError = "Compression error";
            importFile.close();
            SPIFFS.remove(importFilePath);
            importing = false;
            return false;
        }
        offset += sunk;

        // Drain compressed output
        size_t outSz = 0;
        HSE_poll_res pres;
        do {
            pres = heatshrink_encoder_poll(&importEncoder, outBuf, OUT_BUF_SIZE, &outSz);
            if (pres < 0) {
                importError = "Compression error";
                importFile.close();
                SPIFFS.remove(importFilePath);
                importing = false;
                return false;
            }
            if (outSz > 0) {
                importFile.write(outBuf, outSz);
            }
        } while (pres == HSER_POLL_MORE);
    }
    return true;
}

bool CleaningHistory::endImport() {
    if (!importing || !importFile) {
        importError = "No import in progress";
        return false;
    }

    static const size_t OUT_BUF_SIZE = 512;
    uint8_t outBuf[OUT_BUF_SIZE];

    // Finish the encoder — may require multiple poll rounds
    bool done = false;
    while (!done) {
        HSE_finish_res fres = heatshrink_encoder_finish(&importEncoder);
        if (fres < 0) {
            importError = "Compression finish error";
            importFile.close();
            SPIFFS.remove(importFilePath);
            importing = false;
            return false;
        }
        done = (fres == HSER_FINISH_DONE);

        size_t outSz = 0;
        HSE_poll_res pres;
        do {
            pres = heatshrink_encoder_poll(&importEncoder, outBuf, OUT_BUF_SIZE, &outSz);
            if (pres < 0) {
                importError = "Compression finish error";
                importFile.close();
                SPIFFS.remove(importFilePath);
                importing = false;
                return false;
            }
            if (outSz > 0) {
                importFile.write(outBuf, outSz);
            }
        } while (pres == HSER_POLL_MORE);
    }

    importFile.close();
    importing = false;
    LOG("HIST", "Import complete: %s", importFilePath.c_str());
    dataLogger.logGenericEvent("history_import", {{"path", importFilePath, FIELD_STRING}});

    // Delete oldest sessions if storage budget exceeded
    enforceLimits();

    return true;
}

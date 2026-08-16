#include "mapping_manager.h"
#include "config.h"
#include "json_fields.h"

MappingManager::MappingManager(NeatoSerial& serial) : LoopTask(0), serial(serial) {
    TaskRegistry::add(this);
}

const char *MappingManager::stateName(MappingState value) {
    switch (value) {
        case MappingState::IDLE:
            return "idle";
        case MappingState::STARTING:
            return "starting";
        case MappingState::DISABLING_CLEANING:
            return "disabling_cleaning";
        case MappingState::MAPPING:
            return "mapping";
        case MappingState::RETURNING:
            return "returning";
        case MappingState::RESTORING:
            return "restoring";
        case MappingState::ERROR:
            return "error";
    }
    return "unknown";
}

bool MappingManager::isActive() const {
    return state != MappingState::IDLE && state != MappingState::ERROR;
}

bool MappingManager::control(const String& action, std::function<void(bool)> callback) {
    if (action == "start")
        return start(callback);

    if (action == "stop")
        return requestReturn(callback);

    return false;
}

bool MappingManager::start(std::function<void(bool)> callback) {
    if ((state != MappingState::IDLE && state != MappingState::ERROR) || cleaningDisabled)
        return false;

    LOG("MAP", "Starting native mapping run");

    state = MappingState::STARTING;
    pollPending = false;
    cleaningDisabled = false;
    leftDock = false;
    stopRequested = false;

    startedAtMs = millis();
    mappingStartedAtMs = 0;
    returnStartedAtMs = 0;

    lastError = "";
    lastResult = "";
    lastUiState = "";
    lastRobotState = "";
    pendingResult = "";
    pendingError = "";

    startCallback = callback;

    // Mapping must begin from the dock. Invalidate caches so this preflight
    // reads the physical charger/state instead of a stale dashboard value.
    serial.invalidateAll();

    serial.getChargerFresh([this](bool ok, const ChargerData& charger) {
        if (state != MappingState::STARTING)
            return;

        if (!ok) {
            failStart("charger_unavailable");
            return;
        }

        if (!charger.extPwrPresent) {
            failStart("robot_not_docked");
            return;
        }

        serial.getState([this](bool stateOk, const RobotState& robotState) {
            if (state != MappingState::STARTING)
                return;

            if (!stateOk) {
                failStart("state_unavailable");
                return;
            }

            lastUiState = robotState.uiState;
            lastRobotState = robotState.robotState;

            // A docked D3-D7 normally reports either IDLE or STANDBY,
            // depending on firmware/power state. Physical dock contact was
            // already verified above, so both are valid mapping start states.
            const bool readyOnDock =
                    robotState.uiState.indexOf("IDLE") >= 0 ||
                    robotState.uiState.indexOf("STANDBY") >= 0;

            if (!readyOnDock) {
                failStart("robot_not_idle");
                return;
            }

            // Use the robot's native authenticated House Clean event. The D6
            // keeps ownership of SLAM, navigation, obstacle handling and docking.
            serial.clean("house", [this](bool cleanOk) {
                if (!cleanOk && state == MappingState::STARTING) {
                    failStart("house_start_failed");
                }
            });
        });
    });

    return true;
}

bool MappingManager::requestReturn(std::function<void(bool)> callback) {
    if (state == MappingState::STARTING || state == MappingState::DISABLING_CLEANING) {
        stopRequested = true;
        if (callback)
            callback(true);
        return true;
    }

    if (state == MappingState::MAPPING) {
        sendDock("stopped", "", callback);
        return true;
    }

    // Stop is idempotent once the return sequence has already started.
    if (state == MappingState::RETURNING || state == MappingState::RESTORING) {
        if (callback)
            callback(true);
        return true;
    }

    return false;
}

void MappingManager::finishStart(bool ok) {
    auto callback = startCallback;
    startCallback = nullptr;

    if (callback)
        callback(ok);
}

void MappingManager::failStart(const String& error) {
    LOG("MAP", "Start failed: %s", error.c_str());

    lastError = error;
    lastResult = "failed";
    state = MappingState::ERROR;

    if (startedAtMs > 0)
        lastDurationSeconds = (millis() - startedAtMs) / 1000;

    finishStart(false);
}

void MappingManager::startDisablingCleaning() {
    if (state != MappingState::STARTING)
        return;

    LOG("MAP", "House navigation active, disabling cleaning function");

    state = MappingState::DISABLING_CLEANING;

    // Be conservative as soon as CleaningDisable is requested. A UART timeout
    // or desync does not prove the robot ignored the command; it may already
    // have disabled the motors. Keeping this flag set guarantees every failure
    // path still attempts CleaningEnable before the run is considered finished.
    cleaningDisabled = true;

    serial.setCleaningEnabled(false, [this](bool ok) {
        if (state != MappingState::DISABLING_CLEANING)
            return;

        if (!ok) {
            LOG("MAP", "CleaningDisable unconfirmed, returning to dock and restoring");

            finishStart(false);
            sendDock("failed", "cleaning_disable_unconfirmed");
            return;
        }

        mappingStartedAtMs = millis();
        state = MappingState::MAPPING;

        LOG("MAP", "Native mapping active, cleaning motors disabled");

        // A successful start means House navigation is running and the robot
        // has acknowledged CleaningDisable.
        finishStart(true);

        if (stopRequested)
            sendDock("stopped", "");
    });
}

void MappingManager::sendDock(const String& result, const String& error, std::function<void(bool)> callback) {
    LOG("MAP", "Requesting native return to dock");

    pendingResult = result;
    pendingError = error;
    state = MappingState::RETURNING;
    returnStartedAtMs = millis();

    // Important: do NOT send Clean Stop here. The D6 needs the active native
    // cleaning/localization context for SEND_TO_BASE to navigate back to dock.
    serial.clean("dock", [this, callback](bool ok) {
        if (!ok) {
            LOG("MAP", "Dock command failed");

            // If SEND_TO_BASE itself cannot be issued, stop the clean so the
            // robot cannot continue roaming indefinitely.
            serial.clean("stop", nullptr);

            String error = pendingError;
            if (!error.isEmpty())
                error += ";";
            error += "dock_command_failed";

            beginRestore("failed", error);
        }

        if (callback)
            callback(ok);
    });
}

void MappingManager::beginRestore(const String& result, const String& error) {
    if (state == MappingState::RESTORING)
        return;

    pendingResult = result;
    pendingError = error;

    state = MappingState::RESTORING;
    restoreAttempts = 0;

    LOG("MAP", "Restoring normal cleaning function");

    attemptRestore();
}

void MappingManager::attemptRestore() {
    if (!cleaningDisabled) {
        finishRestore(true);
        return;
    }

    restoreAttempts++;

    serial.setCleaningEnabled(true, [this](bool ok) {
        if (ok) {
            cleaningDisabled = false;
            finishRestore(true);
            return;
        }

        if (restoreAttempts < MAPPING_RESTORE_ATTEMPTS) {
            LOG("MAP", "CleaningEnable retry %d/%d", restoreAttempts + 1, MAPPING_RESTORE_ATTEMPTS);
            attemptRestore();
            return;
        }

        finishRestore(false);
    });
}

void MappingManager::finishRestore(bool restoreOk) {
    if (startedAtMs > 0)
        lastDurationSeconds = (millis() - startedAtMs) / 1000;

    if (!restoreOk) {
        if (!pendingError.isEmpty())
            pendingError += ";";
        pendingError += "cleaning_enable_restore_failed";
    }

    lastError = pendingError;
    lastResult = lastError.isEmpty() ? pendingResult : "failed";

    LOG("MAP", "Mapping finished: result=%s error=%s", lastResult.c_str(), lastError.c_str());

    state = lastError.isEmpty() ? MappingState::IDLE : MappingState::ERROR;

    pollPending = false;
    leftDock = false;
    stopRequested = false;

    startedAtMs = 0;
    mappingStartedAtMs = 0;
    returnStartedAtMs = 0;

    pendingResult = "";
    pendingError = "";
}

void MappingManager::pollStartState() {
    if (pollPending)
        return;

    pollPending = true;

    serial.getState([this](bool ok, const RobotState& robotState) {
        pollPending = false;

        if (!ok || state != MappingState::STARTING)
            return;

        lastUiState = robotState.uiState;
        lastRobotState = robotState.robotState;

        if (robotState.uiState.indexOf("HOUSECLEANINGRUNNING") >= 0) {
            startDisablingCleaning();
        }
    });
}

void MappingManager::pollRunState() {
    if (pollPending)
        return;

    pollPending = true;

    // Fresh charger data is required here. The normal charger cache is 30s,
    // which is too stale for reliable undock/dock transition detection.
    serial.getChargerFresh([this](bool chargerOk, const ChargerData& charger) {
        if (!chargerOk) {
            pollPending = false;
            return;
        }

        const bool extPwrPresent = charger.extPwrPresent;

        if (!extPwrPresent)
            leftDock = true;

        // Always pair dock contact with a fresh robot state. A native House run
        // can return to the base for recharge-and-resume (ST_M1_Charging_Cleaning);
        // dock contact alone therefore does not mean the map run is complete.
        serial.getState([this, extPwrPresent](bool stateOk, const RobotState& robotState) {
            pollPending = false;

            if (!stateOk)
                return;

            lastUiState = robotState.uiState;
            lastRobotState = robotState.robotState;

            if (!leftDock || !extPwrPresent)
                return;

            if (state == MappingState::RETURNING) {
                // SEND_TO_BASE was explicitly requested by Mapping Mode.
                beginRestore(pendingResult, pendingError);
                return;
            }

            if (state != MappingState::MAPPING)
                return;

            const bool rechargeAndResume =
                    robotState.robotState.indexOf("ST_M1_Charging_Cleaning") >= 0 ||
                    robotState.uiState.indexOf("CLEANINGSUSPENDED") >= 0;

            if (rechargeAndResume) {
                LOG("MAP", "Native recharge-and-resume detected, keeping cleaning disabled");
                return;
            }

            const bool completedAtDock =
                    robotState.uiState.indexOf("IDLE") >= 0 ||
                    robotState.uiState.indexOf("STANDBY") >= 0 ||
                    robotState.robotState.indexOf("ST_C_Standby") >= 0 ||
                    robotState.robotState.indexOf("ST_M2_Charging_StdBy") >= 0;

            if (completedAtDock) {
                // Native House navigation completed and returned home itself.
                beginRestore("completed", "");
            }
        });
    });
}

void MappingManager::tick() {
    if (state == MappingState::STARTING) {
        if (startedAtMs > 0 && millis() - startedAtMs >= MAPPING_START_TIMEOUT_MS) {
            LOG("MAP", "House navigation start timeout");

            // Cancel a delayed/partial start before exposing the error.
            serial.clean("stop", nullptr);
            failStart("house_start_timeout");
            return;
        }

        if (pollTicker.elapsed(MAPPING_POLL_MS))
            pollStartState();

        return;
    }

    if (state == MappingState::MAPPING) {
        if (!leftDock && mappingStartedAtMs > 0 &&
            millis() - mappingStartedAtMs >= MAPPING_UNDOCK_TIMEOUT_MS) {
            LOG("MAP", "Robot did not leave dock in time");

            serial.clean("stop", nullptr);
            beginRestore("failed", "undock_timeout");
            return;
        }

        if (pollTicker.elapsed(MAPPING_POLL_MS))
            pollRunState();

        return;
    }

    if (state == MappingState::RETURNING) {
        if (returnStartedAtMs > 0 && millis() - returnStartedAtMs >= MAPPING_RETURN_TIMEOUT_MS) {
            LOG("MAP", "Return-to-dock timeout");

            serial.clean("stop", nullptr);

            String error = pendingError;
            if (!error.isEmpty())
                error += ";";
            error += "return_to_dock_timeout";

            beginRestore("failed", error);
            return;
        }

        if (pollTicker.elapsed(MAPPING_POLL_MS))
            pollRunState();
    }
}

String MappingManager::getStatusJson() const {
    unsigned long elapsedSeconds = lastDurationSeconds;

    if (startedAtMs > 0)
        elapsedSeconds = (millis() - startedAtMs) / 1000;

    return fieldsToJson({
            {"active", isActive() ? "true" : "false", FIELD_BOOL},
            {"state", stateName(state), FIELD_STRING},
            {"cleaningDisabled", cleaningDisabled ? "true" : "false", FIELD_BOOL},
            {"leftDock", leftDock ? "true" : "false", FIELD_BOOL},
            {"stopRequested", stopRequested ? "true" : "false", FIELD_BOOL},
            {"elapsedSeconds", String(elapsedSeconds), FIELD_INT},
            {"lastResult", lastResult, FIELD_STRING},
            {"error", lastError, FIELD_STRING},
            {"uiState", lastUiState, FIELD_STRING},
            {"robotState", lastRobotState, FIELD_STRING},
    });
}
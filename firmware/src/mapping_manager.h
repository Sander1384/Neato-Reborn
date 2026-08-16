#ifndef MAPPING_MANAGER_H
#define MAPPING_MANAGER_H

#include <Arduino.h>
#include <functional>
#include "loop_task.h"
#include "neato_serial.h"

enum class MappingState : uint8_t {
    IDLE,
    STARTING,
    DISABLING_CLEANING,
    MAPPING,
    RETURNING,
    RESTORING,
    ERROR,
};

class MappingManager : public LoopTask {
public:
    explicit MappingManager(NeatoSerial& serial);

    bool control(const String& action, std::function<void(bool)> callback = nullptr);
    String getStatusJson() const;
    bool isActive() const;

protected:
    void tick() override;

private:
    NeatoSerial& serial;

    MappingState state = MappingState::IDLE;
    Ticker pollTicker;

    bool pollPending = false;
    bool cleaningDisabled = false;
    bool leftDock = false;
    bool stopRequested = false;

    unsigned long startedAtMs = 0;
    unsigned long mappingStartedAtMs = 0;
    unsigned long returnStartedAtMs = 0;
    unsigned long lastDurationSeconds = 0;

    int restoreAttempts = 0;

    String lastError;
    String lastResult;
    String lastUiState;
    String lastRobotState;
    String pendingResult;
    String pendingError;

    std::function<void(bool)> startCallback;

    bool start(std::function<void(bool)> callback);
    bool requestReturn(std::function<void(bool)> callback);

    void pollStartState();
    void pollRunState();
    void startDisablingCleaning();
    void sendDock(const String& result, const String& error, std::function<void(bool)> callback = nullptr);

    void beginRestore(const String& result, const String& error);
    void attemptRestore();
    void finishRestore(bool restoreOk);

    void failStart(const String& error);
    void finishStart(bool ok);

    static const char *stateName(MappingState value);
};

#endif // MAPPING_MANAGER_H
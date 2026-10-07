#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "shot_detector.h"

struct WifiTelemetry {
    uint16_t micRaw = 0;
    uint16_t micPeakToPeak = 0;
    float micRms = 0.0F;
    bool vibrationReady = false;
    uint16_t vibrationX = 0;
    uint16_t vibrationY = 0;
    uint16_t vibrationZ = 0;
    float vibrationMagnitude = 0.0F;
    float vibrationJerk = 0.0F;
    bool voiceFlag = false;
    bool vibrationFlag = false;
    bool gpsValid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitudeM = 0.0;
    double speedKmph = 0.0;
    double hdop = 0.0;
    uint32_t satellites = 0;
    uint32_t gpsAgeMs = UINT32_MAX;
    uint32_t gpsCharacters = 0;
    uint32_t gpsFailedChecksums = 0;
    uint32_t shotCount = 0;
    uint32_t voiceDroppedSamples = 0;
    uint32_t vibrationDroppedSamples = 0;
};

class WifiTransport {
public:
    void begin();
    void setTelemetry(const WifiTelemetry& telemetry);
    void update(uint32_t nowMs);
    bool enqueue(const ShotEvent& event);

    bool enabled() const { return enabled_; }
    bool connected() const { return WiFi.status() == WL_CONNECTED; }
    uint8_t queuedEvents() const { return count_; }
    uint32_t sentEvents() const { return sentEvents_; }
    uint32_t sentStatuses() const { return sentStatuses_; }
    uint32_t failedSends() const { return failedSends_; }
    uint32_t droppedEvents() const { return droppedEvents_; }

private:
    static constexpr uint8_t QUEUE_SIZE = 16;
    bool send(const ShotEvent& event);
    bool sendStatus(uint32_t nowMs);

    WiFiUDP udp_;
    ShotEvent queue_[QUEUE_SIZE];
    uint8_t head_ = 0;
    uint8_t tail_ = 0;
    uint8_t count_ = 0;
    uint32_t lastConnectAttemptMs_ = 0;
    uint32_t lastStatusMs_ = 0;
    uint32_t lastSendErrorLogMs_ = 0;
    uint32_t sentEvents_ = 0;
    uint32_t sentStatuses_ = 0;
    uint32_t lastShotCount_ = 0;
    uint32_t failedSends_ = 0;
    uint32_t droppedEvents_ = 0;
    WifiTelemetry telemetry_;
    uint16_t windowMicPeakToPeak_ = 0;
    float windowMicRms_ = 0.0F;
    float windowVibrationMagnitude_ = 0.0F;
    float windowVibrationJerk_ = 0.0F;
    bool enabled_ = false;
    bool wasConnected_ = false;
};

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "gps_track_store.h"
#include "shot_event_store.h"
#include "shot_detector.h"

// 定时状态包所需的传感器快照和运行统计。
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
    bool trackStorageReady = false;
    uint32_t trackPendingPoints = 0;
    uint32_t trackDroppedPoints = 0;
    bool shotStorageReady = false;
    uint32_t pendingShotEvents = 0;
    uint32_t droppedShotEvents = 0;
};

class WifiTransport {
public:
    // 建立 Wi-Fi 站点连接；网络不可用时由 update 周期重试。
    void begin();
    void setRadioEnabled(bool enabled);
    void setTelemetry(const WifiTelemetry& telemetry);
    void update(uint32_t nowMs, GpsTrackStore& trackStore, ShotEventStore& shotStore);

    bool enabled() const { return enabled_; }
    bool radioEnabled() const { return radioEnabled_; }
    bool connected() const { return WiFi.status() == WL_CONNECTED; }
    uint32_t sentEvents() const { return sentEvents_; }
    uint32_t sentStatuses() const { return sentStatuses_; }
    uint32_t failedSends() const { return failedSends_; }
    uint32_t droppedEvents() const { return droppedEvents_; }

private:
    bool sendShot(const ShotUpload& shot);
    bool sendTrackBatch(const TrackBatch& batch);
    // 上报周期状态包及最近一个统计窗口的传感器峰值。
    bool sendStatus(uint32_t nowMs);
    void receiveAcks(GpsTrackStore& trackStore, ShotEventStore& shotStore);

    WiFiUDP udp_;
    uint32_t lastConnectAttemptMs_ = 0;
    uint32_t lastStatusMs_ = 0;
    uint32_t lastSendErrorLogMs_ = 0;
    uint32_t lastTrackSendMs_ = 0;
    uint32_t lastShotSendMs_ = 0;
    uint32_t sentEvents_ = 0;
    uint32_t sentStatuses_ = 0;
    uint32_t failedSends_ = 0;
    uint32_t droppedEvents_ = 0;
    WifiTelemetry telemetry_;
    uint16_t windowMicPeakToPeak_ = 0;
    float windowMicRms_ = 0.0F;
    float windowVibrationMagnitude_ = 0.0F;
    float windowVibrationJerk_ = 0.0F;
    bool enabled_ = false;
    bool wasConnected_ = false;
    bool ackPortReady_ = false;
    bool radioEnabled_ = false;
};

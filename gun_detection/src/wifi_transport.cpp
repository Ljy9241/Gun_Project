#include "wifi_transport.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "system_config.h"

namespace {
bool jsonStringField(const char* json, const char* key, char* value, size_t capacity) {
    char token[32];
    snprintf(token, sizeof(token), "\"%s\"", key);
    const char* field = strstr(json, token);
    if (field == nullptr || (field = strchr(field, ':')) == nullptr) {
        return false;
    }
    while (isspace(static_cast<unsigned char>(*++field))) {
    }
    if (*field++ != '"') {
        return false;
    }
    size_t length = 0;
    while (*field != '\0' && *field != '"') {
        if (length + 1 >= capacity) {
            return false;
        }
        value[length++] = *field++;
    }
    if (*field != '"') {
        return false;
    }
    value[length] = '\0';
    return true;
}

bool jsonUintField(const char* json, const char* key, uint32_t& value) {
    char token[32];
    snprintf(token, sizeof(token), "\"%s\"", key);
    const char* field = strstr(json, token);
    if (field == nullptr || (field = strchr(field, ':')) == nullptr) {
        return false;
    }
    char* end = nullptr;
    const unsigned long parsed = strtoul(field + 1, &end, 10);
    if (end == field + 1) {
        return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
}

}  // namespace

void WifiTransport::begin() {
    enabled_ = config::WIFI_SSID[0] != '\0' && config::UPPER_COMPUTER_HOST[0] != '\0';
    if (!enabled_) {
        Serial.println("Wi-Fi upload disabled: configure WIFI_SSID and UPPER_COMPUTER_HOST.");
        return;
    }
    setRadioEnabled(true);
    Serial.printf("Wi-Fi connecting: ssid=%s, UDP target=%s:%u\n", config::WIFI_SSID,
                  config::UPPER_COMPUTER_HOST, config::UPPER_COMPUTER_PORT);
}

void WifiTransport::setRadioEnabled(bool enabled) {
    if (!enabled_ || radioEnabled_ == enabled) return;
    radioEnabled_ = enabled;
    if (!enabled) {
        udp_.stop();
        ackPortReady_ = false;
        wasConnected_ = false;
        WiFi.disconnect(true, false);
        WiFi.mode(WIFI_OFF);
        Serial.println("Wi-Fi radio disabled for low-power mode.");
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
    ackPortReady_ = udp_.begin(config::TRACK_ACK_PORT);
    if (!ackPortReady_) Serial.printf("UDP local port bind failed: %u\n", config::TRACK_ACK_PORT);
    WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);
    lastConnectAttemptMs_ = millis();
    lastTrackSendMs_ = 0;
    lastShotSendMs_ = 0;
    Serial.println("Wi-Fi radio enabled; pending data will upload after reconnect.");
}

void WifiTransport::setTelemetry(const WifiTelemetry& telemetry) {
    telemetry_ = telemetry;
    // 在状态包间隔内累计峰值；成功发送后才清零，发送失败则保留供下次上报。
    windowMicPeakToPeak_ = max(windowMicPeakToPeak_, telemetry.micPeakToPeak);
    windowMicRms_ = max(windowMicRms_, telemetry.micRms);
    windowVibrationMagnitude_ = max(windowVibrationMagnitude_, telemetry.vibrationMagnitude);
    windowVibrationJerk_ = max(windowVibrationJerk_, telemetry.vibrationJerk);
}

void WifiTransport::update(uint32_t nowMs, GpsTrackStore& trackStore,
                           ShotEventStore& shotStore) {
    if (!enabled_ || !radioEnabled_) {
        return;
    }

    const bool isConnected = WiFi.status() == WL_CONNECTED;
    if (isConnected != wasConnected_) {
        wasConnected_ = isConnected;
        if (isConnected) {
            Serial.printf("Wi-Fi connected: local=%s, UDP target=%s:%u\n",
                          WiFi.localIP().toString().c_str(), config::UPPER_COMPUTER_HOST,
                          config::UPPER_COMPUTER_PORT);
        } else {
            Serial.println("Wi-Fi disconnected; UDP upload paused.");
        }
    }

    // 未连接时按间隔重连；连接恢复后发送持久化事件和周期状态。
    if (!isConnected) {
        if (nowMs - lastConnectAttemptMs_ >= config::WIFI_RECONNECT_INTERVAL_MS) {
            lastConnectAttemptMs_ = nowMs;
            WiFi.disconnect();
            WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);
        }
        return;
    }

    if (!ackPortReady_) {
        ackPortReady_ = udp_.begin(config::TRACK_ACK_PORT);
    }

    if (ackPortReady_) {
        receiveAcks(trackStore, shotStore);
    }

    if (nowMs - lastStatusMs_ >= config::UDP_STATUS_INTERVAL_MS) {
        lastStatusMs_ = nowMs;
        if (sendStatus(nowMs)) {
            windowMicPeakToPeak_ = 0;
            windowMicRms_ = 0.0F;
            windowVibrationMagnitude_ = 0.0F;
            windowVibrationJerk_ = 0.0F;
        }
    }

    if (ackPortReady_ && nowMs - lastTrackSendMs_ >= config::TRACK_UPLOAD_RETRY_MS) {
        TrackBatch batch;
        if (trackStore.peekBatch(batch)) {
            lastTrackSendMs_ = nowMs;
            sendTrackBatch(batch);
        }
    }

    if (ackPortReady_ && nowMs - lastShotSendMs_ >= config::TRACK_UPLOAD_RETRY_MS) {
        ShotUpload shot;
        if (shotStore.peek(shot)) {
            lastShotSendMs_ = nowMs;
            sendShot(shot);
        }
    }
}

void WifiTransport::receiveAcks(GpsTrackStore& trackStore, ShotEventStore& shotStore) {
    while (udp_.parsePacket() > 0) {
        char payload[256];
        const int length = udp_.read(reinterpret_cast<uint8_t*>(payload), sizeof(payload) - 1);
        if (length <= 0) {
            continue;
        }
        payload[length] = '\0';
        char type[32];
        if (!jsonStringField(payload, "type", type, sizeof(type))) continue;
        char sessionId[17];
        uint32_t sequence = 0;
        if (!jsonStringField(payload, "sessionId", sessionId, sizeof(sessionId)) ||
            !jsonUintField(payload, "seq", sequence)) continue;
        if (strcmp(type, "track_ack") == 0 && trackStore.acknowledge(sessionId, sequence)) {
            lastTrackSendMs_ = 0;
        } else if (strcmp(type, "shot_ack") == 0 &&
                   shotStore.acknowledge(sessionId, sequence)) {
            Serial.printf("Shot event ACK: session=%s seq=%lu pending=%lu\n", sessionId,
                          static_cast<unsigned long>(sequence),
                          static_cast<unsigned long>(shotStore.pendingEvents()));
            lastShotSendMs_ = 0;
        }
    }
}

bool WifiTransport::sendStatus(uint32_t nowMs) {
    // 状态包使用 JSON，字段结构与 data_show/gun_wifi_monitor.py 的解析保持一致。
    char payload[1024];
    const String localIp = WiFi.localIP().toString();
    const int length = snprintf(
        payload, sizeof(payload),
        "{\"type\":\"status\",\"deviceId\":\"%s\",\"uptimeMs\":%lu,"
        "\"localIp\":\"%s\",\"rssi\":%ld,"
        "\"mic\":{\"raw\":%u,\"p2p\":%u,\"rms\":%.1f,"
        "\"maxP2P5s\":%u,\"maxRms5s\":%.1f},"
        "\"vibration\":{\"ready\":%s,\"x\":%u,\"y\":%u,\"z\":%u,"
        "\"magnitude\":%.1f,\"jerk\":%.1f,\"maxMagnitude5s\":%.1f,"
        "\"maxJerk5s\":%.1f},"
        "\"flags\":{\"voice\":%s,\"vibration\":%s},"
        "\"shotCount\":%lu,\"sentShots\":%lu,"
        "\"udp\":{\"queued\":0,\"failed\":%lu,\"dropped\":%lu},"
        "\"gps\":{\"valid\":%s,\"latitude\":%.7f,\"longitude\":%.7f,"
        "\"altitudeM\":%.1f,\"speedKmph\":%.1f,\"hdop\":%.1f,"
        "\"satellites\":%lu,\"ageMs\":%lu,\"chars\":%lu,"
        "\"failedChecksums\":%lu},"
        "\"sampleDrops\":{\"voice\":%lu,\"vibration\":%lu},"
        "\"track\":{\"storageReady\":%s,\"pendingPoints\":%lu,\"droppedPoints\":%lu},"
        "\"shotEvents\":{\"storageReady\":%s,\"pending\":%lu,\"dropped\":%lu}}\r\n",
        config::DEVICE_ID, static_cast<unsigned long>(nowMs), localIp.c_str(),
        static_cast<long>(WiFi.RSSI()), telemetry_.micRaw, telemetry_.micPeakToPeak,
        telemetry_.micRms, windowMicPeakToPeak_, windowMicRms_,
        telemetry_.vibrationReady ? "true" : "false", telemetry_.vibrationX,
        telemetry_.vibrationY, telemetry_.vibrationZ, telemetry_.vibrationMagnitude,
        telemetry_.vibrationJerk, windowVibrationMagnitude_, windowVibrationJerk_,
        telemetry_.voiceFlag ? "true" : "false",
        telemetry_.vibrationFlag ? "true" : "false",
        static_cast<unsigned long>(telemetry_.shotCount),
        static_cast<unsigned long>(sentEvents_),
        static_cast<unsigned long>(failedSends_),
        static_cast<unsigned long>(droppedEvents_),
        telemetry_.gpsValid ? "true" : "false",
        telemetry_.latitude, telemetry_.longitude, telemetry_.altitudeM,
        telemetry_.speedKmph, telemetry_.hdop,
        static_cast<unsigned long>(telemetry_.satellites),
        static_cast<unsigned long>(telemetry_.gpsAgeMs),
        static_cast<unsigned long>(telemetry_.gpsCharacters),
        static_cast<unsigned long>(telemetry_.gpsFailedChecksums),
        static_cast<unsigned long>(telemetry_.voiceDroppedSamples),
        static_cast<unsigned long>(telemetry_.vibrationDroppedSamples),
        telemetry_.trackStorageReady ? "true" : "false",
        static_cast<unsigned long>(telemetry_.trackPendingPoints),
        static_cast<unsigned long>(telemetry_.trackDroppedPoints),
        telemetry_.shotStorageReady ? "true" : "false",
        static_cast<unsigned long>(telemetry_.pendingShotEvents),
        static_cast<unsigned long>(telemetry_.droppedShotEvents));

    if (length <= 0 || static_cast<size_t>(length) >= sizeof(payload) ||
        !udp_.beginPacket(config::UPPER_COMPUTER_HOST, config::UPPER_COMPUTER_PORT)) {
        ++failedSends_;
        Serial.printf("UDP STATUS failed -> %s:%u\n", config::UPPER_COMPUTER_HOST,
                      config::UPPER_COMPUTER_PORT);
        return false;
    }

    udp_.write(reinterpret_cast<const uint8_t*>(payload), length);
    if (udp_.endPacket() != 1) {
        ++failedSends_;
        Serial.printf("UDP STATUS failed -> %s:%u\n", config::UPPER_COMPUTER_HOST,
                      config::UPPER_COMPUTER_PORT);
        return false;
    }

    ++sentStatuses_;
    Serial.printf("UDP STATUS OK -> %s:%u bytes=%d: %s\n",
                  config::UPPER_COMPUTER_HOST, config::UPPER_COMPUTER_PORT, length, payload);
    return true;
}

bool WifiTransport::sendTrackBatch(const TrackBatch& batch) {
    if (batch.length == 0 ||
        !udp_.beginPacket(config::UPPER_COMPUTER_HOST, config::UPPER_COMPUTER_PORT)) {
        ++failedSends_;
        return false;
    }
    udp_.write(reinterpret_cast<const uint8_t*>(batch.payload), batch.length);
    if (udp_.endPacket() != 1) {
        ++failedSends_;
        return false;
    }
    Serial.printf("GPS track batch sent: session=%s seq=%lu points=%u bytes=%u\n",
                  batch.sessionId, static_cast<unsigned long>(batch.lastSequence),
                  batch.pointCount, static_cast<unsigned>(batch.length));
    return true;
}

bool WifiTransport::sendShot(const ShotUpload& shot) {
    if (!shot.length || !udp_.beginPacket(config::UPPER_COMPUTER_HOST,
                                          config::UPPER_COMPUTER_PORT)) {
        ++failedSends_;
        return false;
    }
    udp_.write(reinterpret_cast<const uint8_t*>(shot.payload), shot.length);
    if (udp_.endPacket() != 1) { ++failedSends_; return false; }
    ++sentEvents_;
    Serial.printf("Shot event sent: session=%s seq=%lu bytes=%u\n", shot.sessionId,
                  static_cast<unsigned long>(shot.sequence),
                  static_cast<unsigned>(shot.length));
    return true;
}

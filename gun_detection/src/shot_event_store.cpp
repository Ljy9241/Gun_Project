#include "shot_event_store.h"

#include <LittleFS.h>
#include <Preferences.h>
#include <esp_system.h>
#include <stdio.h>
#include <string.h>

#include "system_config.h"

namespace {
constexpr char LOG_PATH[] = "/shot_events.csv";
constexpr char TEMP_PATH[] = "/shot_events.tmp";
constexpr char BACKUP_PATH[] = "/shot_events.bak";
constexpr char PREF_NAMESPACE[] = "shot_events";
constexpr char CURSOR_KEY[] = "ack_offset";
constexpr uint32_t COMPACT_THRESHOLD_BYTES = 64 * 1024;

bool saveCursor(uint32_t offset) {
    Preferences preferences;
    if (!preferences.begin(PREF_NAMESPACE, false)) return false;
    const size_t written = preferences.putUInt(CURSOR_KEY, offset);
    preferences.end();
    return written > 0;
}

bool parseRecord(const String& line, char* sessionId, uint32_t& sequence) {
    unsigned long parsedSequence = 0;
    const int parsed = sscanf(line.c_str(), "%16[^,],%lu", sessionId, &parsedSequence);
    if (parsed != 2) return false;
    sequence = static_cast<uint32_t>(parsedSequence);
    return sequence > 0;
}
}  // namespace

bool ShotEventStore::begin(bool filesystemReady) {
    if (!filesystemReady) {
        Serial.println("Shot event storage unavailable: LittleFS is not ready.");
        return false;
    }

    Preferences preferences;
    if (!preferences.begin(PREF_NAMESPACE, false)) return false;
    cursor_ = preferences.getUInt(CURSOR_KEY, 0);
    preferences.end();

    if (LittleFS.exists(BACKUP_PATH)) {
        if (LittleFS.exists(LOG_PATH)) LittleFS.remove(BACKUP_PATH);
        else if (!LittleFS.rename(BACKUP_PATH, LOG_PATH)) return false;
    }
    if (LittleFS.exists(TEMP_PATH)) LittleFS.remove(TEMP_PATH);
    if (!LittleFS.exists(LOG_PATH)) {
        File log = LittleFS.open(LOG_PATH, FILE_WRITE);
        if (!log) return false;
        log.close();
    }
    if (!recover()) {
        Serial.println("Shot event storage recovery failed.");
        return false;
    }

    snprintf(sessionId_, sizeof(sessionId_), "%08lx%08lx",
             static_cast<unsigned long>(esp_random()),
             static_cast<unsigned long>(esp_random()));
    ready_ = true;
    Serial.printf("Shot event storage ready: pending=%lu session=%s\n",
                  static_cast<unsigned long>(pendingEvents_), sessionId_);
    return true;
}

bool ShotEventStore::record(const ShotEvent& event, uint32_t uptimeMs) {
    if (!ready_) return false;
    const uint32_t utcDate = event.gps.utcValid ? event.gps.utcDate : 0;
    const uint32_t utcTimeMs = event.gps.utcValid ? event.gps.utcTimeMs : 0;
    char line[256];
    const int length = snprintf(
        line, sizeof(line),
        "%s,%lu,%lu,%lu,%lu,%llu,%u,%.2f,%.2f,%u,%u,%u,%.2f,%.2f,%u,%u,%.7f,%.7f,%.1f,%.2f,%.2f,%lu,%lu,%lu,%lu,%u\n",
        sessionId_, static_cast<unsigned long>(nextSequence_),
        static_cast<unsigned long>(event.shotId), static_cast<unsigned long>(event.shotCount),
        static_cast<unsigned long>(uptimeMs),
        static_cast<unsigned long long>(event.timestampUs),
        static_cast<unsigned>(event.voice.peakToPeak), event.voice.rms, event.voice.riseRatio,
        event.vibration.rawX, event.vibration.rawY, event.vibration.rawZ,
        event.vibration.peakMagnitude, event.vibration.peakJerk,
        event.vibration.saturated ? 1U : 0U, event.gps.valid ? 1U : 0U,
        event.gps.valid ? event.gps.latitude : 0.0,
        event.gps.valid ? event.gps.longitude : 0.0,
        event.gps.valid ? event.gps.altitudeM : 0.0,
        event.gps.valid ? event.gps.speedKmph : 0.0,
        event.gps.valid ? event.gps.hdop : 0.0,
        static_cast<unsigned long>(event.gps.valid ? event.gps.satellites : 0),
        static_cast<unsigned long>(event.gps.valid ? event.gps.ageMs : UINT32_MAX),
        static_cast<unsigned long>(utcDate), static_cast<unsigned long>(utcTimeMs),
        static_cast<unsigned>(event.gps.utcSource));
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) {
        ++droppedEvents_;
        return false;
    }

    File log = LittleFS.open(LOG_PATH, FILE_APPEND);
    if (!log) {
        ++droppedEvents_;
        return false;
    }
    const size_t written = log.write(reinterpret_cast<const uint8_t*>(line), length);
    log.flush();
    log.close();
    if (written != static_cast<size_t>(length)) {
        ++droppedEvents_;
        if (!recover()) ready_ = false;
        return false;
    }
    ++nextSequence_;
    ++pendingEvents_;
    return true;
}

bool ShotEventStore::peek(ShotUpload& upload) {
    if (!ready_) return false;
    File log = LittleFS.open(LOG_PATH, FILE_READ);
    if (!log || cursor_ >= log.size() || !log.seek(cursor_)) {
        if (log) log.close();
        return false;
    }
    const uint32_t start = static_cast<uint32_t>(log.position());
    String line = log.readStringUntil('\n');
    const uint32_t end = static_cast<uint32_t>(log.position());
    log.close();
    if (end <= start + line.length()) return false;

    char session[17] = {};
    unsigned long sequence = 0, shotId = 0, shotCount = 0, uptimeMs = 0;
    unsigned long long timestampUs = 0;
    unsigned voiceP2P = 0, rawX = 0, rawY = 0, rawZ = 0, saturated = 0, gpsValid = 0;
    unsigned long satellites = 0, gpsAgeMs = UINT32_MAX, utcDate = 0, utcTimeMs = 0;
    unsigned long utcSource = UTC_SOURCE_UPTIME;
    float voiceRms = 0, riseRatio = 0, accelPeak = 0, accelJerk = 0;
    float hdop = 0;
    double latitude = 0, longitude = 0;
    float altitudeM = 0, speedKmph = 0;
    const int parsed = sscanf(
        line.c_str(),
        "%16[^,],%lu,%lu,%lu,%lu,%llu,%u,%f,%f,%u,%u,%u,%f,%f,%u,%u,%lf,%lf,%f,%f,%f,%lu,%lu,%lu,%lu,%lu",
        session, &sequence, &shotId, &shotCount, &uptimeMs, &timestampUs, &voiceP2P,
        &voiceRms, &riseRatio, &rawX, &rawY, &rawZ, &accelPeak, &accelJerk, &saturated,
        &gpsValid,
        &latitude, &longitude, &altitudeM, &speedKmph, &hdop, &satellites, &gpsAgeMs,
        &utcDate, &utcTimeMs, &utcSource);
    if (parsed != 26 || sequence == 0) return false;

    memcpy(upload.sessionId, session, sizeof(upload.sessionId));
    upload.sequence = static_cast<uint32_t>(sequence);
    upload.nextOffset = end;
    const int length = snprintf(
        upload.payload, sizeof(upload.payload),
        "{\"type\":\"shot\",\"deviceId\":\"%s\",\"sessionId\":\"%s\",\"seq\":%lu,"
        "\"shotId\":%lu,\"shotCount\":%lu,\"uptimeMs\":%lu,\"uptimeUs\":%llu,"
        "\"timestampSource\":\"%s\",\"utcDate\":%lu,\"utcTimeMs\":%lu,"
        "\"voiceP2P\":%u,\"voiceRms\":%.1f,\"voiceRiseRatio\":%.2f,"
        "\"accelX\":%u,\"accelY\":%u,\"accelZ\":%u,\"accelPeak\":%.1f,"
        "\"accelJerk\":%.1f,\"accelSaturated\":%s,\"gpsValid\":%s,"
        "\"latitude\":%.7f,\"longitude\":%.7f,\"altitudeM\":%.1f,"
        "\"speedKmph\":%.2f,\"hdop\":%.1f,\"satellites\":%lu,\"gpsAgeMs\":%lu}\r\n",
        config::DEVICE_ID, session, sequence, shotId, shotCount, uptimeMs, timestampUs,
        utcTimeSourceName(static_cast<uint8_t>(utcSource), utcDate != 0),
        utcDate, utcTimeMs, voiceP2P, voiceRms, riseRatio,
        rawX, rawY, rawZ, accelPeak, accelJerk, saturated ? "true" : "false",
        gpsValid ? "true" : "false",
        latitude, longitude, altitudeM, speedKmph, hdop, satellites, gpsAgeMs);
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(upload.payload)) return false;
    upload.length = static_cast<size_t>(length);
    return true;
}

bool ShotEventStore::acknowledge(const char* sessionId, uint32_t sequence) {
    ShotUpload expected;
    if (!peek(expected) || strcmp(expected.sessionId, sessionId) != 0 ||
        expected.sequence != sequence) return false;
    if (!saveCursor(expected.nextOffset)) return false;
    cursor_ = expected.nextOffset;
    if (pendingEvents_) --pendingEvents_;

    File log = LittleFS.open(LOG_PATH, FILE_READ);
    const uint32_t size = log ? static_cast<uint32_t>(log.size()) : 0;
    if (log) log.close();
    if (cursor_ >= size) return compact(cursor_);
    if (cursor_ >= COMPACT_THRESHOLD_BYTES) return compact(cursor_);
    return true;
}

bool ShotEventStore::compact(uint32_t startOffset) {
    if (LittleFS.exists(TEMP_PATH)) LittleFS.remove(TEMP_PATH);
    File source = LittleFS.open(LOG_PATH, FILE_READ);
    File temp = LittleFS.open(TEMP_PATH, FILE_WRITE);
    if (!source || !temp || !source.seek(startOffset)) {
        if (source) source.close();
        if (temp) temp.close();
        return false;
    }
    while (source.available()) {
        const int value = source.read();
        if (value < 0) break;
        if (temp.write(static_cast<uint8_t>(value)) != 1) {
            source.close(); temp.close(); return false;
        }
    }
    source.close(); temp.flush(); temp.close();
    // 先清零游标再替换日志；若断电导致旧日志被恢复，最多重传，SQLite 会去重。
    if (!saveCursor(0)) return false;
    cursor_ = 0;
    if (LittleFS.exists(BACKUP_PATH)) LittleFS.remove(BACKUP_PATH);
    if (!LittleFS.rename(LOG_PATH, BACKUP_PATH)) return false;
    if (!LittleFS.rename(TEMP_PATH, LOG_PATH)) {
        LittleFS.rename(BACKUP_PATH, LOG_PATH);
        return false;
    }
    LittleFS.remove(BACKUP_PATH);
    return true;
}

bool ShotEventStore::recover() {
    File log = LittleFS.open(LOG_PATH, FILE_READ);
    if (!log) return false;
    const uint32_t size = static_cast<uint32_t>(log.size());
    if (cursor_ > size) {
        cursor_ = 0;
        if (!saveCursor(0)) { log.close(); return false; }
    }

    uint32_t completeEnd = 0;
    pendingEvents_ = 0;
    while (log.available()) {
        const uint32_t start = static_cast<uint32_t>(log.position());
        String line = log.readStringUntil('\n');
        const uint32_t end = static_cast<uint32_t>(log.position());
        if (end <= start + line.length()) break;
        char session[17] = {};
        uint32_t sequence = 0;
        if (!parseRecord(line, session, sequence)) { log.close(); return false; }
        completeEnd = end;
        if (start >= cursor_) ++pendingEvents_;
    }
    log.close();
    if (completeEnd < size) {
        // 通过临时文件去掉断电时尚未写完整的最后一条记录。
        if (LittleFS.exists(TEMP_PATH)) LittleFS.remove(TEMP_PATH);
        File source = LittleFS.open(LOG_PATH, FILE_READ);
        File temp = LittleFS.open(TEMP_PATH, FILE_WRITE);
        if (!source || !temp) { if (source) source.close(); if (temp) temp.close(); return false; }
        for (uint32_t i = 0; i < completeEnd; ++i) {
            const int value = source.read();
            if (value < 0 || temp.write(static_cast<uint8_t>(value)) != 1) {
                source.close(); temp.close(); return false;
            }
        }
        source.close(); temp.flush(); temp.close();
        if (LittleFS.exists(BACKUP_PATH)) LittleFS.remove(BACKUP_PATH);
        if (!LittleFS.rename(LOG_PATH, BACKUP_PATH) || !LittleFS.rename(TEMP_PATH, LOG_PATH)) {
            if (!LittleFS.exists(LOG_PATH)) LittleFS.rename(BACKUP_PATH, LOG_PATH);
            return false;
        }
        LittleFS.remove(BACKUP_PATH);
        if (cursor_ > completeEnd) {
            cursor_ = completeEnd;
            if (!saveCursor(cursor_)) return false;
        }
    }
    return true;
}

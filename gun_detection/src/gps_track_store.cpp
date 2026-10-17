#include "gps_track_store.h"

#include <LittleFS.h>
#include <Preferences.h>
#include <esp_system.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "system_config.h"

namespace {
constexpr char LOG_PATH[] = "/gps_track.csv";
constexpr char TEMP_PATH[] = "/gps_track.tmp";
constexpr char BACKUP_PATH[] = "/gps_track.bak";
constexpr char PREF_NAMESPACE[] = "gps_track";
constexpr char CURSOR_KEY[] = "ack_offset";
constexpr uint32_t COMPACT_THRESHOLD_BYTES = 64 * 1024;

bool saveCursor(uint32_t offset) {
    Preferences preferences;
    if (!preferences.begin(PREF_NAMESPACE, false)) {
        return false;
    }
    const size_t written = preferences.putUInt(CURSOR_KEY, offset);
    preferences.end();
    return written > 0;
}
}  // namespace

bool GpsTrackStore::begin() {
    Preferences preferences;
    if (!preferences.begin(PREF_NAMESPACE, false)) {
        Serial.println("GPS track storage unavailable: NVS open failed.");
        return false;
    }
    const bool filesystemInitialized = preferences.getBool("fs_ready", false);
    preferences.end();

    if (!LittleFS.begin(false)) {
        // 首次使用时允许初始化空分区；后续挂载失败绝不自动格式化，以免删掉未上传轨迹。
        if (filesystemInitialized || !LittleFS.begin(true)) {
            Serial.println("GPS track storage unavailable: LittleFS mount failed.");
            return false;
        }
        if (!preferences.begin(PREF_NAMESPACE, false)) {
            return false;
        }
        preferences.putBool("fs_ready", true);
        preferences.end();
    } else if (!filesystemInitialized) {
        if (!preferences.begin(PREF_NAMESPACE, false)) {
            return false;
        }
        preferences.putBool("fs_ready", true);
        preferences.end();
    }

    // 恢复文件替换过程中断电留下的备份；临时文件尚未提交，可以安全删除。
    if (LittleFS.exists(BACKUP_PATH)) {
        if (LittleFS.exists(LOG_PATH)) {
            LittleFS.remove(BACKUP_PATH);
        } else if (!LittleFS.rename(BACKUP_PATH, LOG_PATH)) {
            Serial.println("GPS track storage recovery failed: cannot restore backup.");
            return false;
        }
    }
    if (LittleFS.exists(TEMP_PATH)) {
        LittleFS.remove(TEMP_PATH);
    }

    if (!preferences.begin(PREF_NAMESPACE, false)) {
        Serial.println("GPS track storage unavailable: NVS cursor open failed.");
        return false;
    }
    cursor_ = preferences.getUInt(CURSOR_KEY, 0);
    preferences.end();

    if (!LittleFS.exists(LOG_PATH)) {
        File newLog = LittleFS.open(LOG_PATH, FILE_WRITE);
        if (!newLog) {
            Serial.println("GPS track storage unavailable: cannot create log.");
            return false;
        }
        newLog.close();
    }
    if (!recoverLog()) {
        Serial.println("GPS track storage recovery failed.");
        return false;
    }

    snprintf(sessionId_, sizeof(sessionId_), "%08lx%08lx",
             static_cast<unsigned long>(esp_random()),
             static_cast<unsigned long>(esp_random()));
    ready_ = true;
    Serial.printf("GPS track storage ready: pending=%lu session=%s\n",
                  static_cast<unsigned long>(pendingPoints_), sessionId_);
    return true;
}

bool GpsTrackStore::recordFix(const GpsFix& fix, uint32_t uptimeMs, bool newLocation) {
    if (!ready_ || !newLocation || !fix.valid || fix.ageMs > config::GPS_MAX_FIX_AGE_MS ||
        (fix.hdop > 0.0 && fix.hdop > config::TRACK_MAX_HDOP)) {
        return false;
    }

    if (haveLastPoint_) {
        const double distance = distanceMeters(lastLatitude_, lastLongitude_,
                                               fix.latitude, fix.longitude);
        const uint32_t elapsed = uptimeMs - lastRecordMs_;
        if (distance < config::TRACK_MIN_DISTANCE_M) {
            // 静止时不因 GPS 漂移和定时心跳不断写重复点；移动时仍按时间上限采样。
            if (fix.speedKmph < config::TRACK_MIN_MOVING_SPEED_KMPH ||
                elapsed < config::TRACK_SAMPLE_INTERVAL_MS) {
                return false;
            }
        }
    }

    TrackPoint point;
    memcpy(point.sessionId, sessionId_, sizeof(point.sessionId));
    point.sequence = nextSequence_;
    point.uptimeMs = uptimeMs;
    point.utcDate = fix.utcValid ? fix.utcDate : 0;
    point.utcTimeMs = fix.utcValid ? fix.utcTimeMs : 0;
    point.utcSource = fix.utcValid ? fix.utcSource : UTC_SOURCE_UPTIME;
    point.latitude = fix.latitude;
    point.longitude = fix.longitude;
    point.hdop = static_cast<float>(fix.hdop);
    point.satellites = fix.satellites;

    char line[192];
    const int length = snprintf(line, sizeof(line), "%s,%lu,%lu,%lu,%lu,%.7f,%.7f,%.2f,%lu,%u\n",
                                point.sessionId,
                                static_cast<unsigned long>(point.sequence),
                                static_cast<unsigned long>(point.uptimeMs),
                                static_cast<unsigned long>(point.utcDate),
                                static_cast<unsigned long>(point.utcTimeMs),
                                point.latitude, point.longitude, point.hdop,
                                static_cast<unsigned long>(point.satellites),
                                static_cast<unsigned>(point.utcSource));
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) {
        ++droppedPoints_;
        return false;
    }

    File log = LittleFS.open(LOG_PATH, FILE_APPEND);
    if (!log) {
        ++droppedPoints_;
        return false;
    }
    const size_t written = log.write(reinterpret_cast<const uint8_t*>(line), length);
    log.flush();
    log.close();
    if (written != static_cast<size_t>(length)) {
        ++droppedPoints_;
        return false;
    }

    ++nextSequence_;
    ++pendingPoints_;
    lastLatitude_ = fix.latitude;
    lastLongitude_ = fix.longitude;
    lastRecordMs_ = uptimeMs;
    haveLastPoint_ = true;
    return true;
}

bool GpsTrackStore::peekBatch(TrackBatch& batch) {
    if (!ready_) {
        return false;
    }

    File log = LittleFS.open(LOG_PATH, FILE_READ);
    if (!log || cursor_ >= log.size() || !log.seek(cursor_)) {
        return false;
    }

    memset(&batch, 0, sizeof(batch));
    size_t used = 0;
    int written = snprintf(batch.payload, sizeof(batch.payload),
                           "{\"type\":\"track_chunk\",\"deviceId\":\"%s\",\"sessionId\":\"",
                           config::DEVICE_ID);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(batch.payload)) {
        log.close();
        return false;
    }
    used = static_cast<size_t>(written);

    uint32_t offset = cursor_;
    while (batch.pointCount < config::TRACK_BATCH_POINTS && log.available()) {
        const uint32_t lineStart = static_cast<uint32_t>(log.position());
        String line = log.readStringUntil('\n');
        const uint32_t lineEnd = static_cast<uint32_t>(log.position());
        TrackPoint point;
        if (!parseLine(line, point)) {
            log.close();
            return false;
        }
        if (batch.pointCount > 0 && strcmp(point.sessionId, batch.sessionId) != 0) {
            log.seek(lineStart);
            break;
        }
        if (batch.pointCount == 0) {
            memcpy(batch.sessionId, point.sessionId, sizeof(batch.sessionId));
            written = snprintf(batch.payload + used, sizeof(batch.payload) - used,
                               "%s\",\"points\":[", batch.sessionId);
            if (written <= 0 || static_cast<size_t>(written) >= sizeof(batch.payload) - used) {
                log.close();
                return false;
            }
            used += static_cast<size_t>(written);
        }

        written = snprintf(
            batch.payload + used, sizeof(batch.payload) - used,
            "%s{\"seq\":%lu,\"uptimeMs\":%lu,\"utcDate\":%lu,\"utcTimeMs\":%lu,"
            "\"timestampSource\":\"%s\","
            "\"lat\":%.7f,\"lon\":%.7f,\"hdop\":%.2f,\"satellites\":%lu}",
            batch.pointCount == 0 ? "" : ",",
            static_cast<unsigned long>(point.sequence),
            static_cast<unsigned long>(point.uptimeMs),
            static_cast<unsigned long>(point.utcDate),
            static_cast<unsigned long>(point.utcTimeMs),
            utcTimeSourceName(point.utcSource, point.utcDate != 0),
            point.latitude, point.longitude,
            point.hdop, static_cast<unsigned long>(point.satellites));
        if (written <= 0 || static_cast<size_t>(written) >= sizeof(batch.payload) - used) {
            log.close();
            return false;
        }
        used += static_cast<size_t>(written);
        ++batch.pointCount;
        batch.lastSequence = point.sequence;
        offset = lineEnd;
    }
    log.close();

    if (batch.pointCount == 0) {
        return false;
    }
    written = snprintf(batch.payload + used, sizeof(batch.payload) - used, "]}");
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(batch.payload) - used) {
        return false;
    }
    batch.length = used + static_cast<size_t>(written);
    batch.nextOffset = offset;
    return true;
}

bool GpsTrackStore::acknowledge(const char* sessionId, uint32_t lastSequence) {
    TrackBatch expected;
    if (!peekBatch(expected) || strcmp(expected.sessionId, sessionId) != 0 ||
        expected.lastSequence != lastSequence) {
        return false;
    }

    if (!saveCursor(expected.nextOffset)) {
        Serial.println("GPS track ACK ignored: failed to persist upload cursor.");
        return false;
    }
    cursor_ = expected.nextOffset;
    pendingPoints_ = pendingPoints_ > expected.pointCount
                         ? pendingPoints_ - expected.pointCount
                         : 0;

    File log = LittleFS.open(LOG_PATH, FILE_READ);
    const uint32_t size = log ? static_cast<uint32_t>(log.size()) : 0;
    if (log) {
        log.close();
    }
    if (cursor_ >= size) {
        compactLog(cursor_, 0);
    } else if (cursor_ >= COMPACT_THRESHOLD_BYTES) {
        compactLog(cursor_, 0);
    }
    return true;
}

bool GpsTrackStore::parseLine(const String& line, TrackPoint& point) const {
    unsigned long sequence = 0;
    unsigned long uptime = 0;
    unsigned long utcDate = 0;
    unsigned long utcTimeMs = 0;
    unsigned long satellites = 0;
    unsigned source = UTC_SOURCE_UPTIME;
    int parsed = sscanf(line.c_str(), "%16[^,],%lu,%lu,%lu,%lu,%lf,%lf,%f,%lu,%u",
                        point.sessionId, &sequence, &uptime, &utcDate, &utcTimeMs,
                        &point.latitude, &point.longitude, &point.hdop, &satellites, &source);
    if (parsed != 9 && parsed != 10) {
        return false;
    }
    point.sequence = static_cast<uint32_t>(sequence);
    point.uptimeMs = static_cast<uint32_t>(uptime);
    point.utcDate = static_cast<uint32_t>(utcDate);
    point.utcTimeMs = static_cast<uint32_t>(utcTimeMs);
    point.utcSource = parsed == 10 ? static_cast<uint8_t>(source)
                                   : (point.utcDate ? UTC_SOURCE_GPS : UTC_SOURCE_UPTIME);
    point.satellites = static_cast<uint32_t>(satellites);
    return true;
}

bool GpsTrackStore::compactLog(uint32_t startOffset, uint32_t newCursor) {
    File source = LittleFS.open(LOG_PATH, FILE_READ);
    File temp = LittleFS.open(TEMP_PATH, FILE_WRITE);
    if (!source || !temp || !source.seek(startOffset)) {
        if (source) source.close();
        if (temp) temp.close();
        return false;
    }
    while (source.available()) {
        const uint32_t lineStart = static_cast<uint32_t>(source.position());
        String line = source.readStringUntil('\n');
        const uint32_t lineEnd = static_cast<uint32_t>(source.position());
        if (lineEnd <= lineStart + line.length()) {
            break;  // 不复制尚未写完的尾行。
        }
        if (lineEnd > source.size()) {
            source.close();
            temp.close();
            return false;
        }
        line += '\n';
        if (temp.write(reinterpret_cast<const uint8_t*>(line.c_str()), line.length()) !=
            line.length()) {
            source.close();
            temp.close();
            return false;
        }
    }
    source.close();
    temp.flush();
    temp.close();

    // 先清零游标再替换日志；中途断电最多导致已确认数据重传，由接收端去重。
    if (!saveCursor(newCursor)) {
        LittleFS.remove(TEMP_PATH);
        return false;
    }
    cursor_ = newCursor;
    if (LittleFS.exists(BACKUP_PATH)) {
        LittleFS.remove(BACKUP_PATH);
    }
    if (!LittleFS.rename(LOG_PATH, BACKUP_PATH)) {
        return false;
    }
    if (!LittleFS.rename(TEMP_PATH, LOG_PATH)) {
        LittleFS.rename(BACKUP_PATH, LOG_PATH);
        return false;
    }
    LittleFS.remove(BACKUP_PATH);
    return true;
}

bool GpsTrackStore::recoverLog() {
    File log = LittleFS.open(LOG_PATH, FILE_READ);
    if (!log) {
        return false;
    }
    const uint32_t size = static_cast<uint32_t>(log.size());
    if (cursor_ > size) {
        cursor_ = 0;
        if (!saveCursor(cursor_)) {
            log.close();
            return false;
        }
    }

    uint32_t lastCompleteOffset = 0;
    uint32_t validPending = 0;
    uint32_t totalComplete = 0;
    TrackPoint point;
    while (log.available()) {
        const uint32_t lineStart = static_cast<uint32_t>(log.position());
        String line = log.readStringUntil('\n');
        const uint32_t lineEnd = static_cast<uint32_t>(log.position());
        if (lineEnd <= lineStart + line.length()) {
            break;  // 最后一行没有换行符，视为掉电时写入的残行。
        }
        if (!parseLine(line, point)) {
            log.close();
            return false;
        }
        lastCompleteOffset = lineEnd;
        ++totalComplete;
        if (lineStart >= cursor_) {
            ++validPending;
        }
    }
    log.close();
    pendingPoints_ = validPending;

    if (lastCompleteOffset < size) {
        // 去掉不完整尾行；保留所有完整记录，游标之前的数据可能被安全重传。
        const uint32_t recoveredCursor = cursor_ <= lastCompleteOffset ? cursor_ : 0;
        if (recoveredCursor == 0) {
            pendingPoints_ = totalComplete;
        }
        return compactLog(0, recoveredCursor);
    }
    return true;
}

double GpsTrackStore::distanceMeters(double lat1, double lon1, double lat2, double lon2) {
    constexpr double earthRadiusM = 6371000.0;
    constexpr double radians = 0.017453292519943295;
    const double lat1Rad = lat1 * radians;
    const double lat2Rad = lat2 * radians;
    const double deltaLat = (lat2 - lat1) * radians;
    const double deltaLon = (lon2 - lon1) * radians;
    const double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
                     cos(lat1Rad) * cos(lat2Rad) * sin(deltaLon / 2.0) * sin(deltaLon / 2.0);
    return earthRadiusM * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

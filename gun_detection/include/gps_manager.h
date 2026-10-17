#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

// 时间戳来源写入轨迹和事件记录；0/1 延续旧版 GPS/holdover 编码。
enum UtcTimeSource : uint8_t {
    UTC_SOURCE_GPS = 0,
    UTC_SOURCE_GPS_HOLDOVER = 1,
    UTC_SOURCE_NTP = 2,
    UTC_SOURCE_NTP_HOLDOVER = 3,
    UTC_SOURCE_UPTIME = 4,
};

inline const char* utcTimeSourceName(uint8_t source, bool hasUtc) {
    if (!hasUtc) return "uptime";
    switch (source) {
        case UTC_SOURCE_GPS: return "gps";
        case UTC_SOURCE_GPS_HOLDOVER: return "gps_holdover";
        case UTC_SOURCE_NTP: return "ntp";
        case UTC_SOURCE_NTP_HOLDOVER: return "ntp_holdover";
        default: return "uptime";
    }
}

// 一次 GPS 定位快照；valid 同时考虑定位有效性和数据新鲜度。
struct GpsFix {
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitudeM = 0.0;
    double speedKmph = 0.0;
    double hdop = 0.0;
    uint32_t satellites = 0;
    uint32_t ageMs = UINT32_MAX;
    uint32_t utcDate = 0;  // YYYYMMDD，GPS UTC 日期。
    uint32_t utcTimeMs = 0;  // 当天 UTC 毫秒数。
    uint32_t utcAgeMs = UINT32_MAX;
    bool utcValid = false;
    uint8_t utcSource = UTC_SOURCE_UPTIME;
};

class GpsManager {
public:
    // serial 由调用方提供，便于选择硬件串口并复用其生命周期。
    explicit GpsManager(HardwareSerial& serial) : serial_(serial) {}

    void begin(uint8_t enablePin, uint8_t ppsPin, int8_t rxPin, int8_t txPin, uint32_t baud);
    void update();  // 持续读取串口字节并交给 TinyGPSPlus 解析。
    GpsFix currentFix();  // 将解析器中的最新字段整理成快照。
    bool consumeLocationUpdate();  // 自上次读取后是否解析到新的坐标。
    void setEnabled(bool enabled);
    uint32_t parsedCharacters() const { return gps_.charsProcessed(); }
    uint32_t failedChecksums() const { return gps_.failedChecksum(); }

private:
    HardwareSerial& serial_;
    TinyGPSPlus gps_;
    uint8_t enablePin_ = 0;
    uint8_t ppsPin_ = 0;
    bool locationUpdatePending_ = false;
    bool enabled_ = true;
};

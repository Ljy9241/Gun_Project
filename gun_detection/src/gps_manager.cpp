#include "gps_manager.h"

#include "system_config.h"

void GpsManager::begin(uint8_t enablePin, uint8_t ppsPin, int8_t rxPin, int8_t txPin,
                       uint32_t baud) {
    enablePin_ = enablePin;
    enabled_ = true;
    ppsPin_ = ppsPin;
    pinMode(enablePin_, OUTPUT);
    digitalWrite(enablePin_, HIGH);  // 当前硬件配置为高电平使能 GPS。
    pinMode(ppsPin_, INPUT);
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
}

void GpsManager::setEnabled(bool enabled) {
    if (enabled_ == enabled) {
        return;
    }
    enabled_ = enabled;
    digitalWrite(enablePin_, enabled ? HIGH : LOW);
    if (!enabled) {
        locationUpdatePending_ = false;
    }
}

void GpsManager::update() {
    if (!enabled_) {
        return;
    }
    // 尽可能清空串口接收缓冲区，避免主循环期间 NMEA 数据积压。
    while (serial_.available() > 0) {
        gps_.encode(static_cast<char>(serial_.read()));
        // TinyGPSPlus 在经纬度字段提交新值时置位；保存独立标志供轨迹记录器消费。
        if (gps_.location.isUpdated()) {
            locationUpdatePending_ = true;
        }
    }
}

GpsFix GpsManager::currentFix() {
    GpsFix fix;
    // 定位字段可能仍有旧值；只有坐标有效且未超过允许年龄时才标记为有效。
    fix.valid = gps_.location.isValid() && gps_.location.age() <= config::GPS_MAX_FIX_AGE_MS;
    if (gps_.location.isValid()) {
        fix.latitude = gps_.location.lat();
        fix.longitude = gps_.location.lng();
        fix.ageMs = gps_.location.age();
    }
    if (gps_.altitude.isValid()) {
        fix.altitudeM = gps_.altitude.meters();
    }
    if (gps_.speed.isValid()) {
        fix.speedKmph = gps_.speed.kmph();
    }
    if (gps_.hdop.isValid()) {
        fix.hdop = gps_.hdop.hdop();
    }
    if (gps_.satellites.isValid()) {
        fix.satellites = gps_.satellites.value();
    }
    if (gps_.date.isValid() && gps_.time.isValid() && gps_.date.year() >= 2020 &&
        gps_.date.month() >= 1 && gps_.date.month() <= 12 &&
        gps_.date.day() >= 1 && gps_.date.day() <= 31 &&
        gps_.time.hour() <= 23 && gps_.time.minute() <= 59 && gps_.time.second() <= 59 &&
        gps_.date.age() <= config::GPS_MAX_FIX_AGE_MS &&
        gps_.time.age() <= config::GPS_MAX_FIX_AGE_MS) {
        fix.utcAgeMs = max(gps_.date.age(), gps_.time.age());
        fix.utcDate = static_cast<uint32_t>(gps_.date.year()) * 10000UL +
                      static_cast<uint32_t>(gps_.date.month()) * 100UL + gps_.date.day();
        fix.utcTimeMs = static_cast<uint32_t>(gps_.time.hour()) * 3600000UL +
                        static_cast<uint32_t>(gps_.time.minute()) * 60000UL +
                        static_cast<uint32_t>(gps_.time.second()) * 1000UL +
                        static_cast<uint32_t>(gps_.time.centisecond()) * 10UL;
        fix.utcValid = true;
        fix.utcSource = UTC_SOURCE_GPS;
    }
    return fix;
}

bool GpsManager::consumeLocationUpdate() {
    const bool updated = locationUpdatePending_;
    locationUpdatePending_ = false;
    return updated;
}

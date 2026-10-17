#include <Arduino.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <sys/time.h>
#include <time.h>

#include "gps_manager.h"
#include "gps_track_store.h"
#include "shot_event_store.h"
#include "shot_detector.h"
#include "system_config.h"
#include "vibrate_detection.h"
#include "voice_detection.h"
#include "wifi_transport.h"

// 各功能模块由主循环协同调度，传感器模块自身不创建任务。
VoiceDetection voiceDetection;
VibrateDetection vibrateDetection;
GpsManager gps(Serial1);
ShotDetector shotDetector;
WifiTransport wifiTransport;
GpsTrackStore gpsTrackStore;
ShotEventStore shotEventStore;

uint32_t lastDiagnosticMs = 0;
bool lowPowerMode = false;
bool deviceClockSynchronized = false;
uint32_t clockBaseDate = 0;
uint32_t clockBaseTimeMs = 0;
uint32_t clockBaseUptimeMs = 0;
uint8_t clockBaseSource = UTC_SOURCE_UPTIME;
bool ntpConfigured = false;
uint32_t lastNtpAnchorMs = 0;

static bool isLeapYear(uint32_t year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

static uint32_t nextUtcDate(uint32_t date) {
    uint32_t year = date / 10000;
    uint32_t month = (date / 100) % 100;
    uint32_t day = date % 100;
    static constexpr uint8_t monthDays[] = {31, 28, 31, 30, 31, 30,
                                             31, 31, 30, 31, 30, 31};
    uint32_t days = monthDays[month - 1] + ((month == 2 && isLeapYear(year)) ? 1 : 0);
    if (++day > days) {
        day = 1;
        if (++month > 12) { month = 1; ++year; }
    }
    return year * 10000 + month * 100 + day;
}

static void applyDeviceClock(GpsFix& fix, uint32_t uptimeMs, bool wifiConnected) {
    if (fix.utcValid) {
        // GPS 时间带有解析延迟；将锚点回推到对应的设备运行时刻。
        deviceClockSynchronized = true;
        clockBaseDate = fix.utcDate;
        clockBaseTimeMs = fix.utcTimeMs;
        clockBaseUptimeMs = uptimeMs - fix.utcAgeMs;
        clockBaseSource = UTC_SOURCE_GPS;
        fix.utcSource = UTC_SOURCE_GPS;
        return;
    }
    if (!deviceClockSynchronized) {
        fix.utcSource = UTC_SOURCE_UPTIME;
        return;
    }

    // GPS/Wi-Fi 暂时不可用时，以最近一次校时锚点加单调运行时间维持 UTC 时间。
    const uint64_t elapsedMs = static_cast<uint32_t>(uptimeMs - clockBaseUptimeMs);
    const uint64_t totalMs = static_cast<uint64_t>(clockBaseTimeMs) + elapsedMs;
    uint64_t daysElapsed = totalMs / 86400000ULL;
    uint32_t date = clockBaseDate;
    while (daysElapsed > 0) {
        date = nextUtcDate(date);
        --daysElapsed;
    }
    fix.utcDate = date;
    fix.utcTimeMs = static_cast<uint32_t>(totalMs % 86400000ULL);
    fix.utcAgeMs = 0;
    fix.utcValid = true;
    fix.utcSource = clockBaseSource == UTC_SOURCE_GPS
                        ? UTC_SOURCE_GPS_HOLDOVER
                        : (wifiConnected ? UTC_SOURCE_NTP : UTC_SOURCE_NTP_HOLDOVER);
}

static bool applyNtpTime(uint32_t uptimeMs) {
    // ESP32 SNTP 更新系统 UTC 时钟；过滤启动时尚未校准的默认时间。
    const time_t epochSeconds = time(nullptr);
    if (epochSeconds < 1577836800) return false;
    const time_t seconds = epochSeconds;
    struct tm utc = {};
    if (gmtime_r(&seconds, &utc) == nullptr) return false;
    struct timeval preciseTime = {};
    gettimeofday(&preciseTime, nullptr);
    deviceClockSynchronized = true;
    clockBaseDate = static_cast<uint32_t>(utc.tm_year + 1900) * 10000UL +
                    static_cast<uint32_t>(utc.tm_mon + 1) * 100UL +
                    static_cast<uint32_t>(utc.tm_mday);
    clockBaseTimeMs = static_cast<uint32_t>(utc.tm_hour) * 3600000UL +
                      static_cast<uint32_t>(utc.tm_min) * 60000UL +
                      static_cast<uint32_t>(utc.tm_sec) * 1000UL +
                      static_cast<uint32_t>(preciseTime.tv_usec / 1000);
    clockBaseUptimeMs = uptimeMs;
    clockBaseSource = UTC_SOURCE_NTP;
    Serial.printf("Device clock synchronized from NTP: %08lu %lu ms UTC\n",
                  static_cast<unsigned long>(clockBaseDate),
                  static_cast<unsigned long>(clockBaseTimeMs));
    return true;
}

static void printShot(const ShotEvent& shot) {
    // 输出融合事件摘要，便于不连接上位机时通过串口检查检测结果。
    Serial.printf("SHOT #%lu voice[p2p=%u rms=%.1f] vibration[peak=%.1f jerk=%.1f sat=%s] ",
                  static_cast<unsigned long>(shot.shotCount), shot.voice.peakToPeak,
                  shot.voice.rms, shot.vibration.peakMagnitude, shot.vibration.peakJerk,
                  shot.vibration.saturated ? "yes" : "no");
    if (shot.gps.valid) {
        Serial.printf("gps[%.7f,%.7f age=%lums]\n", shot.gps.latitude, shot.gps.longitude,
                      static_cast<unsigned long>(shot.gps.ageMs));
    } else {
        Serial.println("gps[no fix]");
    }
}

void setup() {
    Serial.begin(config::SERIAL_BAUD);
    delay(500);
    Serial.println();
    Serial.println("Gun detection firmware starting...");

    analogReadResolution(12);
    voiceDetection.begin(config::MIC_ADC_PIN);
    vibrateDetection.begin(config::MMA_X_PIN, config::MMA_Y_PIN, config::MMA_Z_PIN,
                            config::MMA_SLEEP_PIN);
    gps.begin(config::GPS_ENABLE_PIN, config::GPS_PPS_PIN, config::GPS_UART_RX_PIN,
              config::GPS_UART_TX_PIN, config::GPS_BAUD);
    gpsTrackStore.begin();
    shotEventStore.begin(gpsTrackStore.ready());
    wifiTransport.begin();

    Serial.println("MMA7361 and GPS enabled; continuous detection started.");
}

void loop() {
    // 统一使用微秒时钟驱动传感器采样和事件融合，毫秒时钟用于联网周期任务。
    const uint64_t nowUs = esp_timer_get_time();
    const uint32_t nowMs = millis();

    vibrateDetection.update(nowUs);
    const bool movementDetected = vibrateDetection.consumeMovement();
    if (lowPowerMode && movementDetected) {
        lowPowerMode = false;
        vibrateDetection.setSampleIntervalUs(1000000UL / config::VIBRATE_SAMPLE_RATE_HZ);
        vibrateDetection.clearPendingEvent();
        shotDetector.resetCandidates();
        voiceDetection.setSamplingEnabled(true);
        gps.setEnabled(true);
        wifiTransport.setRadioEnabled(true);
        Serial.println("Movement detected; normal monitoring resumed.");
    }

    if (!lowPowerMode) {
        voiceDetection.update(nowUs);
        gps.update();
    }

    if (!lowPowerMode) {
        VoiceEvent voiceEvent;
        if (voiceDetection.consumeEvent(voiceEvent)) {
            Serial.printf("VOICE TRIGGER p2p=%u rms=%.1f rise=%.2f\n", voiceEvent.peakToPeak,
                          voiceEvent.rms, voiceEvent.riseRatio);
            shotDetector.onVoiceEvent(voiceEvent);
        }

        VibrateEvent vibrateEvent;
        if (vibrateDetection.consumeEvent(vibrateEvent)) {
            Serial.printf("VIB TRIGGER peak=%.1f jerk=%.1f sat=%s\n",
                          vibrateEvent.peakMagnitude, vibrateEvent.peakJerk,
                          vibrateEvent.saturated ? "yes" : "no");
            shotDetector.onVibrateEvent(vibrateEvent);
        }

        shotDetector.update(nowUs);
    }

    GpsFix fix = gps.currentFix();
    applyDeviceClock(fix, nowMs, wifiTransport.connected());
    if (!lowPowerMode) {
        gpsTrackStore.recordFix(fix, nowMs, gps.consumeLocationUpdate());
        ShotEvent shot;
        if (shotDetector.consumeShot(shot, fix)) {
            printShot(shot);
            if (!shotEventStore.record(shot, nowMs)) {
                Serial.printf("Shot event could not be persisted; dropped=%lu\n",
                              static_cast<unsigned long>(shotEventStore.droppedEvents()));
            }
        }

        if (vibrateDetection.stationaryForLowPower()) {
            lowPowerMode = true;
            vibrateDetection.setSampleIntervalUs(config::LOW_POWER_POLL_INTERVAL_US);
            vibrateDetection.clearPendingEvent();
            shotDetector.resetCandidates();
            voiceDetection.setSamplingEnabled(false);
            gps.setEnabled(false);
            wifiTransport.setRadioEnabled(false);
            Serial.println("Stationary for 5 seconds; entering low-power monitoring.");
        }
    }

    // 汇总最新传感器状态；WifiTransport 会在自己的周期内打包上报。
    WifiTelemetry telemetry;
    telemetry.micRaw = voiceDetection.latestRaw();
    telemetry.micPeakToPeak = voiceDetection.latestPeakToPeak();
    telemetry.micRms = voiceDetection.latestRms();
    telemetry.vibrationReady = vibrateDetection.calibrated();
    telemetry.vibrationX = vibrateDetection.rawX();
    telemetry.vibrationY = vibrateDetection.rawY();
    telemetry.vibrationZ = vibrateDetection.rawZ();
    telemetry.vibrationMagnitude = vibrateDetection.magnitude();
    telemetry.vibrationJerk = vibrateDetection.jerk();
    telemetry.voiceFlag = shotDetector.flag1();
    telemetry.vibrationFlag = shotDetector.flag2();
    telemetry.gpsValid = fix.valid;
    telemetry.latitude = fix.latitude;
    telemetry.longitude = fix.longitude;
    telemetry.altitudeM = fix.altitudeM;
    telemetry.speedKmph = fix.speedKmph;
    telemetry.hdop = fix.hdop;
    telemetry.satellites = fix.satellites;
    telemetry.gpsAgeMs = fix.ageMs;
    telemetry.gpsCharacters = gps.parsedCharacters();
    telemetry.gpsFailedChecksums = gps.failedChecksums();
    telemetry.shotCount = shotDetector.shotCount();
    telemetry.voiceDroppedSamples = voiceDetection.droppedSamples();
    telemetry.vibrationDroppedSamples = vibrateDetection.droppedSamples();
    telemetry.trackStorageReady = gpsTrackStore.ready();
    telemetry.trackPendingPoints = gpsTrackStore.pendingPoints();
    telemetry.trackDroppedPoints = gpsTrackStore.droppedPoints();
    telemetry.shotStorageReady = shotEventStore.ready();
    telemetry.pendingShotEvents = shotEventStore.pendingEvents();
    telemetry.droppedShotEvents = shotEventStore.droppedEvents();
    wifiTransport.setTelemetry(telemetry);
    wifiTransport.update(nowMs, gpsTrackStore, shotEventStore);
    if (wifiTransport.connected()) {
        if (!ntpConfigured) {
            configTime(0, 0, config::NTP_SERVER_1, config::NTP_SERVER_2,
                       config::NTP_SERVER_3);
            ntpConfigured = true;
            Serial.println("SNTP started; waiting for internet time synchronization.");
        }
        if (!gps.currentFix().utcValid &&
            (!deviceClockSynchronized || nowMs - lastNtpAnchorMs >=
                                             config::NTP_CLOCK_REFRESH_INTERVAL_MS)) {
            if (applyNtpTime(millis())) lastNtpAnchorMs = nowMs;
        }
    } else {
        // Wi-Fi 模块可能在低功耗时被关闭；恢复联网后重新启动 SNTP 客户端。
        ntpConfigured = false;
    }

    // 定期输出健康状态和丢样/发送计数，便于现场排查。
    if (nowMs - lastDiagnosticMs >= config::DIAGNOSTIC_INTERVAL_MS) {
        lastDiagnosticMs = nowMs;
        Serial.printf(
            "mic[raw=%u p2p=%u rms=%.1f] mma[%s x=%u y=%u z=%u mag=%.1f jerk=%.1f] "
            "flags[%d,%d] mode[%s] gps[%s sats=%lu chars=%lu] track[%s pending=%lu drop=%lu] "
            "shots[pending=%lu drop=%lu] wifi[%s] udp[shot=%lu status=%lu fail=%lu] dropped[%lu,%lu]\n",
            voiceDetection.latestRaw(), voiceDetection.latestPeakToPeak(),
            voiceDetection.latestRms(), vibrateDetection.calibrated() ? "ready" : "cal",
            vibrateDetection.rawX(), vibrateDetection.rawY(), vibrateDetection.rawZ(),
            vibrateDetection.magnitude(), vibrateDetection.jerk(), shotDetector.flag1(),
            shotDetector.flag2(), lowPowerMode ? "low" : "active",
            fix.valid ? "fix" : "search",
            static_cast<unsigned long>(fix.satellites),
            static_cast<unsigned long>(gps.parsedCharacters()),
            gpsTrackStore.ready() ? "ready" : "off",
            static_cast<unsigned long>(gpsTrackStore.pendingPoints()),
            static_cast<unsigned long>(gpsTrackStore.droppedPoints()),
            static_cast<unsigned long>(shotEventStore.pendingEvents()),
            static_cast<unsigned long>(shotEventStore.droppedEvents()),
            wifiTransport.connected() ? "connected" :
                (wifiTransport.radioEnabled() ? "search" : "off"),
            static_cast<unsigned long>(wifiTransport.sentEvents()),
            static_cast<unsigned long>(wifiTransport.sentStatuses()),
            static_cast<unsigned long>(wifiTransport.failedSends()),
            static_cast<unsigned long>(voiceDetection.droppedSamples()),
            static_cast<unsigned long>(vibrateDetection.droppedSamples()));
    }

    if (lowPowerMode) {
        // MMA7361 没有运动中断脚，使用定时轻睡眠并周期采样模拟输出作为唤醒哨兵。
        esp_sleep_enable_timer_wakeup(config::LOW_POWER_POLL_INTERVAL_US);
        esp_light_sleep_start();
    } else {
        yield();
    }
}

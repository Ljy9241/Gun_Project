#pragma once

#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// 振动触发事件，记录触发时三轴 ADC 值和本次冲击的峰值特征。
struct VibrateEvent {
    uint64_t timestampUs = 0;
    uint16_t rawX = 0;
    uint16_t rawY = 0;
    uint16_t rawZ = 0;
    float peakMagnitude = 0.0F;
    float peakJerk = 0.0F;
    bool saturated = false;
};

class VibrateDetection {
public:
    // 配置三轴 ADC 输入和传感器休眠控制脚。
    bool begin(uint8_t xPin, uint8_t yPin, uint8_t zPin, uint8_t sleepPin);
    bool startSamplingTask();
    void update(uint64_t nowUs);
    bool consumeEvent(VibrateEvent& event);  // 取走单个待处理事件。
    bool stationaryForLowPower() const;
    bool consumeMovement();
    void setSampleIntervalUs(uint32_t intervalUs);
    void clearPendingEvent();

    bool flag() const;
    bool calibrated() const;
    uint16_t rawX() const;
    uint16_t rawY() const;
    uint16_t rawZ() const;
    float magnitude() const;
    float jerk() const;
    uint32_t droppedSamples() const;

private:
    static void samplingTask(void* argument);
    static void samplingTimerCallback(void* argument);
    void updateLocked(uint64_t nowUs);

    uint8_t xPin_ = 0;
    uint8_t yPin_ = 0;
    uint8_t zPin_ = 0;
    uint8_t sleepPin_ = 0;
    uint32_t sampleIntervalUs_ = 0;
    uint64_t nextSampleUs_ = 0;
    uint64_t lastTriggerUs_ = 0;
    uint32_t droppedSamples_ = 0;

    uint16_t rawX_ = 0;
    uint16_t rawY_ = 0;
    uint16_t rawZ_ = 0;
    float baselineX_ = 0.0F;
    float baselineY_ = 0.0F;
    float baselineZ_ = 0.0F;
    float previousMagnitude_ = 0.0F;
    float magnitude_ = 0.0F;
    float jerk_ = 0.0F;
    float calibrationSumX_ = 0.0F;
    float calibrationSumY_ = 0.0F;
    float calibrationSumZ_ = 0.0F;
    uint16_t calibrationCount_ = 0;
    uint8_t consecutiveTriggerSamples_ = 0;
    float candidatePeakMagnitude_ = 0.0F;
    float candidatePeakJerk_ = 0.0F;
    bool candidateSaturated_ = false;
    bool calibrated_ = false;
    bool eventPending_ = false;
    VibrateEvent pendingEvent_;
    uint64_t motionWindowStartUs_ = 0;
    uint16_t motionMinX_ = 4095;
    uint16_t motionMinY_ = 4095;
    uint16_t motionMinZ_ = 4095;
    uint16_t motionMaxX_ = 0;
    uint16_t motionMaxY_ = 0;
    uint16_t motionMaxZ_ = 0;
    uint32_t stationaryDurationMs_ = 0;
    SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t samplingTaskHandle_ = nullptr;
    esp_timer_handle_t samplingTimer_ = nullptr;
    uint32_t timerIntervalUs_ = 0;
    bool samplingTaskStarted_ = false;
    uint8_t wakeWindows_ = 0;
    uint8_t weakMotionWindows_ = 0;
    bool movementPending_ = false;
};

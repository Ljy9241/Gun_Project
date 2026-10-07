#include "shot_detector.h"

#include "system_config.h"

void ShotDetector::onVoiceEvent(const VoiceEvent& event) {
    voiceEvent_ = event;
    voiceValid_ = true;
    tryFuse(event.timestampUs);
}

void ShotDetector::onVibrateEvent(const VibrateEvent& event) {
    vibrationEvent_ = event;
    vibrationValid_ = true;
    tryFuse(event.timestampUs);
}

void ShotDetector::update(uint64_t nowUs) {
    if (voiceValid_ && nowUs - voiceEvent_.timestampUs > config::FUSION_WINDOW_US) {
        voiceValid_ = false;
    }
    if (vibrationValid_ && nowUs - vibrationEvent_.timestampUs > config::FUSION_WINDOW_US) {
        vibrationValid_ = false;
    }
}

void ShotDetector::tryFuse(uint64_t nowUs) {
    if (!voiceValid_ || !vibrationValid_ || shotPending_) {
        return;
    }

    const uint64_t difference = voiceEvent_.timestampUs > vibrationEvent_.timestampUs
                                    ? voiceEvent_.timestampUs - vibrationEvent_.timestampUs
                                    : vibrationEvent_.timestampUs - voiceEvent_.timestampUs;
    const bool withinWindow = difference <= config::FUSION_WINDOW_US;
    const bool cooldownReady = lastShotUs_ == 0 || nowUs - lastShotUs_ >= config::SHOT_COOLDOWN_US;
    if (!withinWindow || !cooldownReady) {
        return;
    }

    ++shotCount_;
    pendingShot_.shotId = shotCount_;
    pendingShot_.shotCount = shotCount_;
    pendingShot_.timestampUs = max(voiceEvent_.timestampUs, vibrationEvent_.timestampUs);
    pendingShot_.voice = voiceEvent_;
    pendingShot_.vibration = vibrationEvent_;
    shotPending_ = true;
    lastShotUs_ = pendingShot_.timestampUs;
    voiceValid_ = false;
    vibrationValid_ = false;
}

bool ShotDetector::consumeShot(ShotEvent& event, const GpsFix& gpsFix) {
    if (!shotPending_) {
        return false;
    }
    pendingShot_.gps = gpsFix;
    event = pendingShot_;
    shotPending_ = false;
    return true;
}

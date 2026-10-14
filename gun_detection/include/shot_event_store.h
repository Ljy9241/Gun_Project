#pragma once

#include <Arduino.h>

#include "shot_detector.h"

struct ShotUpload {
    char sessionId[17] = {};
    uint32_t sequence = 0;
    uint32_t nextOffset = 0;
    char payload[768] = {};
    size_t length = 0;
};

// 射击事件追加写入 LittleFS，只有电脑数据库提交后返回 ACK 才推进确认游标。
class ShotEventStore {
public:
    bool begin(bool filesystemReady);
    bool record(const ShotEvent& event, uint32_t uptimeMs);
    bool peek(ShotUpload& upload);
    bool acknowledge(const char* sessionId, uint32_t sequence);

    bool ready() const { return ready_; }
    uint32_t pendingEvents() const { return pendingEvents_; }
    uint32_t droppedEvents() const { return droppedEvents_; }

private:
    bool compact(uint32_t startOffset);
    bool recover();

    char sessionId_[17] = {};
    uint32_t nextSequence_ = 1;
    uint32_t cursor_ = 0;
    uint32_t pendingEvents_ = 0;
    uint32_t droppedEvents_ = 0;
    bool ready_ = false;
};

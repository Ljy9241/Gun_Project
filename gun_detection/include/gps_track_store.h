#pragma once

#include <Arduino.h>

#include "gps_manager.h"

struct TrackPoint {
    char sessionId[17] = {};
    uint32_t sequence = 0;
    uint32_t uptimeMs = 0;
    uint32_t utcDate = 0;
    uint32_t utcTimeMs = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    float hdop = 0.0F;
    uint32_t satellites = 0;
};

struct TrackBatch {
    char sessionId[17] = {};
    uint32_t lastSequence = 0;
    uint32_t nextOffset = 0;
    uint8_t pointCount = 0;
    char payload[1400] = {};
    size_t length = 0;
};

// LittleFS 追加式轨迹日志。确认游标持久化后才认为数据已上传。
class GpsTrackStore {
public:
    bool begin();
    bool recordFix(const GpsFix& fix, uint32_t uptimeMs, bool newLocation);
    bool peekBatch(TrackBatch& batch);
    bool acknowledge(const char* sessionId, uint32_t lastSequence);

    bool ready() const { return ready_; }
    uint32_t droppedPoints() const { return droppedPoints_; }
    uint32_t pendingPoints() const { return pendingPoints_; }
    const char* sessionId() const { return sessionId_; }

private:
    bool parseLine(const String& line, TrackPoint& point) const;
    bool compactLog(uint32_t startOffset, uint32_t newCursor);
    bool recoverLog();
    static double distanceMeters(double lat1, double lon1, double lat2, double lon2);

    char sessionId_[17] = {};
    uint32_t nextSequence_ = 1;
    uint32_t cursor_ = 0;
    uint32_t pendingPoints_ = 0;
    uint32_t droppedPoints_ = 0;
    uint32_t lastRecordMs_ = 0;
    double lastLatitude_ = 0.0;
    double lastLongitude_ = 0.0;
    bool haveLastPoint_ = false;
    bool ready_ = false;
};

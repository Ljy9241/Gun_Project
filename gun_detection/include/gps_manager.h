#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GpsFix {
    bool valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitudeM = 0.0;
    double speedKmph = 0.0;
    double hdop = 0.0;
    uint32_t satellites = 0;
    uint32_t ageMs = UINT32_MAX;
};

class GpsManager {
public:
    explicit GpsManager(HardwareSerial& serial) : serial_(serial) {}

    void begin(uint8_t enablePin, uint8_t ppsPin, int8_t rxPin, int8_t txPin, uint32_t baud);
    void update();
    GpsFix currentFix();
    uint32_t parsedCharacters() const { return gps_.charsProcessed(); }
    uint32_t failedChecksums() const { return gps_.failedChecksum(); }

private:
    HardwareSerial& serial_;
    TinyGPSPlus gps_;
    uint8_t enablePin_ = 0;
    uint8_t ppsPin_ = 0;
};

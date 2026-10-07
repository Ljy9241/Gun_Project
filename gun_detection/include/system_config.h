#pragma once

#include <Arduino.h>

namespace config {

// Hardware pins
inline constexpr uint8_t MIC_ADC_PIN = 8;       // ADC1_CH7
inline constexpr uint8_t MMA_X_PIN = 4;         // ADC1_CH3
inline constexpr uint8_t MMA_Y_PIN = 5;         // ADC1_CH4
inline constexpr uint8_t MMA_Z_PIN = 6;         // ADC1_CH5
inline constexpr uint8_t MMA_SLEEP_PIN = 7;     // HIGH: active, LOW: sleep
inline constexpr uint8_t GPS_ENABLE_PIN = 9;    // Assumes HIGH enables the module
inline constexpr uint8_t GPS_PPS_PIN = 10;
inline constexpr uint8_t GPS_UART_TX_PIN = 17;  // ESP32 TX -> GPS RX
inline constexpr uint8_t GPS_UART_RX_PIN = 18;  // ESP32 RX <- GPS TX

inline constexpr uint32_t SERIAL_BAUD = 115200;
inline constexpr uint32_t GPS_BAUD = 9600;
inline constexpr uint32_t GPS_MAX_FIX_AGE_MS = 5000;

// Voice detection. These are initial values and must be calibrated with real data.
inline constexpr uint32_t VOICE_SAMPLE_RATE_HZ = 8000;
inline constexpr uint16_t VOICE_FRAME_SAMPLES = 80;  // 10 ms at 8 kHz
inline constexpr uint16_t VOICE_P2P_THRESHOLD = 600;
inline constexpr float VOICE_RMS_THRESHOLD = 100.0F;
inline constexpr float VOICE_RISE_RATIO = 1.5F;
inline constexpr uint32_t VOICE_RETRIGGER_US = 50000;

// MMA7361 detection. G-SELECT is tied HIGH on the PCB, selecting the +/-6 g range.
inline constexpr uint32_t VIBRATE_SAMPLE_RATE_HZ = 1000;
inline constexpr uint16_t VIBRATE_CALIBRATION_SAMPLES = 300;
inline constexpr float VIBRATE_PEAK_THRESHOLD_COUNTS = 300.0F;
inline constexpr float VIBRATE_JERK_THRESHOLD_COUNTS = 150.0F;
inline constexpr float VIBRATE_QUIET_THRESHOLD_COUNTS = 90.0F;
inline constexpr uint8_t VIBRATE_TRIGGER_SAMPLES = 2;
inline constexpr uint32_t VIBRATE_RETRIGGER_US = 50000;

// Sensor fusion
inline constexpr uint32_t FUSION_WINDOW_US = 100000;
inline constexpr uint32_t SHOT_COOLDOWN_US = 150000;

// Wi-Fi/upper-computer settings. Fill these before network testing.
inline constexpr char WIFI_SSID[] = "ljy";
inline constexpr char WIFI_PASSWORD[] = "zxcvbnm1";
// This must be the computer's WLAN IPv4 address, not the phone hotspot gateway.
inline constexpr char UPPER_COMPUTER_HOST[] = "10.126.104.83";
inline constexpr uint16_t UPPER_COMPUTER_PORT = 8888;
inline constexpr char DEVICE_ID[] = "gun_dection";
inline constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
inline constexpr uint32_t UDP_STATUS_INTERVAL_MS = 5000;

inline constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 500;

}  // namespace config

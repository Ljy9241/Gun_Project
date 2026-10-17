#pragma once

#include <Arduino.h>

namespace config {

// 硬件引脚定义（ESP32-S3）。
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
inline constexpr uint32_t TRACK_SAMPLE_INTERVAL_MS = 2000;
inline constexpr float TRACK_MIN_DISTANCE_M = 1.0F;  //与上一个保存的点相比，位置至少移动 1 米，就保存新点
inline constexpr float TRACK_MIN_MOVING_SPEED_KMPH = 1.0F;
inline constexpr float TRACK_MAX_HDOP = 8.0F;   //HDOP 是水平定位精度的相对指标，通常越小越好。默认上限 8 是一个较宽松的过滤条件；超过 8 时不记录。
inline constexpr uint16_t TRACK_UPLOAD_RETRY_MS = 1000;
inline constexpr uint8_t TRACK_BATCH_POINTS = 8;    
inline constexpr uint16_t TRACK_ACK_PORT = 8889;

// 声音检测参数：当前为初始值，实际使用前应结合采集数据校准。
inline constexpr uint32_t VOICE_SAMPLE_RATE_HZ = 8000;
inline constexpr uint16_t VOICE_FRAME_SAMPLES = 80;  // 10 ms at 8 kHz
inline constexpr uint16_t VOICE_P2P_THRESHOLD = 20; // 峰峰值不低于 20 ADC 计数。
inline constexpr float VOICE_RMS_THRESHOLD = 3.0F;  // RMS 不低于 3。
inline constexpr float VOICE_RISE_RATIO = 1.0F;  // RMS 相对背景的上升比例不低于 1.5；
inline constexpr uint32_t VOICE_RETRIGGER_US = 50000;  // 距离上次声音触发至少 50 ms。

// MMA7361 振动检测参数；PCB 将 G-SELECT 拉高，因此量程为 +/-6 g。
inline constexpr uint32_t VIBRATE_SAMPLE_RATE_HZ = 1000;
inline constexpr uint16_t VIBRATE_CALIBRATION_SAMPLES = 300;
inline constexpr float VIBRATE_PEAK_THRESHOLD_COUNTS = 200.0F;// 触发逻辑要求幅值至少为 200，
inline constexpr float VIBRATE_JERK_THRESHOLD_COUNTS = 120.0F;//候选冲击开始时变化量至少为 120；
inline constexpr float VIBRATE_QUIET_THRESHOLD_COUNTS = 90.0F;
// 低功耗状态检测使用 100 ms 三轴 ADC 窗口峰峰值；阈值需按实物噪声校准。
inline constexpr uint32_t MOTION_WINDOW_US = 100000;
inline constexpr uint16_t MOTION_STILL_AXIS_DELTA = 100;  //45;
inline constexpr uint16_t MOTION_WAKE_AXIS_DELTA = 150;  //75
inline constexpr uint32_t LOW_POWER_STATIONARY_MS = 5000;
inline constexpr uint32_t LOW_POWER_POLL_INTERVAL_US = 10000;
inline constexpr uint8_t VIBRATE_TRIGGER_SAMPLES = 2;  //且随后需要达到连续触发样本数（默认 2）才生成事件
inline constexpr uint32_t VIBRATE_RETRIGGER_US = 50000;

// 声音与振动事件的融合窗口及融合后的重复触发冷却时间。
inline constexpr uint32_t FUSION_WINDOW_US = 2000000;
inline constexpr uint32_t SHOT_COOLDOWN_US = 150000;

// Wi-Fi 与上位机配置；进行网络联调前请填写实际热点和电脑地址。
inline constexpr char WIFI_SSID[] = "ljy";
inline constexpr char WIFI_PASSWORD[] = "zxcvbnm1";
// 此处填写电脑无线网卡的 IPv4 地址，不要填写手机热点网关地址。
inline constexpr char UPPER_COMPUTER_HOST[] = "10.126.104.83";
inline constexpr uint16_t UPPER_COMPUTER_PORT = 8888;
// ESP32 连入互联网后通过 SNTP 校准 UTC 时间；按 UTC 配置，不在设备端做时区偏移。
inline constexpr char NTP_SERVER_1[] = "pool.ntp.org";
inline constexpr char NTP_SERVER_2[] = "time.cloudflare.com";
inline constexpr char NTP_SERVER_3[] = "time.google.com";
inline constexpr uint32_t NTP_CLOCK_REFRESH_INTERVAL_MS = 60000;
inline constexpr char DEVICE_ID[] = "gun_dection";
inline constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;      //Wi-Fi 重新连接的时间间隔
inline constexpr uint32_t UDP_STATUS_INTERVAL_MS = 50;          // UDP 状态包发送的时间间隔

// 串口诊断信息的输出周期。
inline constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 500;

}  // namespace config

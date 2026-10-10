## 硬件设备
SR1818Z2 GPS北斗3双星定位模组微型低功耗GPS BDS3模块LNA信号强

## 前置准备
1. vscode扩展安装：platformio IDE，
2. Arduino.h报错
- 命令面板搜索 PlatformIO: Rebuild IntelliSense Index， 
3. 手机2.4G频段，接收数据把防火墙关闭，打开“Windows 安全中心” → “防火墙和网络保护” → 把当前网络（专用/公用）的防火墙先关掉。
4. 下载程序
手动进入下载模式
在 PlatformIO 中点击上传，按住板子上的 BOOT 键不松手，按一下 RESET键，然后松开，再松开 BOOT 键。

## 接收字段及含义
| 字段 | 含义 | 单位或说明 |
|---|---|---|
| `rssi` | WiFi 信号强度 | dBm，通常是负数；越接近 0，信号越强。例如 `-50 dBm` 通常比 `-80 dBm` 强。 |
| `mic_raw` | 麦克风最近一次 ADC 采样的原始值 | ESP32 ADC 读数，通常在 0–4095 范围内，不是声压或分贝。mic_raw 观察麦克风 ADC 是否有变化 |
| `mic_p2p` | 一个声音采样帧内，最大值减最小值 | ADC 计数，反映这段声音的幅度范围。mic_p2p、mic_rms 表示短时间内声音波动的幅度，可用来查看声音触发是否合理，并辅助调整检测阈值。 |
| `mic_rms` | 一个声音采样帧的均方根幅度 | ADC 计数，用于衡量声音波动强弱，不是 dB SPL。 |
| `vibration_x` / `vibration_y` / `vibration_z` | MMA7361 三个轴的原始 ADC 读数 | ADC 计数；会包含重力、安装方向和静态偏置的影响。 |
| `vibration_magnitude` | 三轴读数减去静止基线后，合成得到的偏差幅值 | ADC 计数；越大通常表示振动或冲击越明显。衡量相对静止状态的总体变化 |
| `vibration_jerk` | 相邻采样点之间，`vibration_magnitude` 的变化量 | ADC 计数变化量；越大表示振动强度变化越突然。反映变化是否突然 |
| `latitude` | GPS 纬度 | 度，南纬为负。 |
| `longitude` | GPS 经度 | 度，西经为负。 |
| `altitude_m` | GPS 海拔高度 | 米。 |
| `speed_kmph` | GPS 地面速度 | 公里/小时。 |
| `satellites` | GPS 当前报告的卫星数量 | 个；数量较多通常有助于定位，但定位质量还受环境和信号条件影响。帮助判断 GPS 是否获得较好的卫星接收条件 |

麦克风的 `p2p` 和 `rms` 来自固件每 **10 ms** 处理的一帧声音采样；三轴数据则是当前的传感器 ADC 值。

- `vibration_magnitude`：三轴加速度相对静止基线的合成偏差，计算方式是  
  \(\sqrt{(x-x_0)^2+(y-y_0)^2+(z-z_0)^2}\)。  
  值越大，表示当前三轴加速度整体偏离静止状态越多。固件中单位是 ADC 计数，不是直接的 `g`。

- `vibration_jerk`：相邻两次采样的 `vibration_magnitude` 变化量绝对值。值越大，说明振动强度变化越突然。它也是 ADC 计数的变化量；当前采样率为 1 kHz，所以相邻采样约间隔 1 ms。
它们用于振动/冲击检测。X/Y/Z 是传感器原始 ADC 值；这两个字段是固件从三轴值计算出的振动指标，都不是姿态角。
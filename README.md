# CIMC - 舵机手势控制系统

基于 **STC8G1K08** 单片机 + **PCA9685** PWM 驱动芯片，控制 **8 路舵机**实现多种手势的嵌入式项目。

## 📁 项目结构

```
CIMC/
├── 51/                   # 主项目 - 串口指令控制
│   ├── 51.c              # 源代码（A-E 五手势）
│   ├── 51.uvproj         # Keil µVision5 项目文件
│   └── STARTUP.A51       # Keil C51 启动代码
│
├── 51_test/              # 测试项目 - 独立调试舵机角度
│   ├── 51.c              # 源代码（逐个舵机扫角测试）
│   ├── 51.uvproj         # Keil µVision5 项目文件
│   └── STARTUP.A51       # Keil C51 启动代码
│
├── AiCube-ISP-v6.96Y.exe # STC 单片机烧录工具
└── README.md
```

## 🖐 手势指令（51 主项目）

通过串口发送单字符指令（115200 baud）：

| 指令 | 手势 | 说明 |
|------|------|------|
| `A` | 放松 (Relax) | 全部手指放松 |
| `B` | 摊开 (Spread) | 全部手指张开 |
| `C` | 握拳 (Fist) | 全部手指握紧 |
| `D` | 伸食指 (Point) | 仅食指伸出 |
| `E` | 伸拇指 (ThumbUp) | 仅拇指翘起 |

发送指令后，MCU 回复 `X OK\r\n` 确认。

## 🔧 舵机对应关系

| 舵机通道 | 对应手指 |
|---------|---------|
| 0, 1 | 大拇指 (Thumb) |
| 2, 3 | 食指 (Index) |
| 4, 5 | 中指 (Middle) |
| 6, 7 | 小指 (Pinky) |

> 没有无名指。

## 🧪 角度测试（51_test）

独立调试版，无需串口。上电后逐个舵机从 **最小值扫到最大值**，帮助确定每个舵机的角度范围。

测试参数在 `51_test/51.c` 顶部调整：

```c
#define SERVO_MIN  200   // 最小 PWM 值
#define SERVO_MAX  400   // 最大 PWM 值
#define SERVO_CENTER 300 // 中间位置

unsigned short servo_min[8] = {200, 200, ...};  // 每个舵机的最小值
unsigned short servo_max[8] = {400, 400, ...};  // 每个舵机的最大值
```

测试流程：
1. 全部舵机回中
2. 依次测试舵机 0 → 1 → 2 → ... → 7
3. 每个舵机：最小值保持 → 逐步增大 → 最大值保持 → 逐步减小 → 回中
4. 8 个舵机测试完后重复循环

## 🛠 开发环境

- **IDE**: Keil µVision5
- **MCU**: STC8G1K08
- **PWM 驱动**: PCA9685 (I²C 地址 0x40)
- **串口**: 115200 baud, 8N1
- **烧录**: AiCube-ISP

## 📥 烧录步骤

1. 在 Keil 中打开 `51/51.uvproj`（或 `51_test/51.uvproj`）
2. **Project → Rebuild all target files** (F7) 编译
3. 打开 `AiCube-ISP-v6.96Y.exe`
4. 选择生成的 `.hex` 文件（在 `Objects/` 目录下）
5. 连接 STC8G1K08，点击下载/编程

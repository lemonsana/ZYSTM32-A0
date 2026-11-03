# ZYSTM32-A0 智能小车综合程序

湖南智宇电子 ZYSTM32-A0 智能机器人主板（STM32F103RCT6）综合程序，包含
**黑线循迹 / 红外避障 / 红外遥控** 三个实验，板载 PC2 按键循环切换模式。

> 版本：`v1.0` 厂商原始代码 → `v2.0` 电机修复+非阻塞+避障新策略 → `v2.1` 感为 8 路灰度循迹

---

## 硬件平台

| 项目 | 参数 |
|---|---|
| 主控 | STM32F103RCT6（`STM32F10X_HD`），72 MHz |
| 底盘 | 四驱（左右两组并联驱动，2 路 PWM + 2 路方向脚） |
| 开发环境 | Keil MDK 5（`USER/Template.uvprojx`） |
| 库 | STM32 标准外设库 V3.5.0 |
| 循迹传感器 | 感为科技 8 路灰度（I2C，5V 供电） |

## 目录结构

```
ZYSTM32-A0/
├── CORE/               CMSIS 内核 + 启动文件（startup_stm32f10x_hd.s）
├── STM32F10x_FWLib/    STM32 标准外设库 V3.5.0
├── SYSTEM/             板级驱动
│   ├── delay/          SysTick 延时
│   ├── sys/            位带宏、WFI、开关中断
│   ├── usart/          串口1 + printf 重定向（115200，板载 CH340G）
│   ├── motor/          电机 PWM/方向、非阻塞速度接口、兼容运动函数
│   ├── keysacn/        按键、蜂鸣器、LED
│   ├── IRSEARCH/       循迹：感为 8 路灰度（软件 I2C）+ PD，可切回两路红外探头
│   ├── IRAvoid/        红外避障（非阻塞弧线绕行）
│   └── remote/         NEC 红外遥控解码（TIM5 输入捕获）
├── USER/               主程序、时钟、工程文件
├── docs/               方案与分析文档
└── README.TXT          厂商原始说明
```

## 引脚分配（板载固定）

| 引脚 | 丝印 | 功能 |
|---|---|---|
| PA1 | A4 | 红外遥控接收（TIM5_CH2 输入捕获） |
| PA4 | IO11 | 右电机方向 |
| PA6 | IO12 | LED D4 |
| PA7 | IO3 | 右循迹探头（v2.1 起默认不用，`LINE_USE_GRAY=0` 时启用） |
| PA9 / PA10 | — | USART1 TX/RX → 板载 CH340G USB 转串口 |
| PB0 | IO4 | 左循迹探头（同上） |
| PB1 | IO5 | 红外避障探头 |
| PB6 | IO7 | LED D3 |
| PB7 | IO8 | 左电机方向 |
| PB8 | IO9 | 左电机 PWM（TIM4_CH3） |
| PB9 | IO10 | 右电机 PWM（TIM4_CH4） |
| **PB10** | **IO16** | **灰度 SCL（软件 I2C，5V 容忍）** |
| **PB11** | **IO15** | **灰度 SDA（软件 I2C，5V 容忍）** |
| PC2 | A2 | 模式切换按键 |
| PC3 | A3 | 蜂鸣器 |

**空闲可用引脚**：PA0(IO2/ADC0)、PA2(IO1/USART2_TX)、PA3(IO0/USART2_RX)、PA8(IO6)、
PA5(IO13)、PB5(IO14)、PC0(A0)、PC1(A1)、PC5(A5)

## 灰度传感器接线（v2.1 新增）

感为 8 路灰度模块（5V 供电，I2C 接口）：

| 模块引脚 | 接到主板 | 说明 |
|---|---|---|
| VCC | **5V** 排针 | 模块只标 +5V，按 5V 供 |
| GND | GND | |
| SCL | **PB10**（丝印 IO16） | 5V 容忍脚 |
| SDA | **PB11**（丝印 IO15） | 5V 容忍脚 |
| PULL 跳线 | **必须短接** | 打开模块内部上拉，否则 I2C 总线拉不高 |

通信协议（感为私有寄存器，来自模块资料 `gw_grayscale_sensor.h`）：

| 寄存器 | 作用 |
|---|---|
| `0xAA` | PING，读回 `0x66` 表示在线 |
| `0xDD` | 读 8 位数字量，bit0 = 探头 1（最右）… bit7 = 探头 8（最左） |
| `0xB0~0xB7` | 8 路模拟量（各 8 位） |
| `0xC1` | 固件版本 |

从机地址 **0x4C**。程序用开漏 GPIO 模拟 I2C（约 100 kHz），不占用硬件 I2C 外设。

## 功能与模式

| 模式 | 切换 | 功能 | 指示灯 |
|---|---|---|---|
| 1 循迹 | 上电默认 | 8 路灰度加权重心 + PD 差速（v2.0 之前是两路红外探头） | D4 亮，D3 灭 |
| 2 避障 | 按 1 次 | 检测到障碍立即差速弧线绕行（不停车、不后退） | D4 灭，D3 作障碍指示 |
| 3 遥控 | 按 2 次 | 红外遥控器控制前进/后退/转向 | D4、D3 全亮 |

## 编译 / 烧录

1. Keil MDK 打开 `USER/Template.uvprojx`
2. 编译（工程已配置 `STM32F10X_HD, USE_STDPERIPH_DRIVER`）
3. ST-Link / J-Link 下载到 ZYSTM32-A0 主板

## 串口调试（115200，插 USB 即可）

```
[BOOT] ZYSTM32-A0 v2.1  build ...
[BOOT] gray ping = 0 (online)
mode=1 L=  55 R=  55 | gray=18 err=  50 ping=0 avoid=1
```

- `gray`：8 位数字量（16 进制），`err`：线位置误差（0 居中、负数偏左、正数偏右、999 丢线）
- `ping=0` 在线 / `ping=1` 通信失败（查 SCL/SDA/PULL/供电）
- `USER/main.c` 里 `#define DEBUG_UART 0` 可关闭打印

## 调参位置

| 内容 | 位置 |
|---|---|
| 循迹基础速度 / KP / KD / 丢线停车时间 | `SYSTEM/IRSEARCH/IRSEARCH.h` 的 `LINE_*` 宏 |
| 灰度数字量极性（黑白反了） | `SYSTEM/IRSEARCH/IRSEARCH.c` 的 `GRAY_BIT_BLACK_IS_1` |
| 避障速度 / 保持时间 / 转向 | `SYSTEM/IRAvoid/IRAvoid.c` 的 `AVOID_*` 宏 |
| 电机方向反了 | `SYSTEM/motor/motor.h` 的 `MOTOR_DIR_INVERT_L/R` |
| PWM 通道确实和引脚交叉的板子 | `SYSTEM/motor/motor.h` 的 `MOTOR_PWM_CH_SWAP` |

## 版本记录

| 版本 | 说明 |
|---|---|
| **v1.0** | 厂商原始综合例程，代码未做任何修改 |
| **v2.0** | ① 修正电机 PWM 通道交叉（左 CH3/PB8、右 CH4/PB9）与方向脚残留；② 电机层非阻塞化，主循环 10ms 节拍；③ 避障改为"不停车、不后退"弧线绕行；④ 打开 USART1 调试串口；⑤ 修 `ZYSTM32_back()` 的 `100-speed` 与 `IRSEARCH` 的 `&` 笔误 |
| **v2.1** | ① 循迹升级为感为 8 路灰度（软件 I2C + PD 控制 + 丢线找回）；② 保留两路红外探头方案，`LINE_USE_GRAY` 一键切换 |

## 已知问题 / 后续

- 灰度模块 **5V 供电**，SCL/SDA 接在 5V 容忍脚（PB10/PB11）上；若模块 `PULL` 跳线未短接，
  I2C 无法通信（串口会一直 `ping=1`）。
- `docs/v2-改造方案.md` 里有完整的根因分析、引脚核对和接线表。

# ZYSTM32-A0 智能小车综合程序

湖南智宇电子 ZYSTM32-A0 智能机器人主板（STM32F103RCT6）综合程序，包含
**黑线循迹 / 红外避障 / 红外遥控** 三个实验，板载 PC2 按键循环切换模式。

> 版本演进：`v1.0` 厂商原始 → `v2.0` 电机修复/非阻塞/避障新策略 → `v2.1`（已废弃，I2C 方案）→ **`v2.2` 感为 8 路灰度（串行 CLK/DAT）**

---

## 硬件平台

| 项目 | 参数 |
|---|---|
| 主控 | STM32F103RCT6（`STM32F10X_HD`），72 MHz |
| 底盘 | 四驱（左右两组并联驱动，2 路 PWM + 2 路方向脚） |
| 开发环境 | Keil MDK 5（`USER/Template.uvprojx`） |
| 库 | STM32 标准外设库 V3.5.0 |
| 循迹传感器 | 感为科技 8 路灰度（**串行 CLK/DAT 接口**，5V 供电） |

## 目录结构

```
ZYSTM32-A0/
├── CORE/               CMSIS 内核 + 启动文件
├── STM32F10x_FWLib/    STM32 标准外设库 V3.5.0
├── SYSTEM/
│   ├── delay/          SysTick 延时
│   ├── sys/            位带宏、WFI、开关中断
│   ├── usart/          串口1 + printf 重定向（115200，板载 CH340G）
│   ├── motor/          电机 PWM/方向、非阻塞速度接口、兼容运动函数
│   ├── keysacn/        按键、蜂鸣器、LED
│   ├── IRSEARCH/       循迹：感为 8 路灰度（串行）+ PD，可切回两路红外探头
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
| PA7 | IO3 | 右循迹探头（v2.2 默认不用） |
| PA9 / PA10 | — | USART1 TX/RX → 板载 CH340G USB 转串口 |
| PB0 | IO4 | 左循迹探头（v2.2 默认不用） |
| PB1 | IO5 | 红外避障探头 |
| PB6 | IO7 | LED D3 |
| PB7 | IO8 | 左电机方向 |
| PB8 | IO9 | 左电机 PWM（TIM4_CH3） |
| PB9 | IO10 | 右电机 PWM（TIM4_CH4） |
| **PB10** | **IO16** | **灰度 CLK（单片机输出时钟）** |
| **PB11** | **IO15** | **灰度 DAT（模块输出数据，5V 容忍脚）** |
| PC2 | A2 | 模式切换按键 |
| PC3 | A3 | 蜂鸣器 |

## 灰度传感器接线（v2.2 串行接口）

| 模块引脚 | 接到主板 | 说明 |
|---|---|---|
| VCC | **5V** | 模块只标 +5V |
| GND | GND | |
| CLK | **PB10**（IO16） | 单片机输出时钟，推挽 |
| DAT | **PB11**（IO15） | 模块输出数据，PB11 是 5V 容忍脚 |

> **不需要** PULL 上拉跳线、不需要 I2C 地址、不需要应答。

读取时序（照抄厂家 `examples/Serial/gpio_toggle_output.c`）：

```c
for (i = 0; i < 8; i++) {
    CLK = 0;        // 时钟下降沿
    delay_us(2);
    bit[i] = DAT;   // 读数据，bit0 先出
    CLK = 1;        // 上升沿，模块切换到下一位
    delay_us(5);
}
// bit0 = 探头 1（最右）…… bit7 = 探头 8（最左）
```

## 功能与模式

| 模式 | 切换 | 功能 | 指示灯 |
|---|---|---|---|
| 1 循迹 | 上电默认 | 8 路灰度加权重心 + PD 差速 + 丢线找回 | D4 亮，D3 灭 |
| 2 避障 | 按 1 次 | 检测到障碍立即差速弧线绕行（不停车、不后退） | D4 灭，D3 作障碍指示 |
| 3 遥控 | 按 2 次 | 红外遥控器控制前进/后退/转向 | D4、D3 全亮 |

## 编译 / 烧录

1. Keil MDK 打开 `USER/Template.uvprojx`
2. 编译后 ST-Link / J-Link 下载到 ZYSTM32-A0 主板

## 串口调试（115200，插 USB 即可）

```
[BOOT] ZYSTM32-A0 v2.2  build ...
[BOOT] gray first read = 0x18
mode=1 L=  55 R=  55 | gray=18 err=  50 avoid=1
```

- `gray`：8 位数字量（16 进制），把车横着移过黑线，看它是否连续变化 → 验证接线
- `err`：0 居中、负数偏左、正数偏右、`999` 丢线
- `USER/main.c` 里 `#define DEBUG_UART 0` 可关闭打印

## 调参位置

| 内容 | 位置 |
|---|---|
| 循迹基础速度 / KP / KD / 丢线停车时间 | `SYSTEM/IRSEARCH/IRSEARCH.h` 的 `LINE_*` 宏 |
| 灰度数字量极性（车往反方向修正时改） | `SYSTEM/IRSEARCH/IRSEARCH.c` 的 `GRAY_BIT_BLACK_IS_1` |
| 避障速度 / 保持时间 / 转向 | `SYSTEM/IRAvoid/IRAvoid.c` 的 `AVOID_*` 宏 |
| 电机方向反了 | `SYSTEM/motor/motor.h` 的 `MOTOR_DIR_INVERT_L/R` |
| PWM 通道确实与引脚交叉的板子 | `SYSTEM/motor/motor.h` 的 `MOTOR_PWM_CH_SWAP` |

## 版本记录

| 版本 | 说明 |
|---|---|
| **v1.0** | 厂商原始综合例程，代码未做任何修改 |
| **v2.0** | ① 修正电机 PWM 通道交叉（左 CH3/PB8、右 CH4/PB9）与方向脚残留；② 电机层非阻塞化，主循环 10ms 节拍；③ 避障改为"不停车、不后退"弧线绕行；④ 打开 USART1 调试串口；⑤ 修 `ZYSTM32_back()` 的 `100-speed`、`IRSEARCH` 的 `&` 笔误 |
| ~~v2.1~~ | I2C 接口方案，**已废弃** |
| **v2.2** | 循迹升级为感为 8 路灰度 **串行 CLK/DAT** 接口（PB10/PB11），8 位数字量加权重心 + PD 差速 + 丢线找回；保留两路红外探头方案，`LINE_USE_GRAY` 一键切换 |

## 已知问题 / 后续

- 灰度数字量极性（bit=1 是白还是黑）各模块批次可能不同，实测方向反了改一个宏即可。
- `docs/v2-改造方案.md` 记录了电机通道问题的完整根因分析（§1）与引脚核对（§2/§3）。

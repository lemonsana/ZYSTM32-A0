# ZYSTM32-A0 智能小车综合程序

湖南智宇电子 ZYSTM32-A0 智能机器人主板（STM32F103RCT6）综合程序，包含
**黑线循迹 / 红外避障 / 红外遥控** 三个实验，板载 PC2 按键循环切换模式。

> 本仓库按版本迭代：`v1.0` = 厂商原始代码（未修改）；`v2.0` = 电机通道修复 + 非阻塞改造 + 避障新策略。

---

## 硬件平台

| 项目 | 参数 |
|---|---|
| 主控 | STM32F103RCT6（`STM32F10X_HD`），72 MHz |
| 底盘 | 四驱（左右两组并联驱动，2 路 PWM + 2 路方向脚） |
| 开发环境 | Keil MDK 5（`USER/Template.uvprojx`） |
| 库 | STM32 标准外设库 V3.5.0 |

## 目录结构

```
ZYSTM32-A0/
├── CORE/               CMSIS 内核 + 启动文件（startup_stm32f10x_hd.s）
├── STM32F10x_FWLib/    STM32 标准外设库 V3.5.0
├── SYSTEM/             板级驱动
│   ├── delay/          SysTick 延时
│   ├── sys/            位带宏、WFI、开关中断
│   ├── usart/          串口1 + printf 重定向（v2 已启用，115200）
│   ├── motor/          电机 PWM/方向、非阻塞速度接口、兼容运动函数
│   ├── keysacn/        按键、蜂鸣器、LED
│   ├── IRSEARCH/       黑线循迹（2 路光电对管）
│   ├── IRAvoid/        红外避障（1 路，非阻塞弧线绕行）
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
| PA7 | IO3 | 右循迹探头 |
| PA9 / PA10 | — | USART1 TX/RX → 板载 CH340G USB 转串口 |
| PB0 | IO4 | 左循迹探头 |
| PB1 | IO5 | 红外避障探头 |
| PB6 | IO7 | LED D3 |
| PB7 | IO8 | 左电机方向 |
| PB8 | IO9 | 左电机 PWM（TIM4_CH3） |
| PB9 | IO10 | 右电机 PWM（TIM4_CH4） |
| PC2 | A2 | 模式切换按键 |
| PC3 | A3 | 蜂鸣器 |

**空闲可用引脚**：PA0(IO2/ADC0)、PA2(IO1/USART2_TX)、PA3(IO0/USART2_RX)、PA8(IO6)、
PA5(IO13)、PB5(IO14)、PB10(IO16/USART3_TX)、PB11(IO15/USART3_RX)、PC0(A0)、PC1(A1)、PC5(A5)

## 功能与模式

| 模式 | 切换 | 功能 | 指示灯 |
|---|---|---|---|
| 1 循迹 | 上电默认 | 双光电探头跨黑线跟踪（v2：非阻塞，10ms 一拍） | D4 亮，D3 灭 |
| 2 避障 | 按 1 次 | 检测到障碍立即差速弧线绕行（v2：不停车、不后退） | D4 灭，D3 作障碍指示 |
| 3 遥控 | 按 2 次 | 红外遥控器控制前进/后退/转向 | D4、D3 全亮 |

## 编译 / 烧录

1. Keil MDK 打开 `USER/Template.uvprojx`
2. 编译（工程已配置 `STM32F10X_HD, USE_STDPERIPH_DRIVER`）
3. ST-Link / J-Link 下载到 ZYSTM32-A0 主板

## 串口调试（v2 新增）

板上 CH340G 已接 **USART1 (PA9/PA10)**，插 USB 线即可：

```
串口助手：115200, 8N1
上电应看到：[BOOT] ZYSTM32-A0 v2.0  build ...
运行中每 200ms 一行：mode=1 L=  55 R=  55  |  trackL=0 trackR=0 avoid=1
```

`USER/main.c` 里 `#define DEBUG_UART 1` 可关闭打印。

## 版本记录

| 版本 | 说明 |
|---|---|
| **v1.0** | 厂商原始综合例程，代码未做任何修改 |
| **v2.0** | ① 修正电机 PWM 通道交叉（左 CH3/PB8、右 CH4/PB9）与方向脚残留问题；② 电机层改为非阻塞 `Motor_SetSpeed()`，主循环 10ms 节拍；③ 避障改为"不停车、不后退"差速弧线绕行；④ 打开 USART1 调试串口；⑤ 修正 `ZYSTM32_back()` 的 `100-speed` 速度语义 bug、`IRSEARCH` 里的 `&` 笔误 |

## 已知问题 / 后续计划

- **v2.0 已修**：循迹模式下只有左轮单向转（根因：`SetMotorSpeed()` 把"右轮"写到 CH3/PB8、"
  左轮"写到 CH4/PB9，与 `motor.h` 的引脚定义交叉；且速度为 0 时不更新方向脚）。
  详见 [`docs/v2-改造方案.md`](docs/v2-改造方案.md) 第 1 节。
- **v2.1 计划**：接入感为 8 路灰度传感器（并行/I2C 接口，5V 供电，需 5V 容忍引脚），
  循迹改为 PD 差速控制，详见同一份方案文档第 4、5 节。
- 若实测某个轮子正反转相反，改 `SYSTEM/motor/motor.h` 里的
  `MOTOR_DIR_INVERT_L / MOTOR_DIR_INVERT_R` 即可，不用改线。

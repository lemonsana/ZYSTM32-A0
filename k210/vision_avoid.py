# -*- coding: utf-8 -*-
# =============================================================================
#  ZYSTM32-A0 视觉避障 —— K210（MaixPy）端脚本
#  ---------------------------------------------------------------------------
#  作用：摄像头找障碍物（默认找"红色色块"），算出它在画面偏左/中/偏右，
#        通过 UART 发给 STM32，STM32 负责绕行。
#
#  使用步骤：
#    1. 把本文件改名为 main.py，拷到 TF 卡根目录，插卡上电
#    2. 或者用 MaixPy IDE 直接运行
#    3. 先跑通"LCD 显示 + 串口打印"，再接 STM32
#
#  协议（6 字节）：
#      AA 55 | FLAGS | DIR | DIST | XOR
#        FLAGS bit0 = 检测到障碍
#        DIR   0=障碍偏左(往右绕) 1=居中 2=障碍偏右(往左绕)
#        DIST  cm，0=未知（单目测距可选，默认填 0）
#        XOR   前 5 字节异或
#
#  接线（K210 → STM32）：
#      K210 TX  →  STM32 PA3 (USART2_RX)
#      GND      →  GND（务必共地）
#      供电：独立 5V
# =============================================================================

import sensor
import image
import lcd
import time
from machine import UART

# ----------------------------- 需要你修改的配置 -----------------------------
UART_NUM   = 2          # K210 的 UART2：TX=P11, RX=P10（亚博/矽速板常见）
UART_BAUD  = 115200

# 颜色阈值（LAB 空间）[L_min,L_max,A_min,A_max,B_min,B_max]
# 红色示例。**必须用 MaixPy IDE 的"阈值编辑器"重新取**：
# 把障碍物放镜头前，拖滑块直到画面里只有障碍物变色、背景全黑，抄下 6 个数。
THRESHOLD_RED = [(30, 80, 20, 70, 0, 60)]

MIN_AREA   = 500        # 色块最小像素面积，小于它当噪声
DIR_LEFT   = 0.40       # 色块中心 x < 画面宽 40% → 偏左
DIR_RIGHT  = 0.60       # 色块中心 x > 画面宽 60% → 偏右
LCD_ON     = True       # 有屏幕就 True，没有就 False（省时间）

# ----------------------------- 工具函数 -----------------------------
H1 = 0xAA
H2 = 0x55


def send_frame(flags, direction, dist):
    """按协议发一帧 6 字节"""
    chk = H1 ^ H2 ^ flags ^ direction ^ dist
    uart.write(bytes([H1, H2, flags, direction, dist, chk]))


def main():
    # ---- 摄像头初始化 ----
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)      # 320x240，够快够准
    sensor.skip_frames(time=2000)          # 等自动曝光稳定

    # 摄像头如果装反了，打开下面两行
    # sensor.set_vflip(1)
    # sensor.set_hmirror(1)

    if LCD_ON:
        lcd.init()

    uart = UART(UART_NUM, UART_BAUD)
    print("vision ready, uart%d @%d" % (UART_NUM, UART_BAUD))

    frame_cnt = 0
    while True:
        img = sensor.snapshot()

        blobs = img.find_blobs(THRESHOLD_RED,
                               area_threshold=MIN_AREA,
                               merge=True)
        flags = 0
        direction = 1
        dist = 0

        if blobs:
            # 取最大的色块（最近的障碍物）
            b = max(blobs, key=lambda x: x.pixels())
            img.draw_rectangle(b.rect(), color=(0, 255, 0))
            img.draw_cross(b.cx(), b.cy(), color=(0, 255, 0))

            flags = 0x01
            ratio = b.cx() * 1.0 / img.width()
            if ratio < DIR_LEFT:
                direction = 0          # 障碍偏左 → STM32 往右绕
            elif ratio > DIR_RIGHT:
                direction = 2          # 障碍偏右 → STM32 往左绕
            else:
                direction = 1          # 居中

            # 可选：用色块高度粗估距离（需要标定，先填 0 让 STM32 用默认策略）
            # dist = int(calib_distance(b.h()))

        send_frame(flags, direction, dist)
        frame_cnt += 1

        if LCD_ON:
            img.draw_string(2, 2, "F:%d D:%d #%d" % (flags, direction, frame_cnt),
                            color=(255, 255, 255))
            lcd.display(img)


# 可选：标定距离（把障碍摆在 20/40/60cm，记录 b.h()，拟合）
# def calib_distance(h_pixel):
#     return int(10000 / h_pixel)   # 示例系数，需实测

main()

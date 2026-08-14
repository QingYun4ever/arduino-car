"""
=============================================================================
k230_color_uart_test.py
Yahboom K230 独立颜色识别 + UART结果发送测试
=============================================================================

用途
----
在 K230（CanMV MicroPython）上运行：
  1. 摄像头连续寻找画面中最大的红、蓝、绿色块；
  2. 在 CanMV IDE 预览图中框出色块、打印识别信息；
  3. 每当稳定识别到一种颜色，向 UART1 发出一个短帧：

       $COLOR_RED#       $COLOR_BLUE#       $COLOR_GREEN#       $COLOR_NONE#

本程序不控制 Arduino Nano、TB6612、电机、蜂鸣器或舵机。
在 K230 尚未接线到 Nano 前，也可直接通过 CanMV IDE 的终端和预览窗口验证。

-----------------------------------------------------------------------------
K230 通信口（Yahboom 默认 UART1）
-----------------------------------------------------------------------------
  GPIO9   = UART1_TXD  ----> 未来接 Nano 的接收脚（必须确认电平）
  GPIO10  = UART1_RXD  <---- 未来接 Nano 的发送脚（本颜色测试不读取命令）
  GND     <------------> Nano GND（未来接线时必须共地）

【严禁未经电平确认直接连接 Nano TX(5V) 到 K230 RX。】
K230 UART I/O 可能是1.8V或3.3V；等模块到手后，以实物丝印/说明书为准，
必要时增加电平转换器。本脚本只发送数据，故首次硬件联调可只接
K230 TX -> Nano RX 和 GND；仍须先确认 K230 TX 电平能被Nano安全可靠识别。

-----------------------------------------------------------------------------
上机步骤（尚未实机验证）
-----------------------------------------------------------------------------
1. K230 单独用 Type-C 稳定供电；不要从 Nano 5V 给K230供电。
2. 在 CanMV IDE K230 打开本文件，先不要连接 Nano。
3. Run 运行，观察 IDE 预览中检测框和终端输出。
4. 用纯红、纯蓝、纯绿纸/色卡逐一测试；根据实际光照使用 CanMV IDE 的
   Tools -> Machine Vision -> Threshold Editor 校准下面的 LAB 阈值。
5. 需要开机运行时，确认无误后保存为 K230 的 main.py。

官方基础例程说明：find_blobs 在 RGB565 图像上使用LAB阈值找色块。
本文件的默认阈值来自 Yahboom K230 颜色识别案例，只是起点，不保证适合赛场。
=============================================================================
"""

import time
import os
import gc

from machine import UART, FPIOA
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

# ============================ 可调整的识别参数 =============================

DETECT_WIDTH = 640
DETECT_HEIGHT = 480

# LAB: (L_min, L_max, A_min, A_max, B_min, B_max)
# 来源：Yahboom K230 官方“颜色识别”示例的通用起点，必须按实物和光照校准。
COLOR_THRESHOLDS = {
    "RED": (0, 66, 7, 127, 3, 127),
    "GREEN": (42, 100, -128, -17, 6, 66),
    "BLUE": (43, 99, -43, -4, -56, -7),
}

# 过滤过小噪点。640x480画面里，目标应先明显大于这些最小值。
PIXELS_THRESHOLD = 300
AREA_THRESHOLD = 300

# 同一颜色连续出现这么多帧，才向串口报告，防止单帧噪声误触发。
STABLE_FRAMES_REQUIRED = 4

# 串口同一结果的最短重复发送间隔；Nano端最终仍应再做连续帧过滤。
UART_REPEAT_MS = 500

# 使用 CanMV IDE 虚拟显示预览。若模块屏幕/固件有差异，请先运行厂家原始颜色例程。
SHOW_IDE_PREVIEW = True

# =============================== UART 设置 =================================

UART_BAUDRATE = 115200
K230_UART_TX_PIN = 9
K230_UART_RX_PIN = 10

# ============================== 工具函数 ===================================

def ticks_ms():
    """兼容 CanMV/MicroPython 的毫秒时钟。"""
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def init_uart():
    """把Yahboom通信接口的GPIO9/10复用为UART1并以115200-8N1初始化。"""
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, FPIOA.UART1_TXD, ie=0, oe=1)
    fpioa.set_function(K230_UART_RX_PIN, FPIOA.UART1_RXD, ie=1, oe=0)
    return UART(UART.UART1, UART_BAUDRATE), fpioa


def init_sensor():
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor


def init_display():
    if SHOW_IDE_PREVIEW:
        Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=60)
    MediaManager.init()


def largest_blob(img, threshold):
    """返回该颜色像素数最多的色块；没有合格色块则返回None。"""
    blobs = img.find_blobs(
        [threshold],
        pixels_threshold=PIXELS_THRESHOLD,
        area_threshold=AREA_THRESHOLD,
        merge=True,
    )
    if not blobs:
        return None

    largest = blobs[0]
    for blob in blobs[1:]:
        if blob.pixels() > largest.pixels():
            largest = blob
    return largest


def detect_dominant_color(img):
    """
    检查红、蓝、绿三种颜色，返回 (颜色名, 最大色块)。
    多种颜色同时存在时，按色块像素数选择最大的一个；本测试只关心主目标。
    """
    best_name = "NONE"
    best_blob = None
    best_pixels = 0

    for name, threshold in COLOR_THRESHOLDS.items():
        blob = largest_blob(img, threshold)
        if blob is not None and blob.pixels() > best_pixels:
            best_name = name
            best_blob = blob
            best_pixels = blob.pixels()

    return best_name, best_blob


def draw_result(img, name, blob):
    if name == "RED":
        color = (255, 0, 0)
    elif name == "GREEN":
        color = (0, 255, 0)
    elif name == "BLUE":
        color = (0, 100, 255)
    else:
        color = (255, 255, 255)

    if blob is not None:
        img.draw_rectangle(blob.rect(), color=color, thickness=3)
        img.draw_cross(blob.cx(), blob.cy(), color=color, thickness=2)
        label = "%s px=%d" % (name, blob.pixels())
    else:
        label = "NONE"

    img.draw_string_advanced(0, 0, 24, label, color=color)


def send_frame(uart, result):
    """发送项目约定帧；ASCII避免中文编码和Nano RAM处理问题。"""
    frame = "$COLOR_%s#\n" % result
    uart.write(frame)
    print("UART ->", frame)


def main():
    sensor = None
    uart = None

    # 稳定性状态：只有连续得到同一结果，才认为它有效。
    candidate = "NONE"
    candidate_frames = 0
    reported = None
    last_send_ms = 0

    try:
        uart, _fpioa = init_uart()
        sensor = init_sensor()
        init_display()
        sensor.run()

        print("K230 color/UART test started.")
        print("UART1: GPIO9 TX, GPIO10 RX, %d baud." % UART_BAUDRATE)
        print("Show a red, blue, or green target. Ctrl+C in IDE stops the test.")

        while True:
            # 允许CanMV IDE在请求时正常结束运行。
            os.exitpoint()

            img = sensor.snapshot()
            detected, blob = detect_dominant_color(img)
            draw_result(img, detected, blob)

            if detected == candidate:
                candidate_frames += 1
            else:
                candidate = detected
                candidate_frames = 1

            now = ticks_ms()
            is_stable = candidate_frames >= STABLE_FRAMES_REQUIRED
            should_report = is_stable and (
                candidate != reported
                or ticks_diff(now, last_send_ms) >= UART_REPEAT_MS
            )

            if should_report:
                send_frame(uart, candidate)
                reported = candidate
                last_send_ms = now

            if SHOW_IDE_PREVIEW:
                Display.show_image(img)

            gc.collect()

    except KeyboardInterrupt:
        print("User stopped K230 color/UART test.")
    except Exception as exc:
        print("K230 color/UART test error:", exc)
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    main()

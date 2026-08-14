"""
=============================================================================
k230_visual_line_follow_controller.py
K230 视觉循迹控制器（黑线/白底，向主控输出左右轮命令）
=============================================================================

这是“真正计算循迹控制量”的独立 K230 程序：
  摄像头灰度图 -> 下方ROI黑线 -> get_regression() -> 偏移/角度 -> PD控制 -> 左右轮命令

K230本身没有接到TB6612，本程序不能直接转动本车电机。它通过UART1输出：

    $MOTOR,<left>,<right>,<offset>,<angle>,<magnitude>#
    $MOTOR,0,0,LOST,0,0#          （失线安全停车命令）

left/right范围是 -100~100：正数=前进，负数=倒退，0=停车。
未来必须另建一个专门的 Nano 串口接收草图，且完成电平转换与台架验证后，
才能由Nano把这两个数真正送到TB6612。绝不能直接连接K230去驱动TB6612。

当前项目已验证的四路光电循迹仍然是正式路线的唯一电机控制方案；本脚本只是
独立视觉循迹试验，未修改任何现有Nano .ino。

=============================================================================
安全使用顺序（不可跳过）
=============================================================================
阶段A：先运行 k230_visual_line_test.py，只观察红线、OFFSET、ANGLE、LOST。
阶段B：本程序把 COMMAND_OUTPUT_ENABLED 保持False，观察IDE控制字和终端命令，
        确认直线时L/R相近、线左/右时修正方向符合预期。
阶段C：仅把K230 UART接到电脑/USB-TTL，打开COMMAND_OUTPUT_ENABLED，观察帧。
阶段D：另建Nano“只接收并打印/蜂鸣”草图，不接电机，验证帧和符号。
阶段E：轮子悬空、低速、带独立急停，才考虑Nano把命令转给TB6612。

【电平保护】K230 UART可能是1.8V或3.3V；Nano TX是5V。未确认模块实物电平
和加电平转换器前，不能直接连接Nano TX -> K230 RX。首轮可仅验证K230 TX
到安全的USB-TTL接收器；若接Nano，必须共地且先确认电平。
=============================================================================
"""

import gc
import math
import os
import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

# ========================= 实物校准/安全参数区 ==============================

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# 只从前方地面的下半区域看线；安装相机后必须实测修改。
ROI_Y = 132
ROI_HEIGHT = 92

# 相机安装方向。改动后必须重新验证左右修正方向。
CAMERA_HMIRROR = False
CAMERA_VFLIP = False

# 黑色候选像素灰度范围：[0, BLACK_GRAY_MAX]；按比赛灯光校准。
BLACK_GRAY_MAX = 90
MIN_MAGNITUDE = 8
X_STRIDE = 2
Y_STRIDE = 1

# 连续多少帧获得可信线，才允许输出行驶命令；避免单帧假线起步。
LINE_STABLE_FRAMES = 3

# ------------ PD循迹参数：必须轮子悬空、低速开始逐项调整 ----------------
# 直线基本速度；首次实车建议20~30，绝不先改到100。
BASE_SPEED = 25
MAX_SPEED = 45

# 合成误差 = OFFSET_GAIN * offset + ANGLE_GAIN * angle
# offset单位：像素，angle单位：度。这里是初始猜测，不能视为已验证参数。
OFFSET_GAIN = 0.65
ANGLE_GAIN = 0.45

# PD输出：P负责当前偏差，D抑制突然摆动。首次保持I=0，避免积分累积失控。
KP = 0.80
KD = 0.12

# 若实测“线在左边，车却往右修正”，把这个值改成 -1。
# 绝不在轮子接地快速行驶时盲改；先观察IDE和悬空轮。
STEERING_SIGN = 1

# 以下默认关闭，保证首次运行只在IDE显示计算结果、终端打印，不向UART发控制帧。
COMMAND_OUTPUT_ENABLED = False
UART_BAUDRATE = 115200
UART_REPEAT_MS = 50          # 最大20Hz，避免下位机命令队列堆积。
K230_UART_TX_PIN = 9
K230_UART_RX_PIN = 10

# ============================================================================

WHITE = (255, 255, 255)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)


def clamp(value, low, high):
    return max(low, min(high, value))


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def init_sensor():
    sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.set_vflip(CAMERA_VFLIP)
    return sensor


def init_display():
    Display.init(Display.VIRT, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=60)
    MediaManager.init()


def init_uart_if_enabled():
    if not COMMAND_OUTPUT_ENABLED:
        return None, None

    from machine import UART, FPIOA
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_PIN, FPIOA.UART1_TXD, ie=0, oe=1)
    fpioa.set_function(K230_UART_RX_PIN, FPIOA.UART1_RXD, ie=1, oe=0)
    uart = UART(UART.UART1, UART_BAUDRATE)
    return uart, fpioa


def line_x_at_y(line, y):
    x1, y1 = line.x1(), line.y1()
    x2, y2 = line.x2(), line.y2()
    if y2 == y1:
        return None
    return x1 + (y - y1) * (x2 - x1) / (y2 - y1)


def angle_from_forward(line):
    """图像向上代表车头正前方：0=直，负=左，正=右。"""
    dx = line.x2() - line.x1()
    dy = line.y2() - line.y1()
    if dy > 0:                 # 固定线向量朝图像上方，消除端点顺序的180度翻转。
        dx, dy = -dx, -dy
    return math.degrees(math.atan2(dx, -dy))


def detect_measurement(img, roi):
    """在下方ROI内快速拟合黑线，返回(offset, angle, magnitude)或None。"""
    line = img.get_regression(
        [(0, BLACK_GRAY_MAX)],
        roi=roi,
        x_stride=X_STRIDE,
        y_stride=Y_STRIDE,
        robust=False,
    )
    if line is None or line.magnitude() < MIN_MAGNITUDE:
        return line, None

    x = line_x_at_y(line, roi[1] + roi[3] - 1)
    if x is None:
        return line, None

    offset = int(round(x - (roi[0] + roi[2] / 2)))
    angle = int(round(angle_from_forward(line)))
    return line, (offset, angle, int(line.magnitude()))


def make_motor_command(measurement, previous_error, elapsed_ms):
    """
    PD循迹：校正量为正时左轮加速、右轮减速。
    STEERING_SIGN解决实际电机方向或画面镜像与本约定相反的情况。
    """
    if measurement is None:
        return 0, 0, 0

    offset, angle, _magnitude = measurement
    error = OFFSET_GAIN * offset + ANGLE_GAIN * angle
    derivative = 0
    if elapsed_ms > 0:
        derivative = (error - previous_error) * 1000.0 / elapsed_ms

    correction = STEERING_SIGN * (KP * error + KD * derivative)
    correction = clamp(correction, -MAX_SPEED, MAX_SPEED)

    left = int(round(clamp(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED)))
    right = int(round(clamp(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED)))
    return left, right, error


def make_frame(left, right, measurement):
    if measurement is None:
        return "$MOTOR,0,0,LOST,0,0#\n"
    offset, angle, magnitude = measurement
    return "$MOTOR,%d,%d,%d,%d,%d#\n" % (
        left, right, offset, angle, magnitude
    )


def draw_overlay(img, roi, line, measurement, left, right, stable_frames, fps):
    img.draw_rectangle(roi, color=YELLOW, thickness=2)
    img.draw_cross(FRAME_WIDTH // 2, roi[1] + roi[3] - 1, color=CYAN, thickness=1)

    if measurement is None:
        img.draw_string_advanced(2, 2, 18, "LOST -> L=0 R=0", color=WHITE)
    else:
        offset, angle, magnitude = measurement
        img.draw_line(line.line(), color=RED, thickness=2)
        color = GREEN if stable_frames >= LINE_STABLE_FRAMES else YELLOW
        text = "L=%+d R=%+d OF=%+d AN=%+d M=%d" % (
            left, right, offset, angle, magnitude
        )
        img.draw_string_advanced(2, 2, 16, text, color=color)
        img.draw_string_advanced(2, 21, 16, "stable=%d fps=%.1f" % (
            stable_frames, fps
        ), color=color)


def main():
    sensor = None
    uart = None
    roi = (0, ROI_Y, FRAME_WIDTH, ROI_HEIGHT)

    stable_frames = 0
    previous_error = 0
    previous_control_ms = ticks_ms()
    last_send_ms = 0

    try:
        # 早期明确报错，避免把ROI/速度参数错误误以为视觉失败。
        assert 0 <= ROI_Y < FRAME_HEIGHT
        assert 1 <= ROI_HEIGHT <= FRAME_HEIGHT - ROI_Y
        assert 0 <= BLACK_GRAY_MAX <= 255
        assert 0 <= BASE_SPEED <= MAX_SPEED <= 100
        assert LINE_STABLE_FRAMES >= 1

        sensor = init_sensor()
        init_display()
        uart, _fpioa = init_uart_if_enabled()
        sensor.run()
        clock = time.clock()

        print("K230 visual line FOLLOW controller started.")
        print("It calculates commands only; no K230 motor driver is used.")
        print("Commands enabled:", COMMAND_OUTPUT_ENABLED)
        print("ROI=(0,%d,%d,%d), black<=%d, base=%d, max=%d" % (
            ROI_Y, FRAME_WIDTH, ROI_HEIGHT, BLACK_GRAY_MAX, BASE_SPEED, MAX_SPEED
        ))

        while True:
            os.exitpoint()
            clock.tick()

            img = sensor.snapshot()
            line, measurement = detect_measurement(img, roi)

            now = ticks_ms()
            elapsed = ticks_diff(now, previous_control_ms)
            previous_control_ms = now

            if measurement is None:
                stable_frames = 0
                left, right, error = 0, 0, 0
                previous_error = 0
            else:
                stable_frames += 1
                # 未稳定前强制停车；稳定后才计算前进命令。
                if stable_frames < LINE_STABLE_FRAMES:
                    left, right, error = 0, 0, 0
                    previous_error = 0
                else:
                    left, right, error = make_motor_command(
                        measurement, previous_error, elapsed
                    )
                    previous_error = error

            draw_overlay(
                img, roi, line, measurement, left, right, stable_frames, clock.fps()
            )
            Display.show_image(img)

            if uart is not None and ticks_diff(now, last_send_ms) >= UART_REPEAT_MS:
                frame = make_frame(left, right, measurement)
                uart.write(frame)
                print(frame, end="")
                last_send_ms = now

            gc.collect()

    except KeyboardInterrupt:
        print("User stopped K230 visual line FOLLOW controller.")
    except Exception as exc:
        print("K230 visual line FOLLOW controller error:", exc)
    finally:
        # 即便后续接了接收端，退出前最后发送明确停车帧。
        if uart is not None:
            try:
                uart.write("$MOTOR,0,0,LOST,0,0#\n")
                uart.deinit()
            except Exception:
                pass
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


if __name__ == "__main__":
    main()

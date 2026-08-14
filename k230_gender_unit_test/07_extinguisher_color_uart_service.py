"""K230灭火器颜色识别独立测试：IO32触发，UART1单向返回。

接线：
- Nano A4经5V->3.3V分压后接K230 IO32，作为灭火器识别触发线
- K230 IO9/UART1_TX接Nano D12/SoftwareSerial RX
- K230与Nano共地
- K230 IO10只在FPIOA中配置为UART1_RXD，物理不连接

协议：
- 启动：K230_FIRE_READY\n
- 触发：Nano A4输出高电平脉冲
- 测量中：BUSY,FIRE\n
- 结果：RESULT,FIRE,RED|GREEN|BLUE|U,<largest_pixels>\n
颜色阈值和上半画面ROI来自用户提供的multi_color_detect.py。
"""

import gc
import os
import time

from machine import FPIOA
from machine import Pin
from machine import UART
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0
from media.sensor import Sensor


DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

COLOR_THRESHOLDS = {
    "RED": (0, 100, 35, 127, -128, 127),
    "GREEN": (0, 100, -128, -16, -128, 127),
    "BLUE": (0, 100, -128, 26, -128, -31),
}

DRAW_COLORS = {
    "RED": (255, 0, 0),
    "GREEN": (0, 255, 0),
    "BLUE": (0, 100, 255),
}

PIXELS_THRESHOLD = 300
AREA_THRESHOLD = 300
DETECT_ROI = (0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT // 2)
STABLE_MS = 500
MEASUREMENT_TIMEOUT_MS = 2500

UART_TX_PIN = 9
UART_RX_PIN = 10
FIRE_TRIGGER_PIN = 32
UART_BAUDRATE = 57600


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def send_line(uart, text):
    uart.write((text + "\n").encode())
    print("TX ->", text)


def draw_text(img, text, color=(255, 255, 255)):
    img.draw_string_advanced(8, 8, 28, text, color=color)


def detect_dominant_color(img):
    """返回(颜色名, 最大色块, 像素数)，没有有效色块时返回(None,None,0)。"""
    best_name = None
    best_blob = None
    best_pixels = 0

    for name, threshold in COLOR_THRESHOLDS.items():
        blobs = img.find_blobs(
            [threshold],
            roi=DETECT_ROI,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
        )
        if not blobs:
            continue

        largest = max(blobs, key=lambda blob: blob.pixels())
        pixels = largest.pixels()
        if pixels > best_pixels:
            best_name = name
            best_blob = largest
            best_pixels = pixels

    if best_blob is not None:
        color = DRAW_COLORS[best_name]
        img.draw_rectangle(best_blob[0:4], color=color, thickness=3)
        img.draw_cross(
            best_blob[5],
            best_blob[6],
            color=color,
            thickness=2,
        )
        label_y = max(best_blob[1] - 32, 0)
        img.draw_string_advanced(
            best_blob[0],
            label_y,
            28,
            best_name,
            color=color,
        )

    return best_name, best_blob, best_pixels


def measure_extinguisher_color(sensor):
    """要求同一颜色连续稳定500ms；颜色发生变化时重新计时。"""
    started_at = ticks_ms()
    stable_color = None
    stable_since = None
    stable_pixels = 0

    while ticks_diff(ticks_ms(), started_at) < MEASUREMENT_TIMEOUT_MS:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        detected, _, pixels = detect_dominant_color(img)
        now = ticks_ms()

        if detected is None:
            stable_color = None
            stable_since = None
            stable_pixels = 0
            status = "FIRE: SEARCHING"
        else:
            if detected != stable_color:
                stable_color = detected
                stable_since = now
                stable_pixels = pixels
            else:
                if pixels > stable_pixels:
                    stable_pixels = pixels

            stable_elapsed = ticks_diff(now, stable_since)
            status = "FIRE: {} {}ms".format(
                stable_color,
                stable_elapsed,
            )
            if stable_elapsed >= STABLE_MS:
                draw_text(img, "RESULT: " + stable_color)
                Display.show_image(img)
                return stable_color, stable_pixels

        draw_text(img, status)
        Display.show_image(img)
        gc.collect()

    return "U", 0


def main():
    sensor = None
    uart = None

    try:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
        # CanMV创建UART1时仍要求配置RX功能，IO10物理不接。
        fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
        fpioa.set_function(FIRE_TRIGGER_PIN, FPIOA.GPIO32)

        trigger = Pin(
            FIRE_TRIGGER_PIN,
            Pin.IN,
            pull=Pin.PULL_NONE,
            drive=7,
        )

        uart = UART(
            UART.UART1,
            baudrate=UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )

        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(
            Sensor.RGB565,
            chn=CAM_CHN_ID_0,
        )

        Display.init(Display.ST7701, to_ide=True)
        MediaManager.init()
        sensor.run()

        trigger_armed = trigger.value() == 0
        print("Extinguisher color service started")
        print("Fire trigger input: IO%d" % FIRE_TRIGGER_PIN)
        print("UART1 TX: IO%d baud=%d" % (
            UART_TX_PIN,
            UART_BAUDRATE,
        ))
        send_line(uart, "K230_FIRE_READY")

        while True:
            os.exitpoint()
            trigger_level = trigger.value()

            if trigger_level and trigger_armed:
                trigger_armed = False
                print("IO32 fire trigger rising edge detected")
                send_line(uart, "BUSY,FIRE")

                color_name, largest_pixels = \
                    measure_extinguisher_color(sensor)
                result = "RESULT,FIRE,{},{}".format(
                    color_name,
                    largest_pixels,
                )
                send_line(uart, result)

            elif not trigger_level:
                trigger_armed = True

            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            _, _, _ = detect_dominant_color(img)
            draw_text(img, "IO32 FIRE READY")
            Display.show_image(img)
            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("Extinguisher color service stopped by user")

    except Exception as error:
        if "IDE interrupt" in str(error):
            print("Extinguisher color service stopped by CanMV IDE")
        else:
            print("Extinguisher color service error:", error)
            raise

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
            uart.deinit()


if __name__ == "__main__":
    main()

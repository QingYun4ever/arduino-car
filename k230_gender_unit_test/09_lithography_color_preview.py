"""K230光刻机颜色识别独立测试。

把目标放到摄像头前，程序会在整个640x480画面中检测红、绿、蓝三种颜色，
选择像素数最大的色块作为当前颜色；同一颜色连续稳定500ms后在终端输出。
本脚本不使用Arduino、GPIO触发、UART或AI模型。
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0
from media.sensor import Sensor


DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# 沿用multi_color_detect.py的LAB阈值。
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

DETECT_ROI = (0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT)
PIXELS_THRESHOLD = 300
AREA_THRESHOLD = 300
STABLE_MS = 500


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def detect_dominant_color(img):
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
        img.draw_string_advanced(
            best_blob[0],
            max(best_blob[1] - 34, 0),
            30,
            "{} {}px".format(best_name, best_pixels),
            color=color,
        )

    return best_name, best_pixels


def main():
    sensor = None

    try:
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

        stable_color = None
        stable_since = None
        stable_pixels = 0
        reported_color = None

        print("Lithography color preview started")
        print("Detecting RED / GREEN / BLUE in the full frame")

        while True:
            os.exitpoint()
            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            detected, pixels = detect_dominant_color(img)
            now = ticks_ms()

            if detected is None:
                stable_color = None
                stable_since = None
                stable_pixels = 0
                reported_color = None
                status = "COLOR: SEARCHING"

            else:
                if detected != stable_color:
                    stable_color = detected
                    stable_since = now
                    stable_pixels = pixels
                    reported_color = None
                else:
                    if pixels > stable_pixels:
                        stable_pixels = pixels

                elapsed = ticks_diff(now, stable_since)
                if elapsed >= STABLE_MS:
                    status = "COLOR: {} STABLE".format(stable_color)
                    if reported_color != stable_color:
                        print(
                            "COLOR = {}  PIXELS = {}".format(
                                stable_color,
                                stable_pixels,
                            )
                        )
                        reported_color = stable_color
                else:
                    status = "COLOR: {} {}ms".format(
                        stable_color,
                        elapsed,
                    )

            img.draw_string_advanced(
                8,
                8,
                28,
                status,
                color=(255, 255, 255),
            )
            Display.show_image(img)
            gc.collect()
            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("Lithography color preview stopped by user")

    except Exception as error:
        if "IDE interrupt" in str(error):
            print("Lithography color preview stopped by CanMV IDE")
        else:
            print("Lithography color preview error:", error)
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


if __name__ == "__main__":
    main()

"""
K230 最小摄像头/Frame Buffer 诊断程序

用途：只确认三件事：
1) CanMV IDE 确实把当前程序运行到了 K230；
2) K230 摄像头能采集画面；
3) Frame Buffer 能显示由 Python 画出的叠加内容。

不做巡线、不输出UART、不控制Nano或任何电机。
运行此文件前，不要保存为 main.py。
"""

import gc
import os
import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

WIDTH = 320
HEIGHT = 240

# RGB565保证黄色/红色/绿色诊断图形在Frame Buffer里清楚可见。
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)


def main():
    sensor = None
    try:
        # 这行应该在点击绿色运行后立刻出现在CanMV IDE终端。
        print("[DIAG-1] Python program has started on K230.")

        sensor = Sensor(width=WIDTH, height=HEIGHT)
        sensor.reset()
        sensor.set_framesize(width=WIDTH, height=HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.VIRT, width=WIDTH, height=HEIGHT, fps=30)
        MediaManager.init()
        sensor.run()
        print("[DIAG-2] Camera and virtual display initialized.")

        frames = 0
        last_report_ms = time.ticks_ms() if hasattr(time, "ticks_ms") else 0

        while True:
            os.exitpoint()
            img = sensor.snapshot()

            # 故意画很粗、很明显的诊断边框与文字，不涉及任何识别算法。
            img.draw_rectangle(3, 3, WIDTH - 6, HEIGHT - 6, color=YELLOW, thickness=5)
            img.draw_cross(WIDTH // 2, HEIGHT // 2, color=RED, thickness=4)
            img.draw_string_advanced(12, 12, 24, "K230 CAMERA OK", color=GREEN)
            img.draw_string_advanced(12, 42, 18, "Frame=%d" % frames, color=GREEN)
            Display.show_image(img)

            frames += 1
            now = time.ticks_ms() if hasattr(time, "ticks_ms") else 0
            if not hasattr(time, "ticks_ms") or time.ticks_diff(now, last_report_ms) >= 1000:
                print("[DIAG-3] alive; frames=%d" % frames)
                last_report_ms = now
            gc.collect()

    except KeyboardInterrupt:
        print("[DIAG] stopped by user.")
    except Exception as exc:
        # 不能静默失败：完整异常会显示在CanMV IDE终端。
        print("[DIAG-ERROR]", repr(exc))
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

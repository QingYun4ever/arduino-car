"""CanMV K230 摄像头预览单元测试。

第一阶段只验证：摄像头能够初始化、持续取图，并在 CanMV IDE 中显示画面。
不包含性别识别、串口通信或小车控制逻辑。
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

sensor = None

try:
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    # VIRT 将画面发送到 CanMV IDE，不依赖具体型号的板载显示屏。
    Display.init(
        Display.VIRT,
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        fps=30,
    )

    MediaManager.init()
    sensor.run()

    print("Camera preview started: {}x{}".format(IMAGE_WIDTH, IMAGE_HEIGHT))

    while True:
        os.exitpoint()
        frame = sensor.snapshot()
        Display.show_image(frame)
        time.sleep_ms(1)

except KeyboardInterrupt:
    print("Camera preview stopped by user")

except Exception as error:
    print("Camera preview error:", error)
    raise

finally:
    if sensor is not None:
        sensor.stop()

    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    gc.collect()

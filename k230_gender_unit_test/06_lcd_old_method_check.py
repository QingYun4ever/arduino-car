"""Physical ST7701 LCD check using the old robot program's display path.

This test contains no AI, GPIO trigger, or UART logic. It isolates only:
camera -> 640x480 RGB565 snapshot -> physical ST7701 LCD + CanMV IDE.
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor


WIDTH = 640
HEIGHT = 480

sensor = None

try:
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    # This is intentionally identical to the old machine program's
    # physically verified display initialization.
    Display.init(
        Display.ST7701,
        to_ide=True,
    )

    MediaManager.init()
    sensor.run()

    print("Old-method LCD check started: 640x480 RGB565")

    while True:
        os.exitpoint()
        frame = sensor.snapshot()
        Display.show_image(frame)
        time.sleep_ms(1)

except KeyboardInterrupt:
    print("Old-method LCD check stopped")
except Exception as error:
    print("Old-method LCD check error:", error)
    raise
finally:
    if sensor is not None:
        sensor.stop()

    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    gc.collect()

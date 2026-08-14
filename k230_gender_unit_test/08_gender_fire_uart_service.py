"""K230性别+灭火器颜色统一视觉服务。

触发输入：
- Nano D11经5V->3.3V分压接K230 IO33：触发性别识别
- Nano A4经5V->3.3V分压接K230 IO32：触发灭火器颜色识别

单向UART返回：
- K230 IO9/UART1_TX -> Nano D12/SoftwareSerial RX，57600 baud
- 性别：BUSY,GENDER；RESULT,GENDER,M|F|U,<confidence_percent>
- 消防：BUSY,FIRE；RESULT,FIRE,RED|GREEN|BLUE|U,<largest_pixels>

摄像头和性别模型只初始化一次。灭火器颜色阈值及上半画面ROI来自用户
提供的multi_color_detect.py，并修正了颜色变化后稳定计时未重置的问题。

需要的文件：
- /sdcard/kmodel/face_detection_320.kmodel
- /sdcard/apps/tests/utils/prior_data_320.bin
- /sdcard/kmodel/face_gender.kmodel
"""

import gc
import image
import os
import time

from machine import FPIOA
from machine import Pin
from machine import UART

import aidemo
import nncase_runtime as nn
import ulab.numpy as np

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine, ScopedTiming
from media.display import *
from media.media import *
from media.sensor import *


DISPLAY_MODE = "lcd"
DISPLAY_SIZE = [640, 480]
RGB888P_SIZE = [640, 480]

FACE_DETECTION_INPUT_SIZE = [320, 320]
FACE_GENDER_INPUT_SIZE = [224, 224]

FACE_CONFIDENCE_THRESHOLD = 0.5
FACE_NMS_THRESHOLD = 0.2
FACE_MARGIN = 0.4

FACE_DETECTION_MODEL_PATH = \
    "/sdcard/kmodel/face_detection_320.kmodel"
FACE_GENDER_MODEL_PATH = \
    "/sdcard/kmodel/face_gender.kmodel"
ANCHORS_PATH = "/sdcard/apps/tests/utils/prior_data_320.bin"

ANCHOR_COUNT = 4200
ANCHOR_DIMENSION = 4

UART_TX_PIN = 9
UART_RX_PIN = 10  # Required by UART1 constructor; no physical RX wire is needed.
GENDER_TRIGGER_PIN = 33
FIRE_TRIGGER_PIN = 32
UART_BAUDRATE = 57600

MEASUREMENT_TIMEOUT_MS = 2000
MIN_CLASSIFICATION_CONFIDENCE = 0.70
MIN_VALID_SAMPLES = 5
MIN_AGREEMENT_PERCENT = 70

# 灭火器颜色参数，沿用multi_color_detect.py。
COLOR_THRESHOLDS = {
    "RED": (0, 100, 35, 127, -128, 127),
    "GREEN": (0, 100, -128, -16, -128, 127),
    "BLUE": (0, 100, -128, 26, -128, -31),
}
COLOR_DRAW_VALUES = {
    "RED": (255, 0, 0),
    "GREEN": (0, 255, 0),
    "BLUE": (0, 100, 255),
}
COLOR_PIXELS_THRESHOLD = 300
COLOR_AREA_THRESHOLD = 300
COLOR_DETECT_ROI = (0, 0, DISPLAY_SIZE[0], DISPLAY_SIZE[1] // 2)
COLOR_STABLE_MS = 500
COLOR_MEASUREMENT_TIMEOUT_MS = 2500


class St7701LandscapePipeLine(PipeLine):
    """Use the old robot program's verified RGB565 direct-LCD path."""

    def create(self, sensor=None, hmirror=None, vflip=None):
        with ScopedTiming("init ST7701 PipeLine", self.debug_mode > 0):
            os.exitpoint(os.EXITPOINT_ENABLE)
            nn.shrink_memory_pool()

            self.sensor = Sensor() if sensor is None else sensor
            self.sensor.reset()

            if hmirror is not None:
                self.sensor.set_hmirror(hmirror)
            if vflip is not None:
                self.sensor.set_vflip(vflip)

            # Channel 0 follows the old program's physically verified path:
            # 640x480 RGB565 snapshot -> Display.show_image().
            self.sensor.set_framesize(
                width=self.display_size[0],
                height=self.display_size[1],
                chn=CAM_CHN_ID_0,
            )
            self.sensor.set_pixformat(
                Sensor.RGB565,
                chn=CAM_CHN_ID_0,
            )

            # Channel 2 remains dedicated to planar RGB AI inference.
            self.sensor.set_framesize(
                width=self.rgb888p_size[0],
                height=self.rgb888p_size[1],
                chn=CAM_CHN_ID_2,
            )
            self.sensor.set_pixformat(
                PIXEL_FORMAT_RGB_888_PLANAR,
                chn=CAM_CHN_ID_2,
            )

            self.osd_img = image.Image(
                self.display_size[0],
                self.display_size[1],
                image.ARGB8888,
            )

            # Match the old machine program exactly; do not use a bound YUV
            # video layer because that path displayed only in CanMV IDE here.
            Display.init(
                Display.ST7701,
                to_ide=True,
            )

            MediaManager.init()
            self.sensor.run()

    def show_image(self):
        with ScopedTiming("show result", self.debug_mode > 0):
            lcd_frame = self.sensor.snapshot(chn=CAM_CHN_ID_0)
            Display.show_image(lcd_frame)
            Display.show_image(
                self.osd_img,
                0,
                0,
                Display.LAYER_OSD3,
            )

    def get_color_frame(self):
        return self.sensor.snapshot(chn=CAM_CHN_ID_0)

    def show_color_frame(self, frame):
        # 颜色框直接画在RGB565帧上；清空旧性别OSD避免重叠。
        self.osd_img.clear()
        Display.show_image(frame)
        Display.show_image(
            self.osd_img,
            0,
            0,
            Display.LAYER_OSD3,
        )


class FaceDetectionApp(AIBase):
    """封装官方 RetinaFace 人脸检测模型。"""

    def __init__(
        self,
        kmodel_path,
        model_input_size,
        anchors,
        confidence_threshold,
        nms_threshold,
        rgb888p_size,
        debug_mode=0,
    ):
        super().__init__(
            kmodel_path,
            model_input_size,
            rgb888p_size,
            debug_mode,
        )

        self.model_input_size = model_input_size
        self.anchors = anchors
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [
            ALIGN_UP(rgb888p_size[0], 16),
            rgb888p_size[1],
        ]
        self.debug_mode = debug_mode

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self):
        """把摄像头画面按比例缩放并填充到 320x320。"""
        with ScopedTiming("face detection preprocess config", self.debug_mode > 0):
            top, bottom, left, right = self._get_padding()
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [104, 117, 123],
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d.build(
                [1, 3, self.rgb888p_size[1], self.rgb888p_size[0]],
                [
                    1,
                    3,
                    self.model_input_size[1],
                    self.model_input_size[0],
                ],
            )

    def postprocess(self, results):
        """使用官方 aidemo 接口解码 RetinaFace 输出。"""
        with ScopedTiming("face detection postprocess", self.debug_mode > 0):
            decoded = aidemo.face_det_post_process(
                self.confidence_threshold,
                self.nms_threshold,
                self.model_input_size[1],
                self.anchors,
                self.rgb888p_size,
                results,
            )
            if len(decoded) == 0:
                return []
            return decoded[0]

    def _get_padding(self):
        destination_width = self.model_input_size[0]
        destination_height = self.model_input_size[1]

        ratio = min(
            destination_width / self.rgb888p_size[0],
            destination_height / self.rgb888p_size[1],
        )
        resized_width = int(ratio * self.rgb888p_size[0])
        resized_height = int(ratio * self.rgb888p_size[1])

        width_difference = destination_width - resized_width
        height_difference = destination_height - resized_height

        top = 0
        bottom = int(round(height_difference + 0.1))
        left = 0
        right = int(round(width_difference - 0.1))
        return top, bottom, left, right


class FaceGenderApp(AIBase):
    """封装嘉楠官方 face_gender.kmodel 性别分类模型。"""

    def __init__(
        self,
        kmodel_path,
        model_input_size,
        rgb888p_size,
        face_margin=0.4,
        debug_mode=0,
    ):
        super().__init__(
            kmodel_path,
            model_input_size,
            rgb888p_size,
            debug_mode,
        )

        self.model_input_size = model_input_size
        self.rgb888p_size = [
            ALIGN_UP(rgb888p_size[0], 16),
            rgb888p_size[1],
        ]
        self.face_margin = face_margin
        self.debug_mode = debug_mode

        self.ai2d = Ai2d(debug_mode)
        # 摄像头输入是 NCHW；官方性别模型输入是 NHWC: [1,224,224,3]。
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.RGB_packed,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, detection):
        """扩大人脸框 40%，裁剪后缩放至模型输入尺寸。"""
        with ScopedTiming("face gender preprocess config", self.debug_mode > 0):
            crop_x, crop_y, crop_width, crop_height = \
                self._get_expanded_crop(detection)

            self.ai2d.crop(
                crop_x,
                crop_y,
                crop_width,
                crop_height,
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d.build(
                [1, 3, self.rgb888p_size[1], self.rgb888p_size[0]],
                [
                    1,
                    self.model_input_size[1],
                    self.model_input_size[0],
                    3,
                ],
            )

    def postprocess(self, results):
        """按官方实现：第一个性别输出大于 0.5 时判为女性。"""
        with ScopedTiming("face gender postprocess", self.debug_mode > 0):
            gender_scores = results[0][0]
            female_score = float(gender_scores[0])
            male_score = float(gender_scores[1])

            if female_score > 0.5:
                return "F", female_score
            return "M", male_score

    def _get_expanded_crop(self, detection):
        x, y, width, height = map(
            lambda value: int(round(value, 0)),
            detection[:4],
        )

        x1 = max(int(x - self.face_margin * width), 0)
        y1 = max(int(y - self.face_margin * height), 0)
        x2 = min(
            int(x + width + self.face_margin * width),
            self.rgb888p_size[0] - 1,
        )
        y2 = min(
            int(y + height + self.face_margin * height),
            self.rgb888p_size[1] - 1,
        )

        crop_width = max(x2 - x1, 1)
        crop_height = max(y2 - y1, 1)
        return x1, y1, crop_width, crop_height


class FaceGenderUnitTest:
    """组合人脸检测和性别分类，并负责绘制结果。"""

    def __init__(
        self,
        face_detection_model_path,
        face_gender_model_path,
        anchors,
        rgb888p_size,
        display_size,
    ):
        self.rgb888p_size = [
            ALIGN_UP(rgb888p_size[0], 16),
            rgb888p_size[1],
        ]
        self.display_size = [
            ALIGN_UP(display_size[0], 16),
            display_size[1],
        ]

        self.face_detector = FaceDetectionApp(
            face_detection_model_path,
            model_input_size=FACE_DETECTION_INPUT_SIZE,
            anchors=anchors,
            confidence_threshold=FACE_CONFIDENCE_THRESHOLD,
            nms_threshold=FACE_NMS_THRESHOLD,
            rgb888p_size=self.rgb888p_size,
            debug_mode=0,
        )
        self.face_classifier = FaceGenderApp(
            face_gender_model_path,
            model_input_size=FACE_GENDER_INPUT_SIZE,
            rgb888p_size=self.rgb888p_size,
            face_margin=FACE_MARGIN,
            debug_mode=0,
        )

        self.face_detector.config_preprocess()

    def run(self, frame):
        detections = self.face_detector.run(frame)
        classifications = []

        for detection in detections:
            self.face_classifier.config_preprocess(detection)
            label, confidence = self.face_classifier.run(frame)
            classifications.append((label, confidence))

        return detections, classifications

    def run_largest_face(self, frame):
        """只分类画面中面积最大的人脸，避免背景人物干扰。"""
        detections = self.face_detector.run(frame)
        if len(detections) == 0:
            return [], []

        largest = detections[0]
        largest_area = float(largest[2]) * float(largest[3])
        for detection in detections:
            area = float(detection[2]) * float(detection[3])
            if area > largest_area:
                largest = detection
                largest_area = area

        self.face_classifier.config_preprocess(largest)
        label, confidence = self.face_classifier.run(frame)
        return [largest], [(label, confidence)]

    def draw_result(self, pipeline, detections, classifications):
        pipeline.osd_img.clear()

        for index, detection in enumerate(detections):
            x, y, width, height = map(
                lambda value: int(round(value, 0)),
                detection[:4],
            )
            label, confidence = classifications[index]

            display_x = x * self.display_size[0] // self.rgb888p_size[0]
            display_y = y * self.display_size[1] // self.rgb888p_size[1]
            display_width = width * self.display_size[0] // self.rgb888p_size[0]
            display_height = height * self.display_size[1] // self.rgb888p_size[1]

            if label == "F":
                color = (255, 255, 0, 255)
            else:
                color = (255, 0, 255, 0)

            pipeline.osd_img.draw_rectangle(
                display_x,
                display_y,
                display_width,
                display_height,
                color=color,
                thickness=3,
            )

            text_y = max(display_y - 32, 0)
            text = "{} {:.2f}".format(label, confidence)
            pipeline.osd_img.draw_string_advanced(
                display_x,
                text_y,
                28,
                text,
                color=color,
            )

    def deinit(self):
        self.face_classifier.deinit()
        self.face_detector.deinit()


def load_anchors(path):
    anchors = np.fromfile(path, dtype=np.float)
    return anchors.reshape((ANCHOR_COUNT, ANCHOR_DIMENSION))


def send_line(uart, text):
    uart.write((text + "\n").encode())
    print("TX ->", text)


def draw_status(pipeline, text, color=(255, 255, 255, 0)):
    pipeline.osd_img.clear()
    pipeline.osd_img.draw_string_advanced(
        10,
        10,
        28,
        text,
        color=color,
    )
    pipeline.show_image()


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
            roi=COLOR_DETECT_ROI,
            pixels_threshold=COLOR_PIXELS_THRESHOLD,
            area_threshold=COLOR_AREA_THRESHOLD,
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
        color = COLOR_DRAW_VALUES[best_name]
        img.draw_rectangle(best_blob[0:4], color=color, thickness=3)
        img.draw_cross(
            best_blob[5],
            best_blob[6],
            color=color,
            thickness=2,
        )
        img.draw_string_advanced(
            best_blob[0],
            max(best_blob[1] - 32, 0),
            28,
            best_name,
            color=color,
        )

    return best_name, best_pixels


def measure_extinguisher_color(pipeline):
    started_at = ticks_ms()
    stable_color = None
    stable_since = None
    stable_pixels = 0

    while ticks_diff(ticks_ms(), started_at) < COLOR_MEASUREMENT_TIMEOUT_MS:
        os.exitpoint()
        frame = pipeline.get_color_frame()
        detected, pixels = detect_dominant_color(frame)
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
            if stable_elapsed >= COLOR_STABLE_MS:
                frame.draw_string_advanced(
                    8,
                    8,
                    28,
                    "RESULT: " + stable_color,
                    color=(255, 255, 255),
                )
                pipeline.show_color_frame(frame)
                return stable_color, stable_pixels

        frame.draw_string_advanced(
            8,
            8,
            28,
            status,
            color=(255, 255, 255),
        )
        pipeline.show_color_frame(frame)
        gc.collect()

    return "U", 0


def agreement_reached(female_count, male_count):
    valid_count = female_count + male_count
    if valid_count < MIN_VALID_SAMPLES:
        return False

    winner_count = max(female_count, male_count)
    return (
        winner_count * 100
        >= valid_count * MIN_AGREEMENT_PERCENT
    )


def measure_gender(pipeline, gender_test):
    female_count = 0
    male_count = 0
    female_confidence_sum = 0.0
    male_confidence_sum = 0.0
    started_at = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), started_at) < MEASUREMENT_TIMEOUT_MS:
        os.exitpoint()
        frame = pipeline.get_frame()
        detections, classifications = gender_test.run_largest_face(frame)

        gender_test.draw_result(
            pipeline,
            detections,
            classifications,
        )

        if len(classifications) > 0:
            label, confidence = classifications[0]
            if confidence >= MIN_CLASSIFICATION_CONFIDENCE:
                if label == "F":
                    female_count += 1
                    female_confidence_sum += confidence
                else:
                    male_count += 1
                    male_confidence_sum += confidence

        valid_count = female_count + male_count
        status = "TRIGGER  F:{} M:{}".format(
            female_count,
            male_count,
        )
        pipeline.osd_img.draw_string_advanced(
            10,
            10,
            28,
            status,
            color=(255, 255, 255, 0),
        )
        pipeline.show_image()

        if agreement_reached(female_count, male_count):
            break

        gc.collect()

    valid_count = female_count + male_count
    if not agreement_reached(female_count, male_count):
        return "U", 0.0, valid_count

    if female_count > male_count:
        return (
            "F",
            female_confidence_sum / female_count,
            valid_count,
        )

    return (
        "M",
        male_confidence_sum / male_count,
        valid_count,
    )


def main():
    pipeline = None
    gender_test = None
    uart = None

    try:
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
        fpioa.set_function(GENDER_TRIGGER_PIN, FPIOA.GPIO33)
        fpioa.set_function(FIRE_TRIGGER_PIN, FPIOA.GPIO32)

        gender_trigger = Pin(
            GENDER_TRIGGER_PIN,
            Pin.IN,
            pull=Pin.PULL_NONE,
            drive=7,
        )
        fire_trigger = Pin(
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

        anchors = load_anchors(ANCHORS_PATH)

        pipeline = St7701LandscapePipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create()

        gender_test = FaceGenderUnitTest(
            FACE_DETECTION_MODEL_PATH,
            FACE_GENDER_MODEL_PATH,
            anchors,
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
        )

        gender_trigger_armed = gender_trigger.value() == 0
        fire_trigger_armed = fire_trigger.value() == 0
        draw_status(pipeline, "IO33 GENDER / IO32 FIRE READY")
        print("Gender + extinguisher vision service started")
        print("Gender trigger input: IO%d" % GENDER_TRIGGER_PIN)
        print("Fire trigger input: IO%d" % FIRE_TRIGGER_PIN)
        print("UART1 TX: IO%d baud=%d" % (
            UART_TX_PIN,
            UART_BAUDRATE,
        ))
        send_line(uart, "K230_VISION_READY")

        while True:
            os.exitpoint()
            gender_level = gender_trigger.value()
            fire_level = fire_trigger.value()

            if gender_level and gender_trigger_armed:
                gender_trigger_armed = False
                print("IO33 gender trigger rising edge detected")
                send_line(uart, "BUSY,GENDER")
                draw_status(pipeline, "MEASURING GENDER")

                label, confidence, valid_count = measure_gender(
                    pipeline,
                    gender_test,
                )
                confidence_percent = int(confidence * 100 + 0.5)
                result = "RESULT,GENDER,{},{}".format(
                    label,
                    confidence_percent,
                )
                send_line(uart, result)
                print("Valid gender samples:", valid_count)

                if label == "U":
                    draw_status(
                        pipeline,
                        "GENDER UNKNOWN",
                        color=(255, 255, 0, 0),
                    )
                else:
                    draw_status(
                        pipeline,
                        "GENDER {} {}%".format(
                            label,
                            confidence_percent,
                        ),
                        color=(255, 0, 255, 0),
                    )

            elif fire_level and fire_trigger_armed:
                fire_trigger_armed = False
                print("IO32 fire trigger rising edge detected")
                send_line(uart, "BUSY,FIRE")
                draw_status(pipeline, "MEASURING FIRE COLOR")

                color_name, largest_pixels = \
                    measure_extinguisher_color(pipeline)
                result = "RESULT,FIRE,{},{}".format(
                    color_name,
                    largest_pixels,
                )
                send_line(uart, result)
                draw_status(
                    pipeline,
                    "FIRE {}".format(color_name),
                    color=(255, 255, 255, 0),
                )

            if not gender_level:
                gender_trigger_armed = True
            if not fire_level:
                fire_trigger_armed = True

            # Keep the physical LCD live while waiting for the next trigger.
            pipeline.show_image()
            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("Gender + extinguisher service stopped by user")

    except Exception as error:
        if "IDE interrupt" in str(error):
            print("Gender + extinguisher service stopped by CanMV IDE")
        else:
            print("Gender + extinguisher service error:", error)
            raise

    finally:
        if gender_test is not None:
            gender_test.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            uart.deinit()


if __name__ == "__main__":
    main()

"""K230性别、消防、晶圆桶形状、光刻机状态四任务视觉服务。

IO32/IO33作为两个二进制任务位，位序为[IO32 IO33]：
- 00：空闲
- 01：性别识别（Nano D11->K230 IO33）
- 10：消防颜色（Nano A4->K230 IO32）
- 11短脉冲：晶圆桶形状（保持约500ms）
- 11长脉冲：光刻机状态牌（保持至少900ms）

11通过保持时间区分形状和光刻机，不增加接线。任务完成后必须回到00。

单向UART返回：
- 性别：BUSY,GENDER；RESULT,GENDER,M|F|U,<confidence_percent>
- 消防：BUSY,FIRE；RESULT,FIRE,RED|GREEN|BLUE|U,<largest_pixels>
- 形状：BUSY,SHAPE；RESULT,SHAPE,Cylinder|tube|U,<score_percent>
- 光刻机：BUSY,LITHO；RESULT,LITHO,RED|GREEN|U,<largest_pixels>

需要的文件：
- /sdcard/kmodel/face_detection_320.kmodel
- /sdcard/apps/tests/utils/prior_data_320.bin
- /sdcard/kmodel/face_gender.kmodel
- /sdcard/kmodel/recognition.kmodel
- /sdcard/utils/features/Cylinder_*.bin与tube_*.bin
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
SHAPE_MODEL_INPUT_SIZE = [224, 224]
# 三任务服务复用已经稳定工作的AI通道2，不再从通道1取形状帧。
# 640x480中心200x200与原1280x960中心400x400视野比例一致。
SHAPE_RGB888P_SIZE = RGB888P_SIZE

# 人脸检测覆盖完整640x480画面，再缩放到模型固定的320x320输入。
# 远处人脸像素较少，将检测阈值由0.50降到0.35；多帧投票抑制误检。
FACE_CONFIDENCE_THRESHOLD = 0.35
FACE_NMS_THRESHOLD = 0.2
FACE_MARGIN = 0.4

FACE_DETECTION_MODEL_PATH = \
    "/sdcard/kmodel/face_detection_320.kmodel"
FACE_GENDER_MODEL_PATH = \
    "/sdcard/kmodel/face_gender.kmodel"
ANCHORS_PATH = "/sdcard/apps/tests/utils/prior_data_320.bin"
SHAPE_MODEL_PATH = "/sdcard/kmodel/recognition.kmodel"
SHAPE_DATABASE_PATH = "/sdcard/utils/features/"
SHAPE_LABELS = ["Cylinder", "tube"]
SHAPE_SIMILARITY_THRESHOLD = 0.45
SHAPE_CROP_WIDTH = 200
SHAPE_CROP_HEIGHT = 200
SHAPE_MEASUREMENT_TIMEOUT_MS = 2500
SHAPE_MIN_VALID_SAMPLES = 3
SHAPE_MIN_AGREEMENT_PERCENT = 67

ANCHOR_COUNT = 4200
ANCHOR_DIMENSION = 4

UART_TX_PIN = 9
UART_RX_PIN = 10  # Required by UART1 constructor; no physical RX wire is needed.
TASK_BIT0_PIN = 33  # IO33: binary bit0; Nano D11
TASK_BIT1_PIN = 32  # IO32: binary bit1; Nano A4
GENDER_TRIGGER_PIN = TASK_BIT0_PIN
FIRE_TRIGGER_PIN = TASK_BIT1_PIN
TASK_CODE_DEBOUNCE_MS = 20
TASK_CODE_11_LONG_THRESHOLD_MS = 900
UART_BAUDRATE = 57600

MEASUREMENT_TIMEOUT_MS = 3000
MIN_CLASSIFICATION_CONFIDENCE = 0.65
MIN_VALID_SAMPLES = 3
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
FIRE_COLOR_DETECT_ROI = \
    (0, 0, DISPLAY_SIZE[0], DISPLAY_SIZE[1] // 2)
LITHOGRAPHY_COLOR_DETECT_ROI = \
    (0, 0, DISPLAY_SIZE[0], DISPLAY_SIZE[1])
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

            # Channel 2 remains dedicated to face detection/gender inference.
            self.sensor.set_framesize(
                width=self.rgb888p_size[0],
                height=self.rgb888p_size[1],
                chn=CAM_CHN_ID_2,
            )
            self.sensor.set_pixformat(
                PIXEL_FORMAT_RGB_888_PLANAR,
                chn=CAM_CHN_ID_2,
            )

            # 形状和性别顺序复用通道2。独立形状测试已经验证通道2，
            # 避免三任务版本在通道1 snapshot后阻塞并冻结LCD。

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

    def get_shape_frame(self):
        return self.sensor.snapshot(chn=CAM_CHN_ID_2)

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


class ShapeFeatureApp(AIBase):
    """05.self_learning.py中的中心裁剪特征提取模型。"""

    def __init__(self):
        super().__init__(
            SHAPE_MODEL_PATH,
            SHAPE_MODEL_INPUT_SIZE,
            SHAPE_RGB888P_SIZE,
            0,
        )
        self.model_input_size = SHAPE_MODEL_INPUT_SIZE
        self.rgb888p_size = [
            ALIGN_UP(SHAPE_RGB888P_SIZE[0], 16),
            SHAPE_RGB888P_SIZE[1],
        ]
        self.crop_x = (
            self.rgb888p_size[0] - SHAPE_CROP_WIDTH
        ) // 2
        self.crop_y = (
            self.rgb888p_size[1] - SHAPE_CROP_HEIGHT
        ) // 2

        self.ai2d = Ai2d(0)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self):
        self.ai2d.crop(
            self.crop_x,
            self.crop_y,
            SHAPE_CROP_WIDTH,
            SHAPE_CROP_HEIGHT,
        )
        self.ai2d.resize(
            nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel,
        )
        self.ai2d.build(
            [
                1,
                3,
                self.rgb888p_size[1],
                self.rgb888p_size[0],
            ],
            [
                1,
                3,
                self.model_input_size[1],
                self.model_input_size[0],
            ],
        )

    def postprocess(self, results):
        return results[0][0]


class ShapeRecognizer:
    def __init__(self):
        self.feature_app = ShapeFeatureApp()
        self.feature_app.config_preprocess()
        self.database = []
        self.counts = {}
        for label in SHAPE_LABELS:
            self.counts[label] = 0
        self._load_database()

    def _load_database(self):
        try:
            file_names = os.listdir(SHAPE_DATABASE_PATH)
        except Exception as error:
            print("Shape database unavailable:", error)
            return

        for file_name in file_names:
            label = None
            for candidate in SHAPE_LABELS:
                if file_name.startswith(candidate + "_") and \
                        file_name.endswith(".bin"):
                    label = candidate
                    break
            if label is None:
                continue

            try:
                with open(
                    SHAPE_DATABASE_PATH + file_name,
                    "rb",
                ) as feature_file:
                    data = feature_file.read()
                vector = np.frombuffer(data, dtype=np.float)
                self.database.append((label, vector))
                self.counts[label] += 1
            except Exception as error:
                print("Skip shape feature:", file_name, error)

        print("Shape feature counts:", self.counts)

    def database_ready(self):
        for label in SHAPE_LABELS:
            if self.counts[label] < 5:
                return False
        return True

    def _similarity(self, vector_a, vector_b):
        dot_product = sum(vector_a * vector_b)
        norm_a = np.sqrt(sum(vector_a * vector_a))
        norm_b = np.sqrt(sum(vector_b * vector_b))
        if norm_a == 0 or norm_b == 0:
            return 0.0
        return float(dot_product / (norm_a * norm_b))

    def run(self, frame):
        if not self.database_ready():
            return "U", 0.0

        feature = self.feature_app.run(frame)
        best_label = "U"
        best_score = 0.0

        for label, saved_feature in self.database:
            score = self._similarity(feature, saved_feature)
            if score > best_score:
                best_label = label
                best_score = score

        if best_score <= SHAPE_SIMILARITY_THRESHOLD:
            return "U", best_score
        return best_label, best_score

    def draw_result(self, pipeline, label, score, sample_count):
        pipeline.osd_img.clear()
        box_x = int(
            ((SHAPE_RGB888P_SIZE[0] - SHAPE_CROP_WIDTH) // 2)
            * DISPLAY_SIZE[0]
            / SHAPE_RGB888P_SIZE[0]
        )
        box_y = int(
            ((SHAPE_RGB888P_SIZE[1] - SHAPE_CROP_HEIGHT) // 2)
            * DISPLAY_SIZE[1]
            / SHAPE_RGB888P_SIZE[1]
        )
        box_width = int(
            SHAPE_CROP_WIDTH
            * DISPLAY_SIZE[0]
            / SHAPE_RGB888P_SIZE[0]
        )
        box_height = int(
            SHAPE_CROP_HEIGHT
            * DISPLAY_SIZE[1]
            / SHAPE_RGB888P_SIZE[1]
        )

        if label == "U":
            color = (255, 255, 0, 0)
        else:
            color = (255, 0, 255, 0)

        pipeline.osd_img.draw_rectangle(
            box_x,
            box_y,
            box_width,
            box_height,
            color=color,
            thickness=4,
        )
        pipeline.osd_img.draw_string_advanced(
            10,
            10,
            26,
            "SHAPE {} {}% N{}".format(
                label,
                int(score * 100 + 0.5),
                sample_count,
            ),
            color=color,
        )

    def deinit(self):
        self.feature_app.deinit()


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


def detect_dominant_color(img, roi, color_names):
    best_name = None
    best_blob = None
    best_pixels = 0

    for name in color_names:
        threshold = COLOR_THRESHOLDS[name]
        blobs = img.find_blobs(
            [threshold],
            roi=roi,
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
        detected, pixels = detect_dominant_color(
            frame,
            FIRE_COLOR_DETECT_ROI,
            ("RED", "GREEN", "BLUE"),
        )
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


def measure_lithography_color(pipeline):
    """检测整个画面的红/绿状态牌；蓝色不属于光刻机计分状态。"""
    started_at = ticks_ms()
    stable_color = None
    stable_since = None
    stable_pixels = 0

    while ticks_diff(ticks_ms(), started_at) < COLOR_MEASUREMENT_TIMEOUT_MS:
        os.exitpoint()
        frame = pipeline.get_color_frame()
        detected, pixels = detect_dominant_color(
            frame,
            LITHOGRAPHY_COLOR_DETECT_ROI,
            ("RED", "GREEN"),
        )
        now = ticks_ms()

        if detected is None:
            stable_color = None
            stable_since = None
            stable_pixels = 0
            status = "LITHO: SEARCHING"
        else:
            if detected != stable_color:
                stable_color = detected
                stable_since = now
                stable_pixels = pixels
            elif pixels > stable_pixels:
                stable_pixels = pixels

            stable_elapsed = ticks_diff(now, stable_since)
            status = "LITHO: {} {}ms".format(
                stable_color,
                stable_elapsed,
            )
            if stable_elapsed >= COLOR_STABLE_MS:
                frame.draw_string_advanced(
                    8,
                    8,
                    28,
                    "LITHO RESULT: " + stable_color,
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
        status = "FACE:{}  F:{} M:{}".format(
            len(detections),
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


def measure_shape(pipeline, shape_recognizer):
    if not shape_recognizer.database_ready():
        draw_status(
            pipeline,
            "SHAPE DB INCOMPLETE",
            color=(255, 255, 0, 0),
        )
        return "U", 0.0, 0

    counts = {}
    score_sums = {}
    for label in SHAPE_LABELS:
        counts[label] = 0
        score_sums[label] = 0.0

    started_at = ticks_ms()
    sample_number = 0

    while ticks_diff(ticks_ms(), started_at) < \
            SHAPE_MEASUREMENT_TIMEOUT_MS:
        os.exitpoint()
        sample_number += 1
        print("SHAPE sample", sample_number, "capture")
        frame = pipeline.get_shape_frame()
        print("SHAPE sample", sample_number, "inference")
        label, score = shape_recognizer.run(frame)
        print(
            "SHAPE sample",
            sample_number,
            "done:",
            label,
            score,
        )

        if label in counts:
            counts[label] += 1
            score_sums[label] += score

        valid_count = sum(counts.values())
        shape_recognizer.draw_result(
            pipeline,
            label,
            score,
            valid_count,
        )
        pipeline.show_image()

        if valid_count >= SHAPE_MIN_VALID_SAMPLES:
            winner_count = max(counts.values())
            if winner_count * 100 >= \
                    valid_count * SHAPE_MIN_AGREEMENT_PERCENT:
                break

        gc.collect()

    valid_count = sum(counts.values())
    if valid_count < SHAPE_MIN_VALID_SAMPLES:
        return "U", 0.0, valid_count

    winner_label = SHAPE_LABELS[0]
    for label in SHAPE_LABELS:
        if counts[label] > counts[winner_label]:
            winner_label = label

    winner_count = counts[winner_label]
    if winner_count * 100 < \
            valid_count * SHAPE_MIN_AGREEMENT_PERCENT:
        return "U", 0.0, valid_count

    return (
        winner_label,
        score_sums[winner_label] / winner_count,
        valid_count,
    )


def read_task_code(bit0_pin, bit1_pin):
    bit0 = 1 if bit0_pin.value() else 0
    bit1 = 1 if bit1_pin.value() else 0
    return (bit1 << 1) | bit0


def resolve_task_code(task_code, bit0_pin, bit1_pin):
    """将11短脉冲解析为SHAPE，11长脉冲解析为LITHO。"""
    if task_code != 3:
        return task_code

    held_since = ticks_ms()
    while read_task_code(bit0_pin, bit1_pin) == 3:
        held_ms = ticks_diff(ticks_ms(), held_since)
        if held_ms >= TASK_CODE_11_LONG_THRESHOLD_MS:
            print("11 held", held_ms, "ms -> LITHO")
            return 4
        os.exitpoint()
        time.sleep_ms(10)

    held_ms = ticks_diff(ticks_ms(), held_since)
    print("11 held", held_ms, "ms -> SHAPE")
    return 3


def release_kpu_memory():
    """在切换人脸模型和形状模型后立即回收KPU/堆内存。"""
    gc.collect()
    nn.shrink_memory_pool()
    gc.collect()


def main():
    pipeline = None
    gender_test = None
    shape_recognizer = None
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

        # 启动时只加载首次会使用的人脸检测+性别模型。
        # recognition.kmodel不再与两个人脸模型同时常驻；独立形状测试
        # 正常而综合服务卡死，最明显差异就是三套KPU模型同时占用资源。
        print("Loading initial face models")
        gender_test = FaceGenderUnitTest(
            FACE_DETECTION_MODEL_PATH,
            FACE_GENDER_MODEL_PATH,
            anchors,
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
        )
        shape_recognizer = None

        task_armed = read_task_code(
            gender_trigger,
            fire_trigger,
        ) == 0
        draw_status(pipeline, "01 GENDER 10 FIRE 11 SHORT/LONG")
        print("Four-task vision service started")
        print("Task bits [IO32 IO33]:")
        print("  00 idle")
        print("  01 gender")
        print("  10 fire color")
        print("  11 short pulse: wafer shape")
        print("  11 long pulse: lithography color")
        print("UART1 TX: IO%d baud=%d" % (
            UART_TX_PIN,
            UART_BAUDRATE,
        ))
        send_line(uart, "K230_VISION4_READY")

        while True:
            os.exitpoint()
            task_code = read_task_code(
                gender_trigger,
                fire_trigger,
            )

            if task_code == 0:
                task_armed = True

            elif task_armed:
                # 两个Arduino输出不是同一条指令完成，等待后再确认11。
                time.sleep_ms(TASK_CODE_DEBOUNCE_MS)
                task_code = read_task_code(
                    gender_trigger,
                    fire_trigger,
                )
                if task_code == 0:
                    continue

                task_code = resolve_task_code(
                    task_code,
                    gender_trigger,
                    fire_trigger,
                )
                task_armed = False
                print("Task code detected:", task_code)

                if task_code == 1:
                    send_line(uart, "BUSY,GENDER")
                    draw_status(pipeline, "LOADING/MEASURING GENDER")

                    # 形状任务结束后才按需重新加载人脸模型，避免两类模型共存。
                    if gender_test is None:
                        print("Loading face models on demand")
                        gender_test = FaceGenderUnitTest(
                            FACE_DETECTION_MODEL_PATH,
                            FACE_GENDER_MODEL_PATH,
                            anchors,
                            rgb888p_size=RGB888P_SIZE,
                            display_size=DISPLAY_SIZE,
                        )

                    label, confidence, valid_count = measure_gender(
                        pipeline,
                        gender_test,
                    )
                    confidence_percent = int(
                        confidence * 100 + 0.5
                    )
                    send_line(
                        uart,
                        "RESULT,GENDER,{},{}".format(
                            label,
                            confidence_percent,
                        ),
                    )
                    print("Valid gender samples:", valid_count)
                    draw_status(
                        pipeline,
                        "GENDER {} {}%".format(
                            label,
                            confidence_percent,
                        ),
                        color=(255, 0, 255, 0),
                    )

                elif task_code == 2:
                    send_line(uart, "BUSY,FIRE")
                    draw_status(pipeline, "MEASURING FIRE COLOR")

                    color_name, largest_pixels = \
                        measure_extinguisher_color(pipeline)
                    send_line(
                        uart,
                        "RESULT,FIRE,{},{}".format(
                            color_name,
                            largest_pixels,
                        ),
                    )
                    draw_status(
                        pipeline,
                        "FIRE {}".format(color_name),
                        color=(255, 255, 255, 0),
                    )

                elif task_code == 3:
                    send_line(uart, "BUSY,SHAPE")
                    draw_status(pipeline, "SWITCHING TO SHAPE MODEL")

                    # 独立形状程序只加载recognition.kmodel且不会卡死。
                    # 综合程序在进入SHAPE前先释放人脸检测和性别模型，确保
                    # KPU中只保留当前形状模型，完成后再立即释放。
                    if gender_test is not None:
                        print("Unloading face models before SHAPE")
                        gender_test.deinit()
                        gender_test = None
                        release_kpu_memory()

                    print("Loading shape model on demand")
                    shape_recognizer = ShapeRecognizer()
                    draw_status(pipeline, "MEASURING SHAPE")

                    label, score, valid_count = measure_shape(
                        pipeline,
                        shape_recognizer,
                    )
                    score_percent = int(score * 100 + 0.5)
                    send_line(
                        uart,
                        "RESULT,SHAPE,{},{}".format(
                            label,
                            score_percent,
                        ),
                    )
                    print("Valid shape samples:", valid_count)

                    print("Unloading shape model after SHAPE")
                    shape_recognizer.deinit()
                    shape_recognizer = None
                    release_kpu_memory()

                    draw_status(
                        pipeline,
                        "SHAPE {} {}%".format(
                            label,
                            score_percent,
                        ),
                        color=(255, 0, 255, 0),
                    )

                elif task_code == 4:
                    send_line(uart, "BUSY,LITHO")
                    draw_status(pipeline, "MEASURING LITHO COLOR")

                    color_name, largest_pixels = \
                        measure_lithography_color(pipeline)
                    send_line(
                        uart,
                        "RESULT,LITHO,{},{}".format(
                            color_name,
                            largest_pixels,
                        ),
                    )
                    draw_status(
                        pipeline,
                        "LITHO {}".format(color_name),
                        color=(255, 255, 255, 0),
                    )

            pipeline.show_image()
            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("Four-task vision service stopped by user")

    except Exception as error:
        if "IDE interrupt" in str(error):
            print("Four-task vision service stopped by CanMV IDE")
        else:
            print("Four-task vision service error:", error)
            raise

    finally:
        if shape_recognizer is not None:
            shape_recognizer.deinit()
        if gender_test is not None:
            gender_test.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            uart.deinit()


if __name__ == "__main__":
    main()

"""CanMV K230 人脸性别分类单元测试。

处理流程：摄像头取图 -> 人脸检测 -> 裁剪人脸 -> 性别分类 -> 显示 M/F。
本程序只验证视觉链路，不包含串口通信、小车控制或路线逻辑。

需要的文件：
- /sdcard/kmodel/face_detection_320.kmodel
- /sdcard/apps/tests/utils/prior_data_320.bin
- /sdcard/kmodel/face_gender.kmodel
"""

import gc
import os

import aidemo
import nncase_runtime as nn
import ulab.numpy as np

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine, ScopedTiming
from media.media import *


DISPLAY_MODE = "lcd"
DISPLAY_SIZE = [800, 480]
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


def main():
    pipeline = None
    gender_test = None

    try:
        anchors = load_anchors(ANCHORS_PATH)

        pipeline = PipeLine(
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

        print("Face gender test started: M=male, F=female")

        while True:
            os.exitpoint()
            frame = pipeline.get_frame()
            detections, classifications = gender_test.run(frame)
            gender_test.draw_result(
                pipeline,
                detections,
                classifications,
            )
            pipeline.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("Face gender test stopped by user")

    except Exception as error:
        print("Face gender test error:", error)
        raise

    finally:
        if gender_test is not None:
            gender_test.deinit()
        if pipeline is not None:
            pipeline.destroy()


if __name__ == "__main__":
    main()

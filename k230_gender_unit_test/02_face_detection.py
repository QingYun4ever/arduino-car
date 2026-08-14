"""CanMV K230 人脸检测单元测试。

本阶段只验证：摄像头画面中能够稳定检测并框出人脸。
不包含性别分类、串口通信或小车控制逻辑。
"""

import gc
import os
import sys

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
MODEL_INPUT_SIZE = [320, 320]

CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.2

MODEL_PATH = "/sdcard/app/tests/kmodel/face_detection_320.kmodel"
ANCHORS_PATH = "/sdcard/app/tests/utils/prior_data_320.bin"
ANCHOR_COUNT = 4200
ANCHOR_DIMENSION = 4


class FaceDetectionApp(AIBase):
    """封装官方 RetinaFace KModel 的预处理、推理和后处理。"""

    def __init__(
        self,
        kmodel_path,
        model_input_size,
        anchors,
        confidence_threshold,
        nms_threshold,
        rgb888p_size,
        display_size,
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
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self):
        """把摄像头画面按比例缩放并填充到 320×320。"""
        with ScopedTiming("face preprocess config", self.debug_mode > 0):
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
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def postprocess(self, results):
        """使用官方 aidemo 接口解码 RetinaFace 输出。"""
        with ScopedTiming("face postprocess", self.debug_mode > 0):
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

    def draw_result(self, pipeline, detections):
        """在显示画面上绘制所有检测到的人脸框。"""
        pipeline.osd_img.clear()

        for detection in detections:
            x, y, width, height = map(
                lambda value: int(round(value, 0)),
                detection[:4],
            )

            display_x = x * self.display_size[0] // self.rgb888p_size[0]
            display_y = y * self.display_size[1] // self.rgb888p_size[1]
            display_width = width * self.display_size[0] // self.rgb888p_size[0]
            display_height = height * self.display_size[1] // self.rgb888p_size[1]

            pipeline.osd_img.draw_rectangle(
                display_x,
                display_y,
                display_width,
                display_height,
                color=(255, 0, 255, 0),
                thickness=3,
            )

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


def load_anchors(path):
    anchors = np.fromfile(path, dtype=np.float)
    return anchors.reshape((ANCHOR_COUNT, ANCHOR_DIMENSION))


def main():
    anchors = load_anchors(ANCHORS_PATH)

    pipeline = PipeLine(
        rgb888p_size=RGB888P_SIZE,
        display_size=DISPLAY_SIZE,
        display_mode=DISPLAY_MODE,
    )
    pipeline.create()

    face_detector = FaceDetectionApp(
        MODEL_PATH,
        model_input_size=MODEL_INPUT_SIZE,
        anchors=anchors,
        confidence_threshold=CONFIDENCE_THRESHOLD,
        nms_threshold=NMS_THRESHOLD,
        rgb888p_size=RGB888P_SIZE,
        display_size=DISPLAY_SIZE,
        debug_mode=0,
    )
    face_detector.config_preprocess()

    print("Face detection started")

    try:
        while True:
            os.exitpoint()
            frame = pipeline.get_frame()
            detections = face_detector.run(frame)
            face_detector.draw_result(pipeline, detections)
            pipeline.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("Face detection stopped by user")

    except Exception as error:
        sys.print_exception(error)
        raise

    finally:
        face_detector.deinit()
        pipeline.destroy()


if __name__ == "__main__":
    main()

"""K230晶圆桶形状自学习/识别独立测试。

来源：适配D:/dev/Robot/小车/src/05.self_learning.py，删除Yahboom私有UART协议，
保留recognition.kmodel、中心400x400裁剪、Cylinder/tube标签、每类5个特征和
0.45相似度阈值。

所需文件：
- /sdcard/kmodel/recognition.kmodel
- 特征目录：/sdcard/utils/features/

首次运行且特征不足时：
1. 按屏幕提示把Cylinder放在中心黄框内，自动采集到5个特征。
2. 再把tube放在中心黄框内，自动采集到5个特征。
3. 采集完成后自动进入连续识别。
已有完整特征库时直接识别，无需Arduino、GPIO或UART。
"""

import gc
import image
import os
import time

import nncase_runtime as nn
import ulab.numpy as np

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine, ScopedTiming
from media.display import *
from media.media import *
from media.sensor import *


DISPLAY_SIZE = [640, 480]
RGB888P_SIZE = [1280, 960]
MODEL_INPUT_SIZE = [224, 224]
MODEL_PATH = "/sdcard/kmodel/recognition.kmodel"
DATABASE_PATH = "/sdcard/utils/features/"

LABELS = ["Cylinder", "tube"]
FEATURES_PER_LABEL = 5
COLLECT_INTERVAL_FRAMES = 60
SIMILARITY_THRESHOLD = 0.45
CROP_WIDTH = 400
CROP_HEIGHT = 400


class ShapePreviewPipeLine(PipeLine):
    def create(self, sensor=None, hmirror=None, vflip=None):
        os.exitpoint(os.EXITPOINT_ENABLE)
        nn.shrink_memory_pool()

        self.sensor = Sensor() if sensor is None else sensor
        self.sensor.reset()

        if hmirror is not None:
            self.sensor.set_hmirror(hmirror)
        if vflip is not None:
            self.sensor.set_vflip(vflip)

        self.sensor.set_framesize(
            width=DISPLAY_SIZE[0],
            height=DISPLAY_SIZE[1],
            chn=CAM_CHN_ID_0,
        )
        self.sensor.set_pixformat(
            Sensor.RGB565,
            chn=CAM_CHN_ID_0,
        )

        self.sensor.set_framesize(
            width=RGB888P_SIZE[0],
            height=RGB888P_SIZE[1],
            chn=CAM_CHN_ID_2,
        )
        self.sensor.set_pixformat(
            PIXEL_FORMAT_RGB_888_PLANAR,
            chn=CAM_CHN_ID_2,
        )

        self.osd_img = image.Image(
            DISPLAY_SIZE[0],
            DISPLAY_SIZE[1],
            image.ARGB8888,
        )

        Display.init(Display.ST7701, to_ide=True)
        MediaManager.init()
        self.sensor.run()

    def show_image(self):
        frame = self.sensor.snapshot(chn=CAM_CHN_ID_0)
        Display.show_image(frame)
        Display.show_image(
            self.osd_img,
            0,
            0,
            Display.LAYER_OSD3,
        )


class ShapeFeatureApp(AIBase):
    def __init__(self):
        super().__init__(
            MODEL_PATH,
            MODEL_INPUT_SIZE,
            RGB888P_SIZE,
            0,
        )

        self.model_input_size = MODEL_INPUT_SIZE
        self.rgb888p_size = [
            ALIGN_UP(RGB888P_SIZE[0], 16),
            RGB888P_SIZE[1],
        ]
        self.crop_width = CROP_WIDTH
        self.crop_height = CROP_HEIGHT
        self.crop_x = (
            self.rgb888p_size[0] - self.crop_width
        ) // 2
        self.crop_y = (
            self.rgb888p_size[1] - self.crop_height
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
            self.crop_width,
            self.crop_height,
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


def ensure_database_directory():
    try:
        os.mkdir("/sdcard/utils")
    except Exception:
        pass
    try:
        os.mkdir(DATABASE_PATH)
    except Exception:
        pass


def cosine_similarity(vector_a, vector_b):
    dot_product = sum(vector_a * vector_b)
    norm_a = np.sqrt(sum(vector_a * vector_a))
    norm_b = np.sqrt(sum(vector_b * vector_b))
    if norm_a == 0 or norm_b == 0:
        return 0.0
    return float(dot_product / (norm_a * norm_b))


def feature_counts():
    counts = {}
    for label in LABELS:
        counts[label] = 0

    try:
        file_names = os.listdir(DATABASE_PATH)
    except Exception:
        return counts

    for file_name in file_names:
        for label in LABELS:
            if file_name.startswith(label + "_") and \
                    file_name.endswith(".bin"):
                counts[label] += 1
                break
    return counts


def database_is_complete(counts):
    for label in LABELS:
        if counts[label] < FEATURES_PER_LABEL:
            return False
    return True


def next_incomplete_label(counts):
    for label in LABELS:
        if counts[label] < FEATURES_PER_LABEL:
            return label
    return None


def load_feature_database():
    entries = []
    try:
        file_names = os.listdir(DATABASE_PATH)
    except Exception:
        return entries

    for file_name in file_names:
        label = None
        for candidate in LABELS:
            if file_name.startswith(candidate + "_") and \
                    file_name.endswith(".bin"):
                label = candidate
                break
        if label is None:
            continue

        try:
            with open(DATABASE_PATH + file_name, "rb") as feature_file:
                data = feature_file.read()
            vector = np.frombuffer(data, dtype=np.float)
            entries.append((label, vector, file_name))
        except Exception as error:
            print("Skip bad feature file:", file_name, error)

    return entries


def recognize_feature(feature, database):
    best_label = "U"
    best_score = 0.0

    for label, saved_feature, _ in database:
        score = cosine_similarity(feature, saved_feature)
        if score > best_score:
            best_label = label
            best_score = score

    if best_score <= SIMILARITY_THRESHOLD:
        return "U", best_score
    return best_label, best_score


def draw_center_box(pipeline, status, color=(255, 255, 0, 255)):
    pipeline.osd_img.clear()

    box_x = int(
        ((RGB888P_SIZE[0] - CROP_WIDTH) // 2)
        * DISPLAY_SIZE[0]
        / RGB888P_SIZE[0]
    )
    box_y = int(
        ((RGB888P_SIZE[1] - CROP_HEIGHT) // 2)
        * DISPLAY_SIZE[1]
        / RGB888P_SIZE[1]
    )
    box_width = int(CROP_WIDTH * DISPLAY_SIZE[0] / RGB888P_SIZE[0])
    box_height = int(CROP_HEIGHT * DISPLAY_SIZE[1] / RGB888P_SIZE[1])

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
        status,
        color=color,
    )


def main():
    pipeline = None
    shape_app = None

    try:
        ensure_database_directory()

        pipeline = ShapePreviewPipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode="lcd",
        )
        pipeline.create()

        shape_app = ShapeFeatureApp()
        shape_app.config_preprocess()

        counts = feature_counts()
        database = []
        collect_frame_count = 0
        last_reported_label = None
        stable_label = None
        stable_count = 0

        if database_is_complete(counts):
            database = load_feature_database()
            print("Shape feature database ready:", len(database), "files")
        else:
            print("Shape feature database is incomplete:", counts)
            print("The script will collect 5 features for each label")

        while True:
            os.exitpoint()
            frame = pipeline.get_frame()
            feature = shape_app.run(frame)

            if not database_is_complete(counts):
                label = next_incomplete_label(counts)
                collect_frame_count += 1
                status = "LEARN {} {}/{}".format(
                    label,
                    counts[label],
                    FEATURES_PER_LABEL,
                )
                draw_center_box(pipeline, status)

                if collect_frame_count >= COLLECT_INTERVAL_FRAMES:
                    file_name = "{}_{}.bin".format(
                        label,
                        counts[label],
                    )
                    with open(
                        DATABASE_PATH + file_name,
                        "wb",
                    ) as feature_file:
                        feature_file.write(feature.tobytes())

                    counts[label] += 1
                    collect_frame_count = 0
                    print(
                        "COLLECTED {} {}/{}".format(
                            label,
                            counts[label],
                            FEATURES_PER_LABEL,
                        )
                    )

                    if database_is_complete(counts):
                        database = load_feature_database()
                        print(
                            "LEARNING DONE; recognition started with",
                            len(database),
                            "features",
                        )

            else:
                label, score = recognize_feature(feature, database)

                if label == stable_label:
                    stable_count += 1
                else:
                    stable_label = label
                    stable_count = 1

                status = "SHAPE: {} {}%".format(
                    label,
                    int(score * 100 + 0.5),
                )
                if label == "U":
                    color = (255, 255, 0, 0)
                else:
                    color = (255, 0, 255, 0)
                draw_center_box(pipeline, status, color=color)

                if stable_count >= 3 and label != last_reported_label:
                    print(
                        "SHAPE = {}  SCORE = {:.3f}".format(
                            label,
                            score,
                        )
                    )
                    last_reported_label = label

            pipeline.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("Wafer shape preview stopped by user")

    except Exception as error:
        if "IDE interrupt" in str(error):
            print("Wafer shape preview stopped by CanMV IDE")
        else:
            print("Wafer shape preview error:", error)
            raise

    finally:
        if shape_app is not None:
            shape_app.deinit()
        if pipeline is not None:
            pipeline.destroy()


if __name__ == "__main__":
    main()

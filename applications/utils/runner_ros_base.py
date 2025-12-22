# runner_ros_base.py

import logging
import time
from collections import deque

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from utils.time_utils import timing_context
from utils.types import DataInput


class RunnerROSBase:
    """
    Base class for ROS1 and ROS2 runners.
    Handles shared logic such as intrinsics/extrinsics loading,
    image decompression, pose conversion, and keyframe processing.
    """

    def __init__(self, cfg, dualmap):       # __init__ 构造函数
        self.cfg = cfg
        self.dualmap = dualmap
        self.logger = logging.getLogger(__name__)   #  __name__ 获取当前模块的名称

        self.kf_idx = 0
        self.intrinsics = None
        self.extrinsics = None
        self.synced_data_queue = deque(maxlen=1)    # 用于存储同步后的数据输入的队列，最大长度为1
        self.shutdown_requested = False
        self.last_message_time = None

    def load_intrinsics(self, dataset_cfg):
        """Load camera intrinsics from config file."""
        intrinsic_cfg = dataset_cfg.get("intrinsic", None)
        if intrinsic_cfg:
            fx, fy, cx, cy = (
                intrinsic_cfg["fx"],
                intrinsic_cfg["fy"],
                intrinsic_cfg["cx"],
                intrinsic_cfg["cy"],
            )
            self.logger.warning("[Main] Loaded intrinsics from config.")
            return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.logger.warning("[Main] No intrinsics provided.")
        return None

    def load_extrinsics(self, dataset_cfg):
        """Load camera extrinsics from config file."""
        extrinsic_cfg = dataset_cfg.get("extrinsics", None)
        if extrinsic_cfg:
            matrix = np.array(extrinsic_cfg)
            if matrix.shape == (4, 4):
                self.logger.warning("[Main] Loaded extrinsics from config.")
                return matrix
        self.logger.warning(
            "[Main] No valid extrinsics provided. Using identity matrix."
        )
        return np.eye(4)

    def create_world_transform(self):       # 创建世界坐标系变换矩阵    # 疑问点1
        """Create world coordinate transformation from roll/pitch/yaw."""
        roll = np.radians(self.cfg.world_roll)
        pitch = np.radians(self.cfg.world_pitch)
        yaw = np.radians(self.cfg.world_yaw)

        Rx = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )
        Ry = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )
        Rz = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        )

        R_combined = Rz @ Ry @ Rx
        T = np.eye(4)
        T[:3, :3] = R_combined
        return T

    def decompress_image(self, msg_data, is_depth=False):
        """Decode compressed image data (RGB or depth)."""
        msg_data = bytes(msg_data)
        if is_depth:
            depth_data = np.frombuffer(msg_data[12:], np.uint8)
            img = cv2.imdecode(depth_data, cv2.IMREAD_UNCHANGED)
        else:
            np_arr = np.frombuffer(msg_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def build_pose_matrix(self, translation, quaternion):
        """Construct 4x4 pose matrix from translation and quaternion."""
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation
        return transformation_matrix

    def push_data(self, rgb_img, depth_img, pose, timestamp):
        """Push synchronized input data into queue for processing."""
        transformed_pose = self.create_world_transform() @ (pose @ self.extrinsics)             # @ 代表矩阵相乘

        data_input = DataInput(
            idx=self.kf_idx,
            time_stamp=timestamp,
            color=rgb_img,
            depth=depth_img,
            color_name=str(timestamp),
            intrinsics=self.intrinsics,
            pose=transformed_pose,
        )
        self.synced_data_queue.append(data_input)
        return data_input

    # 运行一次处理逻辑,注意；dualMap核心处理函数包括： sequential_process、parallel_process、check_keyframe；另外还有部分辅助函数包括：get_keyframe_idx、end_process

    def run_once(self, current_time_fn):
        """Check and process a keyframe if data is ready."""
        if not self.synced_data_queue:      # 如果同步数据队列为空，则直接返回
            return

        data_input = self.synced_data_queue[-1]     # 获取队列中的最新数据输入

        if not self.dualmap.calculate_path:         # 如果不计算路径，则进行以下检查
            current_time = current_time_fn()        # 获取当前时间
            last_time = self.last_message_time      # 获取上次接收消息的时间
            if self.cfg.use_end_process and last_time is not None:      # 如果配置了结束处理且上次消息时间不为空
                if current_time - last_time > 20.0:     # 如果当前时间与上次消息时间的差值大于20秒
                    self.logger.warning(        
                        "[Main] No new data received. Entering end process."        # 进入结束处理
                    )
                    self.dualmap.end_process()      # 调用 dualmap 的结束处理方法
                    self.shutdown_requested = True  # 请求关闭
                    return

        if not self.dualmap.check_keyframe(data_input.time_stamp, data_input.pose):   # 检查当前数据输入是否为关键帧
            return

        data_input.idx = self.dualmap.get_keyframe_idx()    # 更新数据输入的索引为当前关键帧索引

        self.logger.info(
            "[Main] ============================================================"
        )
        with timing_context("Time Per Frame", self.dualmap):    # 计时上下文管理器，用于测量处理时间
            if self.cfg.use_parallel:                 # 如果配置了并行处理
                self.dualmap.parallel_process(data_input)   # 使用并行处理方法处理数据输入
            else:
                self.dualmap.sequential_process(data_input)   # 否则使用顺序处理方法处理数据输入

        self.logger.info(
            f"[Main] Processing keyframe {data_input.idx} took {time.time() - data_input.time_stamp:.2f} seconds."
        )

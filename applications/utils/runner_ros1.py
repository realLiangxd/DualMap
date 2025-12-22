# runner_ros1.py

import logging
import threading
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from omegaconf import OmegaConf     # OmegaConf 用于处理配置文件, 支持层次化结构, 类型转换等功能
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from applications.utils.runner_ros_base import RunnerROSBase
from dualmap.core import Dualmap
from utils.logging_helper import setup_logging


class RunnerROS1(RunnerROSBase):
    """
    ROS1-specific runner, handles topic subscriptions and data flow using rospy.
    """

    def __init__(self, cfg):
        rospy.init_node("runner_ros", anonymous=True)   # 初始化 ROS 节点，节点名称为 "runner_ros"
        setup_logging(output_path=cfg.output_path, config_path=cfg.logging_config)  # 设置日志记录
        self.logger = logging.getLogger(__name__)   #  __name__ 获取当前模块的名称
        self.logger.info("[Runner ROS1]")
        self.logger.info(OmegaConf.to_yaml(cfg))       # 打印配置文件内容

        self.cfg = cfg
        self.dualmap = Dualmap(cfg)     # 初始化 Dualmap 实例
        super().__init__(cfg, self.dualmap)     # Call base class constructor

        self.bridge = CvBridge()
        self.dataset_cfg = OmegaConf.load(cfg.ros_stream_config_path)   # 加载数据集配置文件
        self.intrinsics = self.load_intrinsics(self.dataset_cfg)        # 加载相机内参
        self.extrinsics = self.load_extrinsics(self.dataset_cfg)        # camera to lidar 外参

        # Image and Odometry Subscribers
        if self.cfg.use_compressed_topic:
            self.logger.warning("[Main] Using compressed topics.")
            self.rgb_sub = Subscriber(self.dataset_cfg.ros_topics.rgb, CompressedImage) # 订阅压缩的 RGB 图像话题
            self.depth_sub = Subscriber(
                self.dataset_cfg.ros_topics.depth, CompressedImage      # 订阅压缩的深度图像话题
            )
        else:
            self.logger.warning("[Main] Using uncompressed topics.")
            self.rgb_sub = Subscriber(self.dataset_cfg.ros_topics.rgb, Image)   # 订阅未压缩的 RGB 图像话题
            self.depth_sub = Subscriber(self.dataset_cfg.ros_topics.depth, Image)   # 订阅未压缩的深度图像话题

        self.odom_sub = Subscriber(self.dataset_cfg.ros_topics.odom, Odometry)  # 订阅里程计话题

        # Sync RGB + Depth + Odometry
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.odom_sub],
            queue_size=10,
            slop=self.cfg.sync_threshold,
        )
        self.sync.registerCallback(self.synced_callback)

        # Fallback to camera_info topic if intrinsics not loaded
        rospy.Subscriber(
            self.dataset_cfg.ros_topics.camera_info,
            CameraInfo,
            self.camera_info_callback,
        )

    def synced_callback(self, rgb_msg, depth_msg, odom_msg):        # 同步回调函数
        """Callback for synchronized RGB, Depth, and Odom messages."""
        timestamp = rgb_msg.header.stamp.to_sec()

        if self.cfg.use_compressed_topic:
            rgb_img = self.decompress_image(rgb_msg.data, is_depth=False)       # 解压缩 RGB 图像
            depth_img = self.decompress_image(depth_msg.data, is_depth=True)   # 解压缩深度图像
        else:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")

        depth_factor = getattr(self.dataset_cfg, 'depth_factor', 1000.0)    # 深度图像缩放因子，默认值为1000.0
        depth_img = depth_img.astype(np.float32) / depth_factor      # 将深度图像转换为浮点数并进行缩放
        depth_img = np.expand_dims(depth_img, axis=-1)                     # 扩展深度图像维度以匹配预期格式

        translation = np.array(
            [
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                odom_msg.pose.pose.position.z,
            ]
        )
        quaternion = np.array(
            [
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w,
            ]
        )

        pose_matrix = self.build_pose_matrix(translation, quaternion)
        self.push_data(rgb_img, depth_img, pose_matrix, timestamp)
        self.last_message_time = time.time()

    def camera_info_callback(self, msg):
        """Fallback callback to get intrinsics from CameraInfo if needed."""
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.K).reshape(3, 3)
            self.logger.warning("[Main] Camera intrinsics received and stored.")

    def spin(self):
        """Main loop calling run_once() at configured ROS rate."""
        rate = rospy.Rate(self.cfg.ros_rate)
        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                self.run_once(lambda: time.time())      # 使用系统时间作为当前时间  lambda: time.time() 是一个匿名函数，调用时返回当前时间
            except Exception as e:
                self.logger.error(f"[RunnerROS1] Exception: {e}", exc_info=True)
            rate.sleep()


def run_ros1(cfg):
    """Launch the ROS1 runner in a background thread."""
    runner = RunnerROS1(cfg)
    runner.logger.warning("[Main] ROS1 Runner started. Waiting for data stream...")

    spin_thread = threading.Thread(target=runner.spin)
    spin_thread.start()

    try:
        while not rospy.is_shutdown() and not runner.shutdown_requested:
            time.sleep(0.1)
    except KeyboardInterrupt:
        runner.logger.warning("[Main] KeyboardInterrupt received.")
    finally:
        runner.shutdown_requested = True
        runner.logger.warning("[Main] Shutting down...")
        spin_thread.join(timeout=3.0)

        try:
            rospy.signal_shutdown("User requested shutdown")
        except Exception:
            pass

        runner.logger.warning("[Main] Exit complete.")

        import os

        os._exit(0)

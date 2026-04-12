#!/usr/bin/env python3
"""ROS2 node that subscribes to a compressed camera image, runs
Depth Anything V2 (Small) inference, and publishes a relative inverse
depth map as a 32FC1 Image.

The node optionally rectifies the input image using the camera intrinsics
from the ``camera_info`` topic before running depth estimation so that all
downstream consumers (backprojection, VLM pixel coordinates) can work with
a pinhole camera model.
"""

import os
import threading
import time

import cv2
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from transformers import pipeline


class DepthEstimatorNode(Node):
    """Depth Anything V2 inference node."""

    def __init__(self) -> None:
        super().__init__('depth_estimator')

        # -- parameters --------------------------------------------------
        self.declare_parameter('input_topic',
                               '/ba_overview_camera/image_raw/compressed')
        self.declare_parameter('camera_info_topic',
                               '/ba_overview_camera/camera_info')
        self.declare_parameter('output_topic',
                               '/ba_overview_camera/depth/image_raw')
        self.declare_parameter('model_id',
                               'depth-anything/Depth-Anything-V2-Small-hf')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('rectify', True)
        self.declare_parameter('debug_save_path', '')

        input_topic = self.get_parameter('input_topic') \
            .get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic') \
            .get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic') \
            .get_parameter_value().string_value
        model_id = self.get_parameter('model_id') \
            .get_parameter_value().string_value
        device = self.get_parameter('device') \
            .get_parameter_value().string_value
        self._rectify = self.get_parameter('rectify') \
            .get_parameter_value().bool_value
        self._debug_save_path = self.get_parameter('debug_save_path') \
            .get_parameter_value().string_value
        self._frame_count = 0

        # -- model --------------------------------------------------------
        self.get_logger().info(
            f'Loading model {model_id} on {device} ...')
        t0 = time.perf_counter()
        self._pipe = pipeline(
            task='depth-estimation', model=model_id, device=device)
        self.get_logger().info(
            f'Model loaded in {time.perf_counter() - t0:.2f} s')

        # -- rectification state ------------------------------------------
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None
        self._camera_info_sub = None  # set below if rectify=True

        # -- busy flag (drop frames that arrive during inference) ----------
        self._busy = False
        self._busy_lock = threading.Lock()

        # -- publishers ---------------------------------------------------
        # BEST_EFFORT QoS: CycloneDDS on WSL2 loopback cannot reliably
        # transport large (~1.2 MB) RELIABLE messages — the publisher
        # blocks waiting for ACKs that never arrive.  BEST_EFFORT avoids
        # the freeze.  Downstream in-process consumers are unaffected;
        # external subscribers (rviz2, topic hz) need --qos-reliability
        # best_effort.
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._depth_pub = self.create_publisher(
            Image, output_topic, depth_qos)

        # -- subscribers --------------------------------------------------
        # Keep only the latest image — we process on-demand, not a backlog.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_sub = self.create_subscription(
            CompressedImage, input_topic, self._image_cb, image_qos)

        if self._rectify:
            # v4l2_camera publishes camera_info with default QoS
            # (RELIABLE + VOLATILE), not TRANSIENT_LOCAL.
            info_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._camera_info_sub = self.create_subscription(
                CameraInfo, camera_info_topic, self._camera_info_cb, info_qos)
            self.get_logger().info(
                f'Waiting for camera_info on {camera_info_topic} ...')

        self.get_logger().info(
            f'Subscribing to {input_topic}, '
            f'publishing depth on {output_topic}')

    # -----------------------------------------------------------------
    # camera_info → build rectification LUT once
    # -----------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if self._map_x is not None:
            return  # already computed

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        w, h = msg.width, msg.height

        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            K, D, None, K, (w, h), cv2.CV_32FC1)

        self.get_logger().info(
            f'Rectification LUT built for {w}x{h} '
            f'(D={D.tolist()[:3]}...)')

        # No longer need this subscription.
        if self._camera_info_sub is not None:
            self.destroy_subscription(self._camera_info_sub)
            self._camera_info_sub = None

    # -----------------------------------------------------------------
    # image callback → decode, (rectify), infer, publish
    # -----------------------------------------------------------------
    def _image_cb(self, msg: CompressedImage) -> None:
        # Drop frame if we are still processing the previous one.
        with self._busy_lock:
            if self._busy:
                return
            self._busy = True

        try:
            self._process_frame(msg)
        finally:
            with self._busy_lock:
                self._busy = False

    def _process_frame(self, msg: CompressedImage) -> None:
        t0 = time.perf_counter()

        # 1. Decode JPEG ------------------------------------------------
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().error('Failed to decode compressed image')
            return

        # 2. Rectify -----------------------------------------------------
        if self._rectify and self._map_x is not None:
            bgr = cv2.remap(
                bgr, self._map_x, self._map_y, cv2.INTER_LINEAR)

        # 3. Inference ---------------------------------------------------
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)
        result = self._pipe(pil_img)
        depth_pil = result['depth']  # PIL Image, uint8 grayscale

        # 4. Normalise to float32 [0.0, 1.0] ----------------------------
        depth_f32 = np.ascontiguousarray(
            np.array(depth_pil, dtype=np.float32) / 255.0)

        # 5. Publish -----------------------------------------------------
        depth_msg = Image()
        depth_msg.header.stamp = msg.header.stamp
        depth_msg.header.frame_id = msg.header.frame_id
        depth_msg.height, depth_msg.width = depth_f32.shape[:2]
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = depth_msg.width * 4  # float32 = 4 bytes
        depth_msg.data = bytes(depth_f32.tobytes())
        self._depth_pub.publish(depth_msg)

        # 6. Debug: save depth PNG to disk periodically ------------------
        self._frame_count += 1
        if self._debug_save_path:
            depth_uint8 = (depth_f32 * 255.0).astype(np.uint8)
            path = os.path.join(
                self._debug_save_path, 'depth_latest.png')
            cv2.imwrite(path, depth_uint8)
            if self._frame_count == 1:
                self.get_logger().info(
                    f'Debug: saving depth PNGs to {path}')

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.get_logger().info(
            f'Depth published #{self._frame_count} '
            f'({bgr.shape[1]}x{bgr.shape[0]}, {dt_ms:.0f} ms)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

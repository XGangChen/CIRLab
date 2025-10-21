#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (registers conversions)
from geometry_msgs.msg import PointStamped

import mediapipe as mp

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)

class WristTracker(Node):
    def __init__(self):
        super().__init__('wrist_tracker')
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.have_info = False

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        # Topics (adjust if your names differ)
        color_topic = '/cam2/cam2/color/image_raw'
        depth_topic = '/cam2/cam2/aligned_depth_to_color/image_raw'
        info_topic  = '/cam2/cam2/color/camera_info'

        self.color_sub = self.create_subscription(Image, color_topic, self.on_color, sensor_qos)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.on_depth, sensor_qos)
        self.info_sub  = self.create_subscription(CameraInfo, info_topic,  self.on_info,  sensor_qos) 

        self.latest_depth = None

        # Publishers
        self.pub_point = self.create_publisher(PointStamped, '/wrist_point_base', 10)
        self.pub_marker = self.create_publisher(Marker, '/wrist_marker', 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'platform_base'
        self.source_frame = 'cam2_color_optical_frame'

    def on_info(self, msg: CameraInfo):
        self.cam_model.fromCameraInfo(msg)
        self.have_info = True

    def on_depth(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth.dtype == np.uint16:
            self.latest_depth = depth.astype(np.float32) / 1000.0  # mm → m
        elif depth.dtype == np.float32:
            self.latest_depth = depth
        else:
            self.get_logger().warn(f'Unexpected depth dtype: {depth.dtype}')
            self.latest_depth = None

    def on_color(self, msg: Image):
        if not self.have_info or self.latest_depth is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h_img, w_img = frame.shape[:2]

        # Run MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return

        # Wrist landmark index 0
        lm = res.multi_hand_landmarks[0].landmark[0]
        u = int(np.clip(lm.x * w_img, 0, w_img - 1))
        v = int(np.clip(lm.y * h_img, 0, h_img - 1))

        # Robust depth: median in 7x7 window (ignore zeros)
        d_patch = self.latest_depth[max(v-3,0):min(v+4,h_img),
                                    max(u-3,0):min(u+4,w_img)].astype(np.float32)
        d_vals = d_patch[d_patch > 0]
        if d_vals.size == 0:
            return
        depth_m = float(np.median(d_vals))      # in meters

        self.get_logger().info(f"wrist px=({u},{v}) depth={depth_m:.3f} m")

        # Deproject pixel (u,v,depth) to 3D in camera optical frame
        # Using pinhole model: (x - cx)/fx * z, (y - cy)/fy * z
        fx = self.cam_model.fx(); fy = self.cam_model.fy()
        cx = self.cam_model.cx(); cy = self.cam_model.cy()
        Xc = (u - cx) * depth_m / fx
        Yc = (v - cy) * depth_m / fy
        Zc = depth_m

        pt_cam = PointStamped()
        pt_cam.header.stamp = msg.header.stamp
        pt_cam.header.frame_id = self.source_frame
        pt_cam.point.x = Xc
        pt_cam.point.y = Yc
        pt_cam.point.z = Zc

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
            pt_base = tf2_ros.do_transform_point(pt_cam, tf)
        except Exception as e:
            self.get_logger().warn(f'No TF: {e}')
            return

        # Publish PointStamped in base
        self.pub_point.publish(pt_base)

        # RViz marker
        m = Marker()
        m.header = pt_base.header
        m.ns = 'wrist'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pt_base.point.x
        m.pose.position.y = pt_base.point.y
        m.pose.position.z = pt_base.point.z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.02  # 2 cm sphere
        m.color.a = 1.0
        m.color.r = 1.0
        self.pub_marker.publish(m)

def main():
    rclpy.init()
    node = WristTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

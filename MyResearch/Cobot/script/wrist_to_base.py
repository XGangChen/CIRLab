#!/usr/bin/env python3
# wrist_to_base.py
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import mediapipe as mp
import tf2_ros
import tf2_geometry_msgs as tf2gm   # for do_transform_point()

def robust_depth_at(u, v, depth, h, w):
    """Return (median_depth_m, k_used) using adaptive windows; NaN if no valid pixels."""
    for k in (7, 11, 15):
        r = k // 2
        v0, v1 = max(v-r, 0), min(v+r+1, h)
        u0, u1 = max(u-r, 0), min(u+r+1, w)
        patch = depth[v0:v1, u0:u1]
        vals = patch[patch > 0]
        if vals.size:
            return float(np.median(vals)), k
    return float('nan'), 0

class WristToBase(Node):
    def __init__(self):
        super().__init__('wrist_to_base')

        # ---------- parameters (override with --ros-args -p key:=value) ----------
        self.declare_parameter('color_topic',  '/camera/cam2/color/image_raw')
        self.declare_parameter('depth_topic',  '/camera/cam2/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',   '/camera/cam2/color/camera_info')
        self.declare_parameter('source_frame', 'cam2_color_optical_frame')
        self.declare_parameter('target_frame', 'platform_base')
        self.declare_parameter('min_det_conf', 0.30)
        self.declare_parameter('min_trk_conf', 0.30)

        color_topic  = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic  = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic   = self.get_parameter('info_topic').get_parameter_value().string_value
        self.src     = self.get_parameter('source_frame').get_parameter_value().string_value
        self.tgt     = self.get_parameter('target_frame').get_parameter_value().string_value
        det_c        = float(self.get_parameter('min_det_conf').get_parameter_value().double_value)
        trk_c        = float(self.get_parameter('min_trk_conf').get_parameter_value().double_value)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value  # matches RealSense QoS

        # ---------- state ----------
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.have_info = False
        self.latest_depth = None
        self.depth_frame_id = ""
        self._align_warned = False
        self._tf_warned    = False

        # ---------- subscribers / publishers ----------
        self.create_subscription(Image, color_topic, self.on_color, sensor_qos)
        self.create_subscription(Image, depth_topic, self.on_depth, sensor_qos)
        self.create_subscription(CameraInfo, info_topic, self.on_info, sensor_qos)

        self.pub_point_left  = self.create_publisher(PointStamped, '/wrist_left_point_base', 10)
        self.pub_point_right = self.create_publisher(PointStamped, '/wrist_right_point_base', 10)
        self.pub_marker      = self.create_publisher(Marker, '/wrist_marker', 10)
        self.pub_marker_arr  = self.create_publisher(MarkerArray, '/wrist_marker_array', 10)
        self.pub_dbg         = self.create_publisher(Image, '/wrist/debug_image', 10)

        # ---------- TF ----------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- MediaPipe (2 hands) ----------
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2,
            min_detection_confidence=det_c, min_tracking_confidence=trk_c
        )
        self.drawer = mp.solutions.drawing_utils
        self.styles = mp.solutions.drawing_styles

        self.get_logger().info(f"Subscribing: {color_topic}, {depth_topic}, {info_topic}")
        self.get_logger().info(f"Frames: {self.src} -> {self.tgt}")

    # ------------------- callbacks -------------------
    def on_info(self, msg: CameraInfo):
        self.cam_model.fromCameraInfo(msg)
        self.have_info = True
        self.get_logger().info(f"CameraInfo OK (frame_id={msg.header.frame_id})")

    def on_depth(self, msg: Image):
        d = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if d.dtype == np.uint16:
            self.latest_depth = d.astype(np.float32) / 1000.0  # mm -> m
        elif d.dtype == np.float32:
            self.latest_depth = d
        else:
            self.latest_depth = None
            self.get_logger().warn(f"Unexpected depth dtype: {d.dtype}")
        self.depth_frame_id = msg.header.frame_id
        if (self.depth_frame_id != self.src) and not self._align_warned:
            self.get_logger().warn(
                f"Depth frame_id={self.depth_frame_id} != {self.src}. "
                "Enable align_depth.enable:=true or change 'source_frame'."
            )
            self._align_warned = True

    def on_color(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = bgr.shape[:2]

        # If not ready, still publish a frame so rqt updates
        if not self.have_info or self.latest_depth is None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))
            return

        # MediaPipe
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        hands = res.multi_hand_landmarks or []
        labels = [c.classification[0].label for c in (res.multi_handedness or [])]  # 'Left'/'Right'

        markers = []
        for i, hand in enumerate(hands):
            # Draw skeleton
            self.drawer.draw_landmarks(
                bgr, hand, mp.solutions.hands.HAND_CONNECTIONS,
                self.styles.get_default_hand_landmarks_style(),
                self.styles.get_default_hand_connections_style()
            )

            # Wrist pixel
            wrist = hand.landmark[0]
            u = int(np.clip(wrist.x * w, 0, w - 1))
            v = int(np.clip(wrist.y * h, 0, h - 1))

            # Depth (adaptive median), meters
            z, k_used = robust_depth_at(u, v, self.latest_depth, h, w)

            fx, fy = self.cam_model.fx(), self.cam_model.fy()
            cx, cy = self.cam_model.cx(), self.cam_model.cy()
            if np.isfinite(z):
                Xc = (u - cx) * z / fx
                Yc = (v - cy) * z / fy
                Zc = z
                R  = math.sqrt(Xc*Xc + Yc*Yc + Zc*Zc)
            else:
                Xc = Yc = Zc = R = float('nan')

            # Annotate debug image
            label = labels[i] if i < len(labels) else f"H{i}"
            color = (0, 0, 255) if label == 'Left' else (0, 255, 0)
            cv2.circle(bgr, (u, v), 8, color, -1)
            cv2.putText(
                bgr,
                f"{label} z={Zc:.3f}m )",       # R={R:.3f}m (k={k_used} (true range)
                (u + 8, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )

            # Publish Point + Marker if depth is valid
            if np.isfinite(Zc):
                pt_cam = PointStamped()
                pt_cam.header.stamp = msg.header.stamp
                pt_cam.header.frame_id = self.src
                pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = Xc, Yc, Zc

                try:
                    tf = self.tf_buffer.lookup_transform(self.tgt, self.src, rclpy.time.Time())
                    pt_base = tf2gm.do_transform_point(pt_cam, tf)
                except Exception as e:
                    if not self._tf_warned:
                        self.get_logger().warn(f'No TF {self.tgt}<-{self.src}: {e}')
                        self._tf_warned = True
                    continue

                # publish left/right points
                if label == 'Left':
                    self.pub_point_left.publish(pt_base); mid = 0; col = (1.0, 0.2, 0.2)
                else:
                    self.pub_point_right.publish(pt_base); mid = 1; col = (0.2, 1.0, 0.2)

                # marker (big + persistent)
                m = Marker()
                m.header = pt_base.header                  # frame_id: platform_base
                m.ns = 'wrist'
                m.id = mid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = pt_base.point.x
                m.pose.position.y = pt_base.point.y
                m.pose.position.z = pt_base.point.z
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.12   # easy to see
                m.lifetime = Duration(sec=0, nanosec=0)    # persistent
                m.color.a = 1.0
                m.color.r, m.color.g, m.color.b = col
                markers.append(m)

                self.get_logger().info(
                    f"{label}: px=({u},{v}) z={Zc:.3f}m XYZc=({Xc:.3f},{Yc:.3f},{Zc:.3f}) "
                    f"→ base=({pt_base.point.x:.3f},{pt_base.point.y:.3f},{pt_base.point.z:.3f})",
                    throttle_duration_sec=0.3
                )
            else:
                self.get_logger().info("No valid depth at wrist; skipping publish.", throttle_duration_sec=0.5)

        # publish markers (both single and array for RViz)
        for m in markers:
            self.pub_marker.publish(m)
        if markers:
            self.pub_marker_arr.publish(MarkerArray(markers=markers))

        # publish debug image every frame
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))

def main():
    rclpy.init()
    node = WristToBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

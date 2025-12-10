#!/usr/bin/env python3
# wrist_to_base.py

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
# This imports the Message type directly (has 'sec', 'nanosec')
from builtin_interfaces.msg import Duration 
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import mediapipe as mp
import tf2_ros
import tf2_geometry_msgs as tf2gm

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

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

        # ---------- parameters ----------
        self.declare_parameter('color_topic',  '/camera/cam2/color/image_raw')
        self.declare_parameter('depth_topic',  '/camera/cam2/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',   '/camera/cam2/color/camera_info')
        self.declare_parameter('source_frame', 'cam2_color_optical_frame')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_det_conf', 0.30)
        self.declare_parameter('min_trk_conf', 0.30)

        color_topic  = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic  = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic   = self.get_parameter('info_topic').get_parameter_value().string_value
        self.src     = self.get_parameter('source_frame').get_parameter_value().string_value
        self.tgt     = self.get_parameter('target_frame').get_parameter_value().string_value
        det_c        = float(self.get_parameter('min_det_conf').get_parameter_value().double_value)
        trk_c        = float(self.get_parameter('min_trk_conf').get_parameter_value().double_value)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

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

        self.pub_point_left  = self.create_publisher(PointStamped, '/wrist_right_point_base', 10)
        self.pub_point_right = self.create_publisher(PointStamped, '/wrist_left_point_base', 10)
        self.pub_marker      = self.create_publisher(Marker, '/wrist_marker', 10)
        self.pub_marker_arr  = self.create_publisher(MarkerArray, '/wrist_marker_array', 10)
        self.pub_co          = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.pub_dbg         = self.create_publisher(Image, '/wrist/debug_image', 10)

        # ---------- TF ----------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- MediaPipe ----------
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2,
            min_detection_confidence=det_c, min_tracking_confidence=trk_c
        )
        self.drawer = mp.solutions.drawing_utils
        self.styles = mp.solutions.drawing_styles

        self.get_logger().info(f"Subscribing: {color_topic}, {depth_topic}")
        self.get_logger().info(f"Publishing CollisionObjects to: /collision_object")

    def on_info(self, msg: CameraInfo):
        self.cam_model.fromCameraInfo(msg)
        self.have_info = True

    def on_depth(self, msg: Image):
        d = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if d.dtype == np.uint16:
            self.latest_depth = d.astype(np.float32) / 1000.0
        elif d.dtype == np.float32:
            self.latest_depth = d
        else:
            return
        self.depth_frame_id = msg.header.frame_id

    def on_color(self, msg: Image):
        if not self.have_info or self.latest_depth is None:
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = bgr.shape[:2]

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        hands = res.multi_hand_landmarks or []
        multi_handedness = res.multi_handedness or []
        
        markers = []

        for i, hand in enumerate(hands):
            label = "Unknown"
            if i < len(multi_handedness):
                label = multi_handedness[i].classification[0].label 
            
            self.drawer.draw_landmarks(
                bgr, hand, mp.solutions.hands.HAND_CONNECTIONS,
                self.styles.get_default_hand_landmarks_style(),
                self.styles.get_default_hand_connections_style()
            )

            wrist = hand.landmark[0]
            u = int(np.clip(wrist.x * w, 0, w - 1))
            v = int(np.clip(wrist.y * h, 0, h - 1))

            z, k_used = robust_depth_at(u, v, self.latest_depth, h, w)

            if np.isfinite(z):
                fx, fy = self.cam_model.fx(), self.cam_model.fy()
                cx, cy = self.cam_model.cx(), self.cam_model.cy()
                Xc = (u - cx) * z / fx
                Yc = (v - cy) * z / fy
                Zc = z

                pt_cam = PointStamped()
                pt_cam.header.stamp = msg.header.stamp
                pt_cam.header.frame_id = self.src
                pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = Xc, Yc, Zc

                try:
                    tf = self.tf_buffer.lookup_transform(self.tgt, self.src, rclpy.time.Time())
                    pt_base = tf2gm.do_transform_point(pt_cam, tf)
                except Exception as e:
                    if not self._tf_warned:
                        self.get_logger().warn(f'TF Error {self.tgt}<-{self.src}: {e}')
                        self._tf_warned = True
                    continue

                if label == 'Left':
                    self.pub_point_left.publish(pt_base)
                    col = (1.0, 0.2, 0.2)
                    mid = 0
                    co_id = "wrist_left"
                else:
                    self.pub_point_right.publish(pt_base)
                    col = (0.2, 1.0, 0.2)
                    mid = 1
                    co_id = "wrist_right"

                # VISUAL MARKER
                m = Marker()
                m.header = pt_base.header
                m.ns, m.id = 'wrist', mid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position = pt_base.point
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.10
                m.color.r, m.color.g, m.color.b = col
                m.color.a = 0.8
                
                # FIX IS HERE:
                # We imported 'Duration' from builtin_interfaces.msg. 
                # It is the Message class itself, so it has 'sec' and 'nanosec' fields 
                # and DOES NOT have a .to_msg() method.
                m.lifetime = Duration(sec=0, nanosec=500000000)
                
                markers.append(m)

                # COLLISION OBJECT
                co = CollisionObject()
                co.header = pt_base.header
                co.id = co_id
                sphere = SolidPrimitive()
                sphere.type = SolidPrimitive.SPHERE
                sphere.dimensions = [0.12]
                pose = Pose()
                pose.position = pt_base.point
                pose.orientation.w = 1.0
                co.primitives.append(sphere)
                co.primitive_poses.append(pose)
                co.operation = CollisionObject.ADD
                self.pub_co.publish(co)

                cv2.putText(bgr, f"{label} z={Zc:.2f}", (u+10, v), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        if markers:
            self.pub_marker_arr.publish(MarkerArray(markers=markers))
        
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))

def main():
    rclpy.init()
    node = WristToBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy, cv2, numpy as np, math
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import mediapipe as mp

class WristDebug(Node):
    def __init__(self):
        super().__init__('wrist_debug')

        # --- params (change if your topic names differ) ---
        self.declare_parameter('color_topic', '/camera/cam2/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/cam2/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',  '/camera/cam2/color/camera_info')

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic  = self.get_parameter('info_topic').get_parameter_value().string_value

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        # --- state ---
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.have_info = False
        self.latest_depth = None
        self.depth_frame_id = ""

        # --- MediaPipe (2 hands) ---
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2,
            min_detection_confidence=0.3, min_tracking_confidence=0.3
        )
        self.drawer = mp.solutions.drawing_utils
        self.styles = mp.solutions.drawing_styles

        # --- pubs/subs ---
        self.pub_dbg = self.create_publisher(Image, '/wrist/debug_image', 10)
        self.create_subscription(Image, color_topic, self.on_color, sensor_qos)
        self.create_subscription(Image, depth_topic, self.on_depth, sensor_qos)
        self.create_subscription(CameraInfo, info_topic, self.on_info, sensor_qos)

        self.get_logger().info(f"Listening to {color_topic} + {depth_topic}; publishing /wrist/debug_image")

    # Strip transport suffixes if user passes them accidentally
    def _base_image_topic(self, t):
        for suf in ('/theora', '/compressed', '/compressedDepth'):
            if t.endswith(suf):
                return t[:-len(suf)]
        return t

    def on_info(self, msg: CameraInfo):
        self.cam_model.fromCameraInfo(msg)
        self.have_info = True
        fx, fy, cx, cy = self.cam_model.fx(), self.cam_model.fy(), self.cam_model.cx(), self.cam_model.cy()
        self.get_logger().info(f"CameraInfo OK (frame_id={msg.header.frame_id}) fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    def on_depth(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if depth.dtype == np.uint16:
            self.latest_depth = depth.astype(np.float32) / 1000.0  # mm→m
        elif depth.dtype == np.float32:
            self.latest_depth = depth
        else:
            self.latest_depth = None
            self.get_logger().warn(f"Unexpected depth dtype: {depth.dtype}")
        self.depth_frame_id = msg.header.frame_id

    def on_color(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = bgr.shape[:2]

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)

        hands = res.multi_hand_landmarks or []
        labels = []
        if getattr(res, 'multi_handedness', None):
            labels = [hd.classification[0].label for hd in res.multi_handedness]  # 'Left'/'Right'

        for i, hand in enumerate(hands):
            self.drawer.draw_landmarks(
                bgr, hand, mp.solutions.hands.HAND_CONNECTIONS,
                self.styles.get_default_hand_landmarks_style(),
                self.styles.get_default_hand_connections_style()
            )
            wrist = hand.landmark[0]
            u = int(np.clip(wrist.x * w, 0, w - 1))
            v = int(np.clip(wrist.y * h, 0, h - 1))
            cv2.circle(bgr, (u, v), 6, (0, 0, 255), -1)

            # If depth or intrinsics not ready, skip numbers but still draw
            if (self.latest_depth is None) or (not self.have_info):
                label = labels[i] if i < len(labels) else f"H{i}"
                cv2.putText(bgr, f"{label} ({u},{v})", (u + 8, v - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                continue

            # Robust depth around the wrist pixel (7x7 median), units: meters
            v0, v1 = max(v-3, 0), min(v+4, h)
            u0, u1 = max(u-3, 0), min(u+4, w)
            patch = self.latest_depth[v0:v1, u0:u1]
            vals = patch[patch > 0]
            if vals.size == 0:
                label = labels[i] if i < len(labels) else f"H{i}"
                cv2.putText(bgr, f"{label} z=?m", (u + 8, v + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                continue
            z = float(np.median(vals))

            # Warn once if depth isn’t aligned to color
            if self.depth_frame_id and self.depth_frame_id != 'cam2_color_optical_frame':
                self.get_logger().warn_once(
                    f"Depth frame_id is {self.depth_frame_id}; expected cam2_color_optical_frame. "
                    "Enable align_depth to avoid pixel mismatch."
                )

            # Deproject to camera coordinates and compute true range
            fx, fy, cx, cy = self.cam_model.fx(), self.cam_model.fy(), self.cam_model.cx(), self.cam_model.cy()
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            R = math.sqrt(X*X + Y*Y + z*z)

            label = labels[i] if i < len(labels) else f"H{i}"
            cv2.putText(bgr, f"{label} z={z:.3f} m | R={R:.3f} m",
                        (u + 8, v + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # always publish a frame
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))

def main():
    rclpy.init()
    n = WristDebug()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

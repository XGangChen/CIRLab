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
from ultralytics import YOLO


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


def put_text(img, txt, org, color=(255,255,255), scale=0.6, thick=2):
    cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class WristToBase(Node):
    def __init__(self):
        super().__init__('wrist_to_base')

        # ---------- parameters (override with --ros-args -p key:=value) ----------
        self.declare_parameter('color_topic',  '/camera/cam2/color/image_raw')
        self.declare_parameter('depth_topic',  '/camera/cam2/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',   '/camera/cam2/color/camera_info')
        self.declare_parameter('source_frame', 'cam2_color_optical_frame')
        self.declare_parameter('target_frame', 'platform_base')
        self.declare_parameter('min_det_conf', 0.30)  # MediaPipe
        self.declare_parameter('min_trk_conf', 0.30)  # MediaPipe

        # --- YOLO params ---
        self.declare_parameter('yolo_model_path', '/media/xgang/XGang-1T/CIRLab/MyResearch/CoBot/script/UR_pose_best.pt')
        self.declare_parameter('yolo_conf', 0.25)
        self.declare_parameter('yolo_imgsz', 640)
        # If your model.names already match, you can ignore this; otherwise override with a CSV string.
        # Example: "base,shoulder,elbow1,elbow2,elbow3,wrist"
        self.declare_parameter('ur3_class_names', 'UR_joints.csv')

        color_topic  = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic  = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic   = self.get_parameter('info_topic').get_parameter_value().string_value
        self.src     = self.get_parameter('source_frame').get_parameter_value().string_value
        self.tgt     = self.get_parameter('target_frame').get_parameter_value().string_value
        det_c        = float(self.get_parameter('min_det_conf').get_parameter_value().double_value)
        trk_c        = float(self.get_parameter('min_trk_conf').get_parameter_value().double_value)

        self.yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.yolo_conf       = float(self.get_parameter('yolo_conf').get_parameter_value().double_value)
        self.yolo_imgsz      = int(self.get_parameter('yolo_imgsz').get_parameter_value().integer_value)
        ur3_names_csv        = self.get_parameter('ur3_class_names').get_parameter_value().string_value

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

        # --- UR3 joint points + skeleton markers ---
        self.pub_ur3_points  = self.create_publisher(MarkerArray, '/ur3/joint_markers', 10)
        self.pub_ur3_skel    = self.create_publisher(Marker, '/ur3/skeleton', 10)
        # Optional: individual PointStamped topics for each joint
        self.pub_joint_points = {
            'base':     self.create_publisher(PointStamped, '/ur3/base_point', 10),
            'shoulder': self.create_publisher(PointStamped, '/ur3/shoulder_point', 10),
            'elbow1':   self.create_publisher(PointStamped, '/ur3/elbow1_point', 10),
            'elbow2':   self.create_publisher(PointStamped, '/ur3/elbow2_point', 10),
            'elbow3':   self.create_publisher(PointStamped, '/ur3/elbow3_point', 10),
            'wrist':    self.create_publisher(PointStamped, '/ur3/wrist_point', 10),
        }

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

        # ---------- YOLOv11 (UR3 joints) ----------
        try:
            self.yolo = YOLO(self.yolo_model_path)
            self.model_names = self.yolo.model.names  # dict: {class_id: name}
            self.get_logger().info(f"Loaded YOLO model: {self.yolo_model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model '{self.yolo_model_path}': {e}")
            self.yolo = None
            self.model_names = {}

        # Class name normalization & order for skeleton
        # Preferred order for links: base -> shoulder -> elbow1 -> elbow2 -> elbow3 -> wrist
        default_order = ['base', 'shoulder', 'elbow1', 'elbow2', 'elbow3', 'wrist']
        if ur3_names_csv.strip():
            self.ur3_names = [s.strip() for s in ur3_names_csv.split(',')]
        else:
            # Try to read from model.names, else fallback to defaults
            # We will later match by case-insensitive comparison against model.names
            self.ur3_names = default_order

        self.link_order = default_order

        self.get_logger().info(f"Subscribing: {color_topic}, {depth_topic}, {info_topic}")
        self.get_logger().info(f"Frames: {self.src} -> {self.tgt}")
        self.get_logger().info(f"UR3 class names (expected): {self.ur3_names}")

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

    def _pixels_to_cam_xyz(self, u, v, z):
        fx, fy = self.cam_model.fx(), self.cam_model.fy()
        cx, cy = self.cam_model.cx(), self.cam_model.cy()
        Xc = (u - cx) * z / fx
        Yc = (v - cy) * z / fy
        Zc = z
        return Xc, Yc, Zc

    def _to_base_frame(self, stamped_point: PointStamped):
        try:
            tf = self.tf_buffer.lookup_transform(self.tgt, self.src, rclpy.time.Time())
            return tf2gm.do_transform_point(stamped_point, tf)
        except Exception as e:
            if not self._tf_warned:
                self.get_logger().warn(f'No TF {self.tgt}<-{self.src}: {e}')
                self._tf_warned = True
            return None

    def _publish_marker_sphere(self, ns, mid, frame_id, xyz, color_rgb=(0.2, 0.6, 1.0), scale=0.08):
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.a = 1.0
        m.color.r, m.color.g, m.color.b = color_rgb
        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    def _publish_marker_linestrip(self, ns, mid, frame_id, xyz_list, color_rgb=(1.0, 1.0, 0.2), width=0.03):
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.a = 1.0
        m.color.r, m.color.g, m.color.b = color_rgb
        from geometry_msgs.msg import Point
        m.points = [Point(x=x, y=y, z=z) for (x,y,z) in xyz_list]
        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    def _best_joint_detections(self, result, w, h):
        """
        From a single YOLO result, pick the highest-confidence box per UR3 class.
        Returns dict: name -> (u, v, conf, bbox)
        """
        if result is None or result.boxes is None or len(result.boxes) == 0:
            return {}

        # Build mapping from class name (normalized) to class id(s) in model
        model_name_map = {int(k): str(v).strip() for k, v in getattr(self, 'model_names', {}).items()}
        norm = lambda s: s.lower().strip()

        # Accept either exactly matching ur3_names or model's names if already correct
        wanted_names = [norm(n) for n in self.ur3_names]
        name_to_ids = {}
        for cid, cname in model_name_map.items():
            c = norm(cname)
            if c in wanted_names:
                name_to_ids.setdefault(c, []).append(cid)

        picks = {}
        for box in result.boxes:
            cls_id = int(box.cls.item()) if hasattr(box.cls, 'item') else int(box.cls)
            conf   = float(box.conf.item() if hasattr(box.conf, 'item') else box.conf)
            if conf < self.yolo_conf:
                continue
            cname = norm(model_name_map.get(cls_id, f'cls{cls_id}'))
            if cname not in name_to_ids:
                continue

            # center pixel of bbox
            xyxy = box.xyxy[0].tolist()
            x0, y0, x1, y1 = xyxy
            u = int(np.clip((x0+x1)/2.0, 0, w-1))
            v = int(np.clip((y0+y1)/2.0, 0, h-1))

            cur = picks.get(cname)
            if (cur is None) or (conf > cur[2]):
                picks[cname] = (u, v, conf, (x0, y0, x1, y1))

        return picks  # keys are normalized names

    def on_color(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = bgr.shape[:2]

        # If not ready, still publish a frame so rqt updates
        if not self.have_info or self.latest_depth is None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))
            return

        # --------- 1) Hands (human wrists) ----------
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res_hand = self.hands.process(rgb)
        hands = res_hand.multi_hand_landmarks or []
        labels = [c.classification[0].label for c in (res_hand.multi_handedness or [])]  # 'Left'/'Right'

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

            if np.isfinite(z):
                Xc, Yc, Zc = self._pixels_to_cam_xyz(u, v, z)
                pt_cam = PointStamped()
                pt_cam.header.stamp = msg.header.stamp
                pt_cam.header.frame_id = self.src
                pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = Xc, Yc, Zc

                pt_base = self._to_base_frame(pt_cam)
                if pt_base is None:
                    continue

                label = labels[i] if i < len(labels) else f"H{i}"
                color = (0, 0, 255) if label == 'Left' else (0, 255, 0)
                cv2.circle(bgr, (u, v), 8, color, -1)
                put_text(bgr, f"{label} z={Zc:.3f}m (k={k_used})", (u + 8, v - 8))

                # publish left/right points
                if label == 'Left':
                    self.pub_point_left.publish(pt_base); mid = 0; col = (1.0, 0.2, 0.2)
                else:
                    self.pub_point_right.publish(pt_base); mid = 1; col = (0.2, 1.0, 0.2)

                # marker (big + persistent)
                m = Marker()
                m.header = pt_base.header
                m.ns = 'wrist'
                m.id = mid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = pt_base.point.x
                m.pose.position.y = pt_base.point.y
                m.pose.position.z = pt_base.point.z
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.12
                m.lifetime = Duration(sec=0, nanosec=0)
                m.color.a = 1.0
                m.color.r, m.color.g, m.color.b = col
                markers.append(m)

        # publish hand markers
        for m in markers:
            self.pub_marker.publish(m)
        if markers:
            self.pub_marker_arr.publish(MarkerArray(markers=markers))

        # --------- 2) YOLOv11 UR3-joint detection ----------
        ur3_px = {}       # name(normalized) -> (u,v,conf,bbox)
        ur3_xyz_base = {} # name(normalized) -> (x,y,z) in base frame
        if self.yolo is not None:
            try:
                # NOTE: Ultralytics accepts BGR or RGB np arrays; we'll pass BGR as-is.
                # We request a single result; stream=False returns list
                results = self.yolo.predict(
                    source=bgr,
                    conf=self.yolo_conf,
                    imgsz=self.yolo_imgsz,
                    verbose=False,
                    device=None  # auto
                )
                result = results[0] if results else None
                ur3_px = self._best_joint_detections(result, w, h)

                # Draw detections + compute 3D + publish markers
                joint_markers = []
                for jname in self.link_order:
                    key = jname.lower()
                    if key not in ur3_px:
                        continue
                    u, v, conf, (x0,y0,x1,y1) = ur3_px[key]
                    # bbox
                    cv2.rectangle(bgr, (int(x0),int(y0)), (int(x1),int(y1)), (255, 200, 0), 2)
                    put_text(bgr, f"{jname} {conf:.2f}", (int(x0), max(0, int(y0)-6)))

                    # depth & 3D
                    z, k_used = robust_depth_at(u, v, self.latest_depth, h, w)
                    if not np.isfinite(z):
                        put_text(bgr, f"no depth", (u+8, v+14), (50,255,255))
                        continue
                    Xc, Yc, Zc = self._pixels_to_cam_xyz(u, v, z)
                    pt_cam = PointStamped()
                    pt_cam.header.stamp = msg.header.stamp
                    pt_cam.header.frame_id = self.src
                    pt_cam.point.x, pt_cam.point.y, pt_cam.point.z = Xc, Yc, Zc

                    pt_base = self._to_base_frame(pt_cam)
                    if pt_base is None:
                        continue
                    ur3_xyz_base[key] = (pt_base.point.x, pt_base.point.y, pt_base.point.z)

                    # image annotation
                    cv2.circle(bgr, (u, v), 6, (0, 255, 255), -1)
                    put_text(bgr, f"z={Zc:.3f}m", (u+8, v-8))

                    # 3D marker for the joint
                    jid = self.link_order.index(jname)  # stable small id
                    joint_markers.append(
                        self._publish_marker_sphere(
                            ns='ur3_joints', mid=jid, frame_id=self.tgt,
                            xyz=ur3_xyz_base[key],
                            color_rgb=(0.2, 0.6, 1.0), scale=0.08
                        )
                    )

                # publish joint spheres
                if joint_markers:
                    self.pub_ur3_points.publish(MarkerArray(markers=joint_markers))

                # publish skeleton as LINE_STRIP in base frame (in given order when available)
                xyz_chain = [ur3_xyz_base[k] for k in self.link_order if k in ur3_xyz_base]
                if len(xyz_chain) >= 2:
                    skel_marker = self._publish_marker_linestrip(
                        ns='ur3_skeleton', mid=100, frame_id=self.tgt,
                        xyz_list=xyz_chain, color_rgb=(1.0, 1.0, 0.2), width=0.03
                    )
                    self.pub_ur3_skel.publish(skel_marker)

                # 2D overlay skeleton on debug image
                pts2d = []
                for jname in self.link_order:
                    k = jname.lower()
                    if k in ur3_px:
                        pts2d.append((ur3_px[k][0], ur3_px[k][1]))
                for i in range(len(pts2d) - 1):
                    cv2.line(bgr, pts2d[i], pts2d[i+1], (0, 215, 255), 3)

            except Exception as e:
                self.get_logger().warn(f"YOLO inference failed: {e}")

        # --------- publish debug image every frame ----------
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, 'bgr8'))

    # ------------------- ros spin -------------------
    def destroy_node(self):
        try:
            if hasattr(self, 'hands') and self.hands:
                self.hands.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WristToBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

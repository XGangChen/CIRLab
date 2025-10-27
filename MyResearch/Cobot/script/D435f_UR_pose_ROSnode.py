#!/usr/bin/env python3
from html import parser
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
import numpy as np
import argparse
import threading
import time

import os, argparse
DEFAULT_MODEL = "/media/xgang/XGang-1T/CIRLab/MyResearch/CoBot/script/UR_pose_best.pt"
DEFAULT_IMAGE_TOPIC = "/cam2/color/image_raw"
DEFAULT_OUT_IMG = "/ur3_pose/annotated"
DEFAULT_OUT_KPT = "/ur3_pose/keypoints"


JOINT_NAMES = ['base', 'shoulder', 'elbow1', 'elbow2', 'elbow3', 'wrist']
JOINT_COLORS = [
    (200, 0, 200), (255, 150, 50), (0, 100, 255),
    (255, 255, 0), (255, 255, 0), (255, 0, 255)
]
SKELETON = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)]

def draw_pose(frame, kpt):
    for i, j in SKELETON:
        if kpt[i][2] > 0.5 and kpt[j][2] > 0.5:
            pt1 = tuple(map(int, kpt[i][:2]))
            pt2 = tuple(map(int, kpt[j][:2]))
            cv2.line(frame, pt1, pt2, JOINT_COLORS[i], 3)
    for i, (x, y, v) in enumerate(kpt):
        if v > 0.5:
            cv2.circle(frame, (int(x), int(y)), 6, JOINT_COLORS[i], -1)
            cv2.putText(frame, JOINT_NAMES[i], (int(x)+6, int(y)-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

def parse_cli():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.getenv("UR_MODEL", DEFAULT_MODEL))
    ap.add_argument("--image-topic", default=os.getenv("UR_IMAGE_TOPIC", DEFAULT_IMAGE_TOPIC))
    ap.add_argument("--out-image-topic", default=os.getenv("UR_OUT_IMG", DEFAULT_OUT_IMG))
    ap.add_argument("--out-kpt-topic", default=os.getenv("UR_OUT_KPT", DEFAULT_OUT_KPT))
    ap.add_argument("--conf", type=float, default=float(os.getenv("UR_CONF", 0.30)))
    args, _ = ap.parse_known_args()  # tolerate --ros-args etc.
    return args

class UR3PoseNode(Node):
    def __init__(self, model_path, conf, image_topic, out_image_topic, out_kpt_topic):
        super().__init__('ur3_pose_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.conf = conf

        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.pub_img = self.create_publisher(Image, out_image_topic, qos)
        self.pub_kpt = self.create_publisher(Float32MultiArray, out_kpt_topic, 10)

        self.last_infer_t = 0.0
        self.max_fps = 30.0  # simple throttle to avoid backlog

        self.get_logger().info(f'UR3PoseNode ready. Subscribing: {image_topic}')
        self.get_logger().info(f'Publishing annotated: {out_image_topic}')
        self.get_logger().info(f'Publishing keypoints: {out_kpt_topic}')

    def image_cb(self, msg: Image):
        now = time.time()
        if now - self.last_infer_t < (1.0 / self.max_fps):
            return
        self.last_infer_t = now

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        with torch.inference_mode():
            results = self.model.predict(source=cv_img, conf=self.conf, save=False, show=False, verbose=False)
        kpts = results[0].keypoints

        annotated = cv_img.copy()
        out_array = None

        if kpts is not None and kpts.conf is not None and len(kpts.conf) > 0:
            avg_conf = kpts.conf.mean(dim=1)
            best_idx = int(torch.argmax(avg_conf))
            best_kpt = kpts.data[best_idx].detach().cpu().numpy()  # (6, 3)
            draw_pose(annotated, best_kpt)

            # Publish keypoints as Float32MultiArray with dims [6,3]
            out_array = Float32MultiArray()
            out_array.layout.dim = [
                MultiArrayDimension(label='joints', size=6, stride=18),
                MultiArrayDimension(label='fields(x,y,conf)', size=3, stride=3),
            ]
            out_array.data = best_kpt.astype(np.float32).flatten().tolist()
        else:
            # no detection: publish empty array (size 0)
            out_array = Float32MultiArray()
            out_array.data = []

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header  # keep timestamps/frame_id
        self.pub_img.publish(out_msg)

        # Publish keypoints
        self.pub_kpt.publish(out_array)

# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--model', required=True, help='/media/xgang/XGang-1T/CIRLab/MyResearch/CoBot/script/UR_pose_best.pt')
#     parser.add_argument('--conf', type=float, default=0.30)
#     parser.add_argument('--image-topic', default='/camera/cam2/color/image_raw')
#     parser.add_argument('--out-image-topic', default='/ur3_pose/annotated')
#     parser.add_argument('--out-kpt-topic', default='/ur3_pose/keypoints')
#     args, _ = parser.parse_known_args()
    
#     rclpy.init()
#     node = UR3PoseNode(args.model, args.conf, args.image_topic, args.out_image_topic, args.out_kpt_topic)
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# --- keep everything above as-is, including DEFAULT_* and parse_cli() ---

def main():
    # use your tolerant parser with safe defaults
    args = parse_cli()

    rclpy.init()  # no ROS flags needed when launching from VS Code
    node = UR3PoseNode(
        model_path=args.model,
        conf=args.conf,
        image_topic=args.image_topic,
        out_image_topic=args.out_image_topic,
        out_kpt_topic=args.out_kpt_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

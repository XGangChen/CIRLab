#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles
from sensor_msgs.msg import CameraInfo
import tf2_ros

class CamInfoTfCheck(Node):
    def __init__(self):
        super().__init__('caminfo_tf_check')

        self.declare_parameter('info_topic', '/camera/cam2/color/image_raw/theora')
        self.declare_parameter('source_frame', 'cam2_color_optical_frame')
        self.declare_parameter('target_frame', 'platform_base')

        info_topic   = self.get_parameter('info_topic').get_parameter_value().string_value
        self.src     = self.get_parameter('source_frame').get_parameter_value().string_value
        self.tgt     = self.get_parameter('target_frame').get_parameter_value().string_value

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.on_info, QoSProfile(depth=10))

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.check_tf)

        self.got_info = False
        self.get_logger().info(f"Waiting for CameraInfo on {info_topic} and TF {self.tgt} <- {self.src} ...")

    def on_info(self, msg: CameraInfo):
        if not self.got_info:
            self.got_info = True
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            self.get_logger().info(
                f"CameraInfo received (frame_id={msg.header.frame_id}) "
                f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}"
            )

    def check_tf(self):
        try:
            tr = self.tf_buffer.lookup_transform(self.tgt, self.src, rclpy.time.Time())
            t = tr.transform.translation
            q = tr.transform.rotation
            self.get_logger().info(
                f"TF OK {self.tgt} <- {self.src} | t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) "
                f"q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})"
            )
        except Exception as e:
            self.get_logger().warn(f"TF not available yet: {e}")

def main():
    rclpy.init()
    n = CamInfoTfCheck()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

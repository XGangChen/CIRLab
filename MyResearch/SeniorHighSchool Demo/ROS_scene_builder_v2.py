#!/usr/bin/env python3
# scene_builder.py
# ROS 2 node: UR3 base_link is now the CENTER (0,0,0).
# The platform and camera positions are calculated relative to the robot.

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

# Converts Euler angles (roll, pitch, yaw) to a quaternion (ZYX)
def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return (qx, qy, qz, qw)

# Rotates a vector v by quaternion q
def rotate_vec_by_quat(v, q):
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)

# Returns the conjugate (inverse rotation) of a quaternion
def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])

class SceneBuilder(Node):
    def __init__(self):
        super().__init__('scene_builder')

        # ---------- Frames ----------
        # NEW HIERARCHY: world -> base_link -> platform_base -> camera
        self.world_frame = 'world'
        self.ur_root_frame = 'base_link'        # This is now our origin (0,0,0)
        self.platform_frame = 'platform_base'
        self.camera_frame = 'cam2_color_optical_frame'

        # ---------- CONFIGURATION (Physical Measurements) ----------
        # We keep these exactly as you measured them.
        # "Where is the robot relative to the platform center?"
        ur_xyz_wrt_plat = (-0.20, -0.35, 0.38)
        ur_rpy_wrt_plat = (math.pi/2, 0, -math.pi/2)
        
        # "Where is the camera relative to the platform center?"
        cam_xyz_wrt_plat = (0.0, 0.0, 1.25)
        cam_rpy_wrt_plat = (math.pi, 0, 0.0)

        # ---------- GEOMETRY ----------
        self.platform_size = (0.80, 0.56, 0.02)
        self.cube_size     = (0.40, 0.14, 0.50)
        self.cube_center   = (0.0, -0.35, 0.25) # Relative to platform center

        # ---------- MATH: Invert the Platform->Robot transform ----------
        # We need T_robot_to_platform.
        # T_inv = ( R^T,  -R^T * t )
        
        # 1. Get Forward Rotation (Platform -> Robot)
        q_plat_to_ur = quat_from_rpy(*ur_rpy_wrt_plat)
        
        # 2. Get Inverse Rotation (Robot -> Platform) = Conjugate
        q_ur_to_plat = quat_conjugate(q_plat_to_ur)

        # 3. Get Inverse Translation
        # t_inv = rotate_vector(-t, q_inv)
        neg_t = (-ur_xyz_wrt_plat[0], -ur_xyz_wrt_plat[1], -ur_xyz_wrt_plat[2])
        t_ur_to_plat = rotate_vec_by_quat(neg_t, q_ur_to_plat)

        # ---------- TF Broadcasting ----------
        self.static_broadcaster = StaticTransformBroadcaster(self)
        t0 = Time().to_msg()

        # 1. world -> base_link (Identity: Robot is at World Origin)
        t_world_ur = TransformStamped()
        t_world_ur.header.stamp = t0
        t_world_ur.header.frame_id = self.world_frame
        t_world_ur.child_frame_id = self.ur_root_frame
        t_world_ur.transform.rotation.w = 1.0

        # 2. base_link -> platform_base (The calculated inverse)
        t_ur_plat = TransformStamped()
        t_ur_plat.header.stamp = t0
        t_ur_plat.header.frame_id = self.ur_root_frame
        t_ur_plat.child_frame_id = self.platform_frame
        t_ur_plat.transform.translation.x = float(t_ur_to_plat[0])
        t_ur_plat.transform.translation.y = float(t_ur_to_plat[1])
        t_ur_plat.transform.translation.z = float(t_ur_to_plat[2])
        t_ur_plat.transform.rotation.x = float(q_ur_to_plat[0])
        t_ur_plat.transform.rotation.y = float(q_ur_to_plat[1])
        t_ur_plat.transform.rotation.z = float(q_ur_to_plat[2])
        t_ur_plat.transform.rotation.w = float(q_ur_to_plat[3])

        # 3. platform_base -> camera (Unchanged, camera moves with platform)
        cam_q = quat_from_rpy(*cam_rpy_wrt_plat)
        t_plat_cam = TransformStamped()
        t_plat_cam.header.stamp = t0
        t_plat_cam.header.frame_id = self.platform_frame
        t_plat_cam.child_frame_id = self.camera_frame
        t_plat_cam.transform.translation.x = float(cam_xyz_wrt_plat[0])
        t_plat_cam.transform.translation.y = float(cam_xyz_wrt_plat[1])
        t_plat_cam.transform.translation.z = float(cam_xyz_wrt_plat[2])
        t_plat_cam.transform.rotation.x = float(cam_q[0])
        t_plat_cam.transform.rotation.y = float(cam_q[1])
        t_plat_cam.transform.rotation.z = float(cam_q[2])
        t_plat_cam.transform.rotation.w = float(cam_q[3])

        self.static_broadcaster.sendTransform([t_world_ur, t_ur_plat, t_plat_cam])
        self.get_logger().info(f'Published TFs. World Origin is now {self.ur_root_frame}')

        # ---------- RViz Markers ----------
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)
        
        # Cache for axes drawing
        self.cam_xyz = cam_xyz_wrt_plat
        self.cam_quat = cam_q

    def _mk(self, ns, mid, mtype, frame, rgba, pose_xyz=(0.0,0,0), pose_q=(0,0,0,1), scale=(1,1,1)):
        m = Marker()
        m.header.frame_id = str(frame)
        m.header.stamp = Time().to_msg()
        m.ns, m.id, m.type, m.action = str(ns), int(mid), int(mtype), Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pose_xyz)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, pose_q)
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        return m

    def _axis_arrow(self, mid, p0, p1, color, frame_id):
        m = self._mk('axes', mid, Marker.ARROW, frame_id, color, scale=(0.01, 0.03, 0.06))
        m.points = [Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
                    Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))]
        return m

    def publish_markers(self):
        ma = MarkerArray()
        # NOTE: Markers are still drawn in 'platform_base' frame. 
        # Since TF handles the relationship to base_link, we don't need to change coordinates here!
        
        # 1) Platform slab
        ma.markers.append(self._mk(
            ns='scene', mid=1, mtype=Marker.CUBE, frame=self.platform_frame,
            rgba=(0.75, 0.75, 0.75, 1.0), scale=self.platform_size))

        # Outline
        outline = self._mk(ns='scene', mid=2, mtype=Marker.LINE_STRIP, frame=self.platform_frame,
                           rgba=(0.2, 0.2, 0.2, 1.0), scale=(0.01, 0.0, 0.0))
        w, d, _ = self.platform_size
        corners = [(+w/2, +d/2, 0), (+w/2, -d/2, 0), (-w/2, -d/2, 0), (-w/2, +d/2, 0), (+w/2, +d/2, 0)]
        outline.points = [Point(x=float(x), y=float(y), z=float(z)) for (x, y, z) in corners]
        ma.markers.append(outline)

        # 2) Cube
        ma.markers.append(self._mk(
            ns='scene', mid=3, mtype=Marker.CUBE, frame=self.platform_frame,
            rgba=(0.3, 0.6, 0.9, 0.9), pose_xyz=self.cube_center, scale=self.cube_size))

        # 3) Camera Axes (in platform frame)
        axis_len = 0.25
        p0 = self.cam_xyz
        ex, ey, ez = (axis_len,0,0), (0,axis_len,0), (0,0,axis_len)
        ex_w = rotate_vec_by_quat(ex, self.cam_quat)
        ey_w = rotate_vec_by_quat(ey, self.cam_quat)
        ez_w = rotate_vec_by_quat(ez, self.cam_quat)
        p1x = (p0[0]+ex_w[0], p0[1]+ex_w[1], p0[2]+ex_w[2])
        p1y = (p0[0]+ey_w[0], p0[1]+ey_w[1], p0[2]+ey_w[2])
        p1z = (p0[0]+ez_w[0], p0[1]+ez_w[1], p0[2]+ez_w[2])
        
        ma.markers.append(self._axis_arrow(10, p0, p1x, (1,0,0,1), self.platform_frame))
        ma.markers.append(self._axis_arrow(11, p0, p1y, (0,1,0,1), self.platform_frame))
        ma.markers.append(self._axis_arrow(12, p0, p1z, (0,0,1,1), self.platform_frame))

        self.marker_pub.publish(ma)

def main():
    rclpy.init()
    node = SceneBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
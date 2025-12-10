#!/usr/bin/env python3
# ur3_socket_driver.py
# 1. Moves to A (Using ROS Joint Action).
# 2. Checks for Wrist Detection (Trigger).
# 3. If Triggered: Moves A -> Manual C -> B (interpolating orientation).
# 4. If Not Triggered: Moves A -> B (interpolating orientation).

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

import time
import socket
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- CONFIGURATION ---
ROBOT_IP = "192.168.77.101"  # <--- CHANGE THIS TO YOUR ROBOT IP
WRIST_TOPIC = '/wrist_right_point_base' 

class UR3SocketDriver(Node):
    def __init__(self):
        super().__init__('ur3_socket_driver')
        
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        self.latest_wrist_point = None

        # 1. ROS Client (For FK and Initial Joint Move)
        self._traj_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self._fk_client = self.create_client(GetPositionFK, 'compute_fk')
        
        # 2. Wrist Subscriber
        self.create_subscription(PointStamped, WRIST_TOPIC, self.wrist_cb, 10)

        self.get_logger().info("Waiting for Driver & FK Service...")
        self._traj_client.wait_for_server()
        self._fk_client.wait_for_service()
        self.get_logger().info("Ready!")

    def wrist_cb(self, msg):
        # We still listen to the wrist to use it as a TRIGGER
        self.latest_wrist_point = msg.point

    def wait_for_future(self, future):
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return future.result()

    def move_to_joints(self, target_joints, duration_sec=5.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in target_joints]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        traj.points.append(point)
        goal.trajectory = traj
        
        self.get_logger().info(f"Moving to A (Joints)...")
        future = self._traj_client.send_goal_async(goal)
        res = self.wait_for_future(future)
        if not res.accepted: return False
        
        res_future = res.get_result_async()
        result = self.wait_for_future(res_future)
        return result.result.error_code == 0

    def get_tcp_data(self, joint_values):
        req = GetPositionFK.Request()
        req.header.frame_id = "base"
        req.fk_link_names = ["tool0"]
        rs = RobotState()
        rs.joint_state.name = self.joint_names
        rs.joint_state.position = [float(j) for j in joint_values]
        req.robot_state = rs
        
        future = self._fk_client.call_async(req)
        resp = self.wait_for_future(future)
        
        if resp.error_code.val != 1: return None
        
        pos = resp.pose_stamped[0].pose.position
        ori = resp.pose_stamped[0].pose.orientation
        
        r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        rot_vec = r.as_rotvec() 
        return (pos.x, pos.y, pos.z, rot_vec[0], rot_vec[1], rot_vec[2])

    def send_socket_script(self, c_pose, b_pose, accel=0.5, speed=0.10):
        """
        Executes A -> C (Manual) -> B (End)
        """
        cx, cy, cz, crx, cry, crz = c_pose
        bx, by, bz, brx, bry, brz = b_pose

        script = (
            f"def my_custom_move():\n"
            f"  movel(p[{cx},{cy},{cz},{crx},{cry},{crz}], a={accel}, v={speed}, r=0.05)\n"
            f"  movel(p[{bx},{by},{bz},{brx},{bry},{brz}], a={accel}, v={speed}, r=0.0)\n"
            f"end\n"
            f"my_custom_move()\n"
        )
        self._send_script(script)

    def send_socket_script_direct(self, b_pose, accel=0.5, speed=0.10):
        """
        Executes A -> B (Directly)
        """
        bx, by, bz, brx, bry, brz = b_pose

        script = (
            f"def my_custom_move_direct():\n"
            f"  movel(p[{bx},{by},{bz},{brx},{bry},{brz}], a={accel}, v={speed}, r=0.0)\n"
            f"end\n"
            f"my_custom_move_direct()\n"
        )
        self._send_script(script)

    def _send_script(self, script):
        self.get_logger().info(f"Connecting to Robot {ROBOT_IP}:30002...")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((ROBOT_IP, 30002))
            s.sendall(script.encode('utf-8'))
            s.close()
            self.get_logger().info("Script sent successfully via Socket.")
        except Exception as e:
            self.get_logger().error(f"Socket Error: {e}")

def main():
    rclpy.init()
    bot = UR3SocketDriver()

    try:
        # ==========================================
        # 1. DEFINE POINTS (Joint Angles in Radians)
        # ==========================================
        
        # Point A: Start
        start_joints = [-3.1665, -0.1983, 0.3315, -1.2329, -0.0497, -6.7186]
        
        # Point C: Manual Avoidance Point (TODO: Set your actual values here)
        mid_joints_c = [-3.1807, -1.1060, 1.9888, -0.8009, -0.0430, -7.9497] 

        # Point B: End
        end_joints_ref = [-3.1671, 0.4690, 1.0725, -1.2330, -0.0398, -6.7188]

        # ==========================================
        # 2. MOVE TO START (A)
        # ==========================================
        if bot.move_to_joints(start_joints, duration_sec=5.0):
            print("   -> Reached A. Calculating Coordinates...")
            time.sleep(1.0)
            
            # Note: We no longer 'lock' the rotation of A.
            # We calculate the full pose (XYZ + RxRyRz) for B and C independently.

            # Calculate B Coordinates (Position + Orientation)
            target_b = bot.get_tcp_data(end_joints_ref)
            
            # Calculate C Coordinates (Position + Orientation)
            target_c = bot.get_tcp_data(mid_joints_c)

            if not target_b or not target_c:
                print("FK Failed. Check joint values.")
                return

            # ==========================================
            # 3. CHECK WRIST TRIGGER
            # ==========================================
            print("   Waiting for Wrist Detection (Trigger)...")
            attempts = 0
            while bot.latest_wrist_point is None and attempts < 40:
                rclpy.spin_once(bot, timeout_sec=0.1)
                attempts += 1
            
            # ==========================================
            # 4. EXECUTE PATH
            # ==========================================
            if bot.latest_wrist_point:
                print("   [!] Wrist Detected!")
                print("   -> Sending A -> C (Manual) -> B...")
                bot.send_socket_script(target_c, target_b)
                
            else:
                print("   [.] No Wrist Detected.")
                print("   -> Sending A -> B (Direct)...")
                bot.send_socket_script_direct(target_b)

            print("Done! 'External Control' should have stopped.")

    except KeyboardInterrupt:
        pass
    
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Cobot-Human Collision Avoidance (UR3 + Kinect V2 + RealSense D435f)  

> **Goal:** Design a safety algorithm for the Human-Robot workspace to avoid collisions. The experimental environment setup is using [UR3](https://www.universal-robots.com/download/manuals-cb-series/user/ur3/33/user-manual-ur3-cb-series-sw33-english-international/) as a collaborative robot, using Kinect V2 to detect human hand's 3D position, and using RealSense D435f to do human arm's pose estimation.

## 1) Experimental Environment
### Universal Robot UR3
To set up the UR3 ROS2 Humble environment, follow this guide document [UR3](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/UR3) in this repository. I tried to make it as clear as possible for all processes.

### Using KinectV2 and Realsense D435f by ROS2 Humble
I installed the packages of [KinectV2](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/%20KinectV2) and [RealSense](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/RealSense) separately. There are several issues when I tried to run two cameras side-by-side using ROS2, so here’s a clean, practical checklist + ready-to-run examples so you can run Kinect v2 (kinect2_bridge) and RealSense (realsense2_camera) side-by-side:
<details>
  <summary>
    Troubleshooting
  </summary>
  https://chatgpt.com/share/68de2bf1-5fbc-800f-8a53-534d66b82783
</details>
---

# WorkFlow

The commands to launch devices and drivers, in order:  
1. Launch the RealSense D435f camera
   ```bash
   ros2 launch realsense2_camera rs_launch.py   camera_name:=cam2 enable_color:=true enable_depth:=true   align_depth.enable:=true initial_reset:=true
   ```
2. Launch the UR3 driver to get the robot state
   ```bash
   # To connect the URSim UR3
   ros2 launch ur_robot_driver ur_control.launch.py     ur_type:=ur3     robot_ip:=127.0.0.1     reverse_ip:=192.168.105.45     launch_rviz:=false

   # To connect the real-world UR3
   ros2 launch ur_robot_driver ur_control.launch.py     ur_type:=ur3     robot_ip:=192.168.77.101     launch_rviz:=false
   ```
3. Launch the MoveIt 2
   ```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
   ```
4. Run the Python script to detect wrists by MediaPipe
   ```bash
   python3 D435f_wrist_to_base.py
   ```
5. Using RQT to get images from the camera
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ````
6. Run the Python script to launch the scene builder
   ```bash
   python3 Ros_scene_builder.py
   ```
7. Run the RViz
   ```bash
   rviz2
   ```

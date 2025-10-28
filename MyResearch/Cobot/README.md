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
2. Camera coordinate setup in TFtree
   ```bash
   ros2 run tf2_ros static_transform_publisher   0 0 1.25 3.14159265 -3.14159265 0  platform_base cam2_color_optical_frame
   ```
3. Run the Python script to detect wrists by MediaPipe
   ```bash
   python3 wrist_to_base.py \
    --ros-args \
    -p color_topic:=/camera/cam2/color/image_raw \
    -p depth_topic:=/camera/cam2/aligned_depth_to_color/image_raw \
    -p info_topic:=/camera/cam2/color/camera_info \
    -p source_frame:=cam2_color_optical_frame \
    -p target_frame:=platform_base
   ```
4. Using RQT to get images from the camera
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ````
5. Launch the UR3 driver to get the robot state
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur3   robot_ip:=127.0.0.1   launch_rviz:=false
   ```
6. Publish a static transform from `platform_base` to `base_link`
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.70710678 0 0.70710678 platform_base base
   ```
7. Run the RViz
   ```bash
   rviz2
   ```

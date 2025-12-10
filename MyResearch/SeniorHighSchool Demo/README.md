# WorkFlow

## ROS
- Launch RealSense Driver
  ```bash
  ros2 launch realsense2_camera rs_launch.py   camera_name:=cam2 enable_color:=true enable_depth:=true   align_depth.enable:=true initial_reset:=true
  ```
- Launch UR3 Driver
  ```bash
  ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur3   robot_ip:=192.168.77.101   launch_rviz:=false
  ```
- Launch MoveIt2
  ```bash
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
  ```

## Python
- Build the virtual scene
  ```bash
  python ROS_scene_builder_v2.py
  ```
- Detect the wrists
  ```bash
  python D435f_wrist_to_base_v2.py
  ```
- Show GUI detection
  ```bash
  python D435f_mediapipe_wrist_debug.py
  ```
- Run the UR3 control command
  ```bash
  python ur3_direct_driver.py
  ```

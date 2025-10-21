# Function Explanation
I'll explain what each script works for. The Python scripts need to be paired with ROS2 commands and tools.  
There's the [ROS2](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence/ROS2) folder under [RealSense](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence) in this repository.  
We need to launch the camera before running these scripts. Please check it out.  

## mediapipe_wrist_base.py
This script detects a wrist in the RGB image using MediaPipe, reads the aligned depth at that pixel, projects it to 3D in the camera frame, transforms that point into the robot base frame via TF2, and then publishes both a `PointStamped` and an RViz sphere marker at the wrist position. 

### High-level Data Flow
1. Subscribe to `color`, `aligned depth`, and `camera_info` (intrinsics) from cam2.
2. Run **MediaPipe Hands** on each color frame → get the **wrist landmark** (index 0).
3. From the wrist pixel `(u,v)`, take a **7×7 median depth** (ignoring zeros) → robust `z`.
4. **Deproject** using pinhole intrinsics `(fx, fy, cx, cy)` to get `(Xc, Yc, Zc)` in `cam2_color_optical_frame`.
5. **TF2** transforms that point into `platform_base`.
6. Publish `/wrist_point_base` (geometry_msgs/PointStamped) and `/wrist_marker` (visualization_msgs/Marker).


## mediapipe_wrist_debug.py

## cam_info_tf_check.py

## wrist_to_base.py

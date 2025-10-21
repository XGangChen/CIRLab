# Launch & Tools
This project is using ROS2 Humble to launch the RealSense D435f camera. I'll go through all the commands and tools I used in this document.

## Launch the Camera
```
ros2 launch realsense2_camera rs_launch.py   camera_name:=cam2 enable_color:=true enable_depth:=true   align_depth.enable:=true initial_reset:=true
```

## TFtree2 Environment Setup
```
ros2 run tf2_ros static_transform_publisher   0 0 1.25 3.14159265 -3.14159265 0  platform_base cam2_color_optical_frame
```

## Using RQT to See the Images
```
ros2 run rqt_image_view rqt_image_view
```

## Using RViz2 to Get the Whole Environment
```
rviv2
```

# RealSense Environment Setup
## RealSense Viewer
To use the Intel RealSense D-series, I followed the [librealsense](https://github.com/IntelRealSense/librealsense) to build using vcpkg. Then I followed the document [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) to install the package. You can verify the package by running `realsense-viewer`. 

## Librealsense2
My device is using Ubuntu 22.04 LTS. Follow the [Linux Ubuntu Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) document to install librealsense2 step-by-step. While building librealsense2 SDK, there was an error about permission denied. I just solved this problem by adding `sudo` before the cmake command.

## Realsense ROS package
### Launch RealSense
To use the RealSense camera with ROS, follow the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) GitHub. My device has installed ROS2 Humble already, and also installed the latest Intel RealSense SDK 2.0 by following the [librealsense GitHub](https://github.com/IntelRealSense/librealsense), so I skipped the first two steps of [Installation on Ubuntu](https://github.com/IntelRealSense/realsense-ros#installation-on-ubuntu). Then in step 3, I choose option 2 to install from source since I've already installed other ROS2 packages. The important thing is to ensure that the packages are not mutually exclusive, or you will need to rebuild.  
To run the RealSense and use rqt to see the image, follow the command below:  
```
ros2 launch realsense2_camera rs_launch.py

# View color image
ros2 run rqt_image_view rqt_image_view
```
### Bring Up Two RealSense Cameras

---

**1. Get each camera's serial number**
  ```
  # In terminal
  rs-enumerate-devices | grep Serial
  
  # my results
  Serial Number                 : 	841612071686
  Asic Serial Number            : 	850123050984
  Serial Number                 : 	317622075526
  Asic Serial Number            : 	318123026975
  ```

---

**2. Launch the two cameras**  
  Terminal A -> Camera 1
  ```
  source /opt/ros/humble/setup.bash
  source ~/ros_ws/install/setup.bash
  
  ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=cam1 \
    camera_name:=cam1 \
    serial_no:="_841612071686" \
    rgb_camera.profile:=1280x720x30 \
    depth_module.profile:=1280x720x30 \
    align_depth:=true
  ```
  Terminal B -> Camera 2
  ```
  source /opt/ros/humble/setup.bash
  source ~/ros_ws/install/setup.bash
  
  ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=cam2 \
    camera_name:=cam2 \
    serial_no:="_317622075526" \
    rgb_camera.profile:=1280x720x30 \
    depth_module.profile:=1280x720x30 \
    align_depth:=true
  ```
  > **Notes:** Set both `camera_namespace` and `camera_name` uniquely (e.g., `cam1`, `cam2`). This keeps topics and TF frames separate (e.g., `cam1_link`, `cam2_link`).

---

**3. Open rqt and view the images**  
  ```
  rqt
  ```

---

**4. Optional: enable point clouds**  
  Add this to the launch line if you want live RGB-D point clouds:
  ```
  pointcloud.enable:=true
  ```
  Then visualize in RViz2 (not rqt): 
  ```
  ros2 run rviz2 rviz2
  # Add PointCloud2 for "/<ns>/depth/color/points".
  ```

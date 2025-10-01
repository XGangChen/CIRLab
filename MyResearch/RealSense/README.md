# RealSense Environment Setup
## RealSense Viewer
To use the Intel RealSense D-series, I followed the [librealsense](https://github.com/IntelRealSense/librealsense) to build using vcpkg. Then I followed the document [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) to install the package. You can verify the package by running `realsense-viewer`. 

## Librealsense2
My device is using Ubuntu 22.04 LTS. Follow the [Linux Ubuntu Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) document to install librealsense2 step-by-step. While building librealsense2 SDK, there was an error about permission denied. I just solved this problem by adding `sudo` before the cmake command.

## Realsense ROS package
To use the RealSense camera with ROS, follow the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) GitHub. My device has installed ROS2 Humble already, and also installed the latest Intel RealSense SDK 2.0 by following the [librealsense GitHub](https://github.com/IntelRealSense/librealsense), so I skipped the first two steps of [Installation on Ubuntu](https://github.com/IntelRealSense/realsense-ros#installation-on-ubuntu). Then in step 3, I choose option 2 to install from source since I've already installed other ROS2 packages. The important thing is to ensure that the packages are not mutually exclusive, or you will need to rebuild.

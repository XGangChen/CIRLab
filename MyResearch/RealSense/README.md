# RealSense Environment Setup
## RealSense Viewer
To use the Intel RealSense D-series, I followed the [librealsense](https://github.com/IntelRealSense/librealsense) to build using vcpkg. Then I followed the document [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) to install the package. You can verify the package by running `realsense-viewer`. 

## Librealsense2
My device is using Ubuntu 22.04 LTS. Follow the [Linux Ubuntu Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) document to install librealsense2 step-by-step. While building librealsense2 SDK, there was an error about permission denied. I just solved this problem by adding `sudo` before the cmake command.

## 

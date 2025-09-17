# UR3 ROS2 Humble Environment
[The Universal Robots ROS2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)

## The installation
I followed this [tutorial document](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/toc.html) to set up the UR3_ROS2_Humble environment. But there are some issues while doing the Compilation:
Building the kernel using `make -j $(getconf _NPROCESSORS_ONLN) deb-pkg` would result in errors. I solved this by following GPT5's resolution. Here's the conversation [GPT5 solution](https://chatgpt.com/share/68ca7264-6fac-800f-898e-0cd488db0bb7).

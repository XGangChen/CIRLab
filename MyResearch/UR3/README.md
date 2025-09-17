# UR3 ROS2 Humble Environment
[The Universal Robots ROS2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)

## Setup for real-time scheduling
I followed this [tutorial document](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/toc.html) to set up the UR3_ROS2_Humble environment. But there are some issues while doing the Compilation:
Building the kernel using `make -j $(getconf _NPROCESSORS_ONLN) deb-pkg` would result in errors. I solved this by following GPT5's resolution. Here's the conversation [GPT5 solution](https://chatgpt.com/share/68ca7264-6fac-800f-898e-0cd488db0bb7).
After compilation, in the [Setup user privileges to use real-time scheduling](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/real_time.html#setup-user-privileges-to-use-real-time-scheduling) section. You need to add the "@realtime" lines to the .conf file.
```
sudo nano /etc/security/limits.conf

# in the .conf file, scroll to the bottom (all lines should be annotated by #)
#@realtime soft rtprio 99
#@realtime soft priority 99
#@realtime soft memlock 102400
#@realtime hard rtprio 99
#@realtime hard priority 99
#@realtime hard memlock 102400
```
I didn't do the optional step:[Disable CPU speed scaling](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/real_time.html#optional-disable-cpu-speed-scaling)


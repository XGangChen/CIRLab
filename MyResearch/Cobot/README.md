# The Goal
The goal for this project is to design a safety algorithm for the Human-Robot workspace to avoid collisions. The experimental environment setup is using [UR3](https://www.universal-robots.com/download/manuals-cb-series/user/ur3/33/user-manual-ur3-cb-series-sw33-english-international/) as a collaborative robot to help the experimenter get something. 

## Experimental Environment

### Using KinectV2 and Realsense D435f by ROS2 Humble
I installed the packages of KinectV2 and RealSense separately; you can see other projects in this repository. There are several issues when I tried to run two cameras side-by-side using ROS2, so here’s a clean, practical checklist + ready-to-run examples so you can run Kinect v2 (kinect2_bridge) and RealSense (realsense2_camera) side-by-side:
<details>
  <sumary>


* [Troubleshooting](https://chatgpt.com/share/68de2bf1-5fbc-800f-8a53-534d66b82783)

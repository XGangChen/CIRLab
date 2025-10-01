# The Goal
The goal for this project is to design a safety algorithm for the Human-Robot workspace to avoid collisions. The experimental environment setup is using [UR3](https://www.universal-robots.com/download/manuals-cb-series/user/ur3/33/user-manual-ur3-cb-series-sw33-english-international/) as a collaborative robot to help the experimenter get something. 

## Experimental Environment

### Using KinectV2 and Realsense D435f by ROS2 Humble
I installed the packages of KinectV2 and RealSense separately; you can see other projects in this repository. There are several issues when I tried to run two cameras side-by-side using ROS2, so here's how to fix it:

# UR3 ROS2 Humble Environment
[The Universal Robots ROS2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
## Installation
### Setup for real-time scheduling
I followed this [Tutorial Document](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/toc.html) to set up the UR3_ROS2_Humble environment.  
But there are some issues while doing the Compilation:  
Building the kernel using `make -j $(getconf _NPROCESSORS_ONLN) deb-pkg` would result in errors. I solved this by following GPT5's resolution. Here's the conversation [GPT5 solution](https://chatgpt.com/share/68ca7264-6fac-800f-898e-0cd488db0bb7).  
After compilation, in the [Setup user privileges to use real-time scheduling](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/real_time.html#setup-user-privileges-to-use-real-time-scheduling) section. You need to add the "@realtime" lines to the .conf file.
```
sudo nano /etc/security/limits.conf

# in the .conf file, scroll to the bottom, then add:(all lines should be annotated by #)
#@realtime soft rtprio 99
#@realtime soft priority 99
#@realtime soft memlock 102400
#@realtime hard rtprio 99
#@realtime hard priority 99
#@realtime hard memlock 102400
```
I didn't do the optional step:[Disable CPU speed scaling](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/real_time.html#optional-disable-cpu-speed-scaling)

### Setup URSim with Docker
  The Docker Hub that the tutorial provided is for e-series. Since I'm using UR3, which is the older one, I use this: [ursim_cb3](https://hub.docker.com/r/universalrobots/ursim_cb3)
  * **Run the image:**
    `docker run --rm -it universalrobots/ursim_cb3`
    This Docker command couldn't really view the user interface by clicking the link that the terminal provides, or using VNC applications.
      ```
      # VNC port: 5900
      # Web browser VNC port: 6080
      # ROBOT_MODEL: UR3
      docker run --rm -it -e ROBOT_MODEL=UR3 -p 5900:5900 -p 6080:6080 universalrobots/ursim_cb3
      ```
    The right way is running this Docker command and using this [link](http://localhost:6080/vnc.html?host=localhost&port=6080): `http://localhost:6080/vnc.html?host=localhost&port=6080`.  
    The robot model can be selected using the environment variable `ROBOT_MODEL`. The models' options available are `UR3`, `UR5`, and `UR10`. The default is UR5.
  * **External Control:**
    Follow the tutorial about [External Control](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/ursim_docker.html#external-control), and don't forget to specify the model of the robot.
    After this, you should be able to setup the `external_control` URCap and create a program as described in URCap setup guide.
    In my experiment, I use:
      ```
      docker run --rm -it -e ROBOT_MODEL=UR3 -p 5900:5900 -p 6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_cb3
      ```
  * **Network Setup:**
    To use the specific docker network, we can assign a static IP address to our URSim container:
    ```
    docker network create --subnet=192.168.56.0/24 ursim_net
    docker run --rm -it -e ROBOT_MODEL=UR3 -p 5900:5900 -p 6080:6080 \
      -v ${HOME}/.ursim/urcaps:/urcaps \
      -v ${HOME}/.ursim/programs:/ursim/programs \
      --net ursim_net --ip 192.168.56.101 \
      --name ursim universalrobots/ursim_cb3
    ```
    Although the tutorial says that the VNC web server should be available at `http://192.168.56.101:6080/vnc.html`, I still can't open it.
    So, I still use the original one [Link](http://localhost:6080/vnc.html?host=localhost&port=6080)
  * **Script Startup:**  
    I followed the tutorial step-by-step, so I didn't use the `start_ursim.sh` script.  
    There's another issue that the ROS2 command `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101` would result in an error about the IP address.  

    (A) **The Three Network Layers You Have: Currently Wrong IP Setup**  
      | Layer | What it is | Example IP range | Role |
      |-------|------------|------------------|------|
      | **Host (local machine)** | Ubuntu PC (where ROS 2 runs) | `192.168.1.x` (LAN) or `127.0.0.1` (loopback) | Runs ROS 2 |
      | **Docker container network** | Internal Docker bridge (ursim container) | `172.17.0.0/16` (e.g., host: `172.17.0.1`, container: `172.17.0.2`) | Runs URSim |
      | **VirtualBox Host-Only Adapter** | Host-only network for VMs | `192.168.56.0/24` | Not used by Docker; can conflict if reused |
    
      1. **Host to Docker container**  
        By default, Docker makes a bridge network:
        ```
        Host (Ubuntu)
        └── docker0 interface: 172.17.0.1
            └── container IP: 172.17.0.2
        ```
        * The host can reach the container at `172.17.0.2`.
        * The container can reach the host at `172.17.0.1`.
        * To expose container ports to the outside, we use `-p HOST_PORT:CONTAINER_PORT`.  
          `-p 30001:30001`: Any connection to `127.0.0.1:30001` on the host forwards to the container’s port `30001`.  
      2. **VirtualBox Host-Only Adapter**  
        VirtualBox installs an interface like this: `vboxnet0 → 192.168.56.1 (host)`
        This network is meant for VirtualBox VMs (not Docker).  
        If you assign a Docker container IP inside `192.168.56.x`, your host will send packets to VirtualBox’s network instead — causing the “cannot connect to robot” error.
      
    (B) **The Correct IP Roles in Your Setup**  
      | Component                  | What runs there           | Typical IP                                            | Explanation                                           |
      |----------------------------|---------------------------|-------------------------------------------------------|-------------------------------------------------------|
      | Host machine (Ubuntu)     | ROS 2, `ur_robot_driver`  | `127.0.0.1`, `172.17.0.1`, or LAN IP (e.g., `192.168.1.50`) | “localhost” means this machine itself |
      | Docker container (URSim)  | Robot simulator           | `172.17.0.2` (inside `docker0` network)               | The virtual robot |
      | VirtualBox                | (Not used for URSim)      | `192.168.56.x`                                        | Separate, unrelated network — should not be reused here |
      
    (C) **Connection Directions**  
      | Direction                     | Who connects                           | Destination IP/port                                       | Explanation                                                         |
      |-------------------------------|----------------------------------------|-----------------------------------------------------------|---------------------------------------------------------------------|
      | ROS 2 → URSim                | ROS driver connects to URSim           | `127.0.0.1:30001–30004` (if ports are mapped)             | Set `robot_ip:=127.0.0.1` in the launch command                     |
      | URSim → ROS 2 (reverse control) | External Control node connects back     | Host IP visible to container (`172.17.0.1` or LAN IP) port `50002` | Configure this in the URSim GUI External Control node               |

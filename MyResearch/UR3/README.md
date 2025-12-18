# <img src="https://github.com/XGangChen/CIRLab/blob/main/MyResearch/UR3/icon/Universal_robots_logo.svg" alt="Universal Robots Icon" width="30"> UR3 ROS2 Humble Environment
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
> I didn't do the optional step:[Disable CPU speed scaling](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/real_time.html#optional-disable-cpu-speed-scaling)  

---

### Launch the ur_robot_driver

- Network Setup:
  - UR control box:
    1. Connect the UR control box directly to the remote PC with an Ethernet cable.
    2. Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:
    ```
    IP address: 192.168.1.102
    Subnet mask: 255.255.255.0
    Default gateway: 192.168.1.1
    Preferred DNS server: 192.168.1.1
    Alternative DNS server: 0.0.0.0
    ```
  - Remote PC:
    1. Turn off all network devices except the “wired connection”.
    2. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:
    ```
    IPv4
    Manual
    Address: 192.168.1.101
    Netmask: 255.255.255.0
    Gateway: 192.168.1.1
    ```
  - Verify the connection from the PC with e.g., ping.
    ```
    ping 192.168.1.102
    ```
- Launch the driver:
  ```bash
  # Run the ROS2 command to see the robot in RViz:
  ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3 \
    robot_ip:=192.168.1.102 \
  ```

---

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
  
  ---
  
  * **External Control:**
    Follow the tutorial about [External Control](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/doc/installation/ursim_docker.html#external-control), and don't forget to specify the model of the robot.
    After this, you should be able to setup the `external_control` URCap and create a program as described in URCap setup guide.
    In my experiment, I use:
      ```
      docker run --rm -it -e ROBOT_MODEL=UR3 -p 5900:5900 -p 6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_cb3
      ```
  
  ---
  
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

  ---
  
  * **Network Configuration for URSim & ROS2 (Docker setup)**
    This project uses a "Split-IP" configuration to handle communication between the ROS 2 driver (running natively on Ubuntu) and the URSim robot (running inside a Docker container).
    - The Architecture
      Because Docker isolates the robot's network, we use two different IP addresses for the two-way communication:
      1. **Command Stream (ROS 2 → Robot):** Sent to `localhost` (mapped by Docker).
      2. **Feedback Stream (Robot → ROS 2):** Sent to the **Host's Real LAN IP**, allowing the container to "break out" and talk to Ubuntu.
    - Configuration Table
      | Direction / Role | Setting Location | Value / Command | Explanation |
      | :--- | :--- | :--- | :--- |
      | **1. Host IP Discovery** | Ubuntu Terminal | `hostname -I` | Run this to find your computer's real LAN IP (e.g., `192.168.1.50`). |
      | **2. Sending Commands**<br>(Forward Connection) | `ros2 launch` arg | `robot_ip:=127.0.0.1` | We target `localhost` because Docker forwards ports 30003/50002 from the container to the host. |
      | **3. Receiving Feedback**<br>(Reverse Connection) | `ros2 launch` arg | `reverse_ip:=<YOUR_LAN_IP>` | Tells the ROS driver to listen on your actual Wi-Fi/Ethernet interface, not just localhost. |
      | **4. Robot Configuration**<br>(Inside URSim) | **Installation** → **URCaps** → **External Control** | **Host IP:** `<YOUR_LAN_IP>`<br>**Custom Port:** `50002` | Tells the virtual robot to send data back to your Ubuntu machine's LAN IP. |
  * Quick Start Commands
    - Find your IP
      ```bash
      hostname -I
      # Example output: 192.168.105.45
      ```
    - Launch ROS 2 Driver: Replace `192.168.105.45` with the IP found above.
      ```bash
      ros2 launch ur_robot_driver ur_control.launch.py \
      ur_type:=ur3 \
      robot_ip:=127.0.0.1 \
      reverse_ip:=192.168.105.45 \
      launch_rviz:=false
      ```
    - Configure URSim (GUI)
      1. Open [URSim](http://localhost:6080/vnc.html?host=localhost&port=6080)
      2. Set Host IP to `192.168.105.45`, port as default `50002`
      3. Try the robot play bottom.
         You should see the log in your driver terminal like bellow:
         ```
         [ur_ros2_control_node-1] [INFO] [1765353483.561948491] [UR_Client_Library:]: Robot requested program
         [ur_ros2_control_node-1] [INFO] [1765353483.562008749] [UR_Client_Library:]: Sent program to robot
         [ur_ros2_control_node-1] [INFO] [1765353501.582142255] [UR_Client_Library:]: Robot connected to reverse interface. Ready to receive control commands.
         # After you press the stop bottom:
         [ur_ros2_control_node-1] [INFO] [1765353504.821915320] [UR_Client_Library:]: Connection to reverse interface dropped.
         ```
  
  ---
  
  * **The Final Command I Use**  
    Since my real-world experimental setup, the UR3 has a ROBOTIQ 2F-85 Gripper installed on it, I installed the ROBOTIQ Gripper URCap file into the URSim while I created the container.
    However, the ROBOTIQ Gripper URCap can only show the commands in URSim because URSim couldn't detect the gripper in the simulation world.
      ```
      # Prepare host folders
      mkdir -p ${HOME}/.ursim/urcaps
      mkdir -p ${HOME}/.ursim/programs
      
      cp /path/to/Robotiq_Grippers-<version>.urcap ${HOME}/.ursim/programs  # You have to install the gripper URCap yourself.
      URCAP_VERSION=1.0.5 # latest version as if writing this
      curl -L -o ${HOME}/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar \
        https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar

      # Replace `--rm` can keep the container existing
      docker run -it \ 
        -e ROBOT_MODEL=UR3 \
        -p 5900:5900 \
        -p 6080:6080 \
        -p 29999:29999 \
        -p 30001:30001 \
        -p 30002:30002 \
        -p 30003:30003 \
        -p 30004:30004 \
        -v ${HOME}/.ursim/urcaps:/urcaps \
        -v ${HOME}/.ursim/programs:/ursim/programs \
        --name ursim universalrobots/ursim_cb3

      # After the creation, you just need to start the container, then click the link for URSim VNC.
      docker start ursim

      # Run the ROS2 command to see the robot in RViz:
      ros2 launch ur_robot_driver ur_control.launch.py \
      ur_type:=ur3 \
      robot_ip:=127.0.0.1 \
      reverse_ip:=192.168.105.45
      ```

# MoveIt  
  I followed the tutorial document: [MoveIt](https://moveit.picknik.ai/main/index.html)  
  If there's any difference with my device, I will show and describe it below.  
  
## Getting Started  
### Install ROS2 and colcon  
  I've installed the ROS2 Humble and colcon before. Also, `rosdep` and `vsctool` packages have been installed.  
  To verify whether it is installed in your device, try the commands below:  

  ---
  
  * Are the APT packages installed?
    ```
    apt-cache policy python3-colcon-common-extensions python3-colcon-mixin
    ```
    Logs from my terminal:
    ```
    python3-colcon-common-extensions:
      Installed: 0.3.0-100
      Candidate: 0.3.0-100
      Version table:
     *** 0.3.0-100 500
            500 http://packages.ros.org/ros2/ubuntu jammy/main amd64 Packages
            500 http://packages.ros.org/ros2/ubuntu jammy/main i386 Packages
            100 /var/lib/dpkg/status
    python3-colcon-mixin:
      Installed: 0.2.3-100
      Candidate: 0.2.3-100
      Version table:
     *** 0.2.3-100 500
            500 http://packages.ros.org/ros2/ubuntu jammy/main amd64 Packages
            500 http://packages.ros.org/ros2/ubuntu jammy/main i386 Packages
            100 /var/lib/dpkg/status
    ```

  ---
  
  * Does the `colcon` + `mixin` command work?
    ```
    command -v colcon
    colcon --version
    colcon mixin -h
    ```
    Logs from my terminal:
    ```
    /usr/bin/colcon
    usage: colcon [-h] [--log-base LOG_BASE] [--log-level LOG_LEVEL]
                  {build,extension-points,extensions,graph,info,list,metadata,mixin,test,test-result,version-check} ...
    colcon: error: argument verb_name: invalid choice: '--version' (choose from 'build', 'extension-points', 'extensions', 'graph', 'info', 'list', 'metadata', 'mixin', 'test', 'test-result', 'version-check')
    usage: colcon mixin [-h] {add,list,remove,show,update} ...
    
    Manage CLI mixins.
    
    options:
      -h, --help            show this help message and exit
    
    colcon mixin verbs:
      add                   Add the URL of a repository index
      list                  List all repositories and their mixin
      remove                Remove a repository from the list of indexes
      show                  Show available mixins and their mapping
      update                Update the mixin from the repository indexes
    
      {add,list,remove,show,update}
                            call `colcon mixin VERB -h` for specific help
    ```

  ---
  
  * Is the “default” mixin repository added and updated?
    ```
    colcon mixin list
    # If there is nothing turns up:
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default
    # Verify again:
    ls -R ~/.colcon/mixin
    cat ~/.colcon/mixin/index.yaml
    ```

# <img src="https://assets.robotiq.com/website-assets/support_documents/document/User_Interface_PDF_20190724.pdf" alt="ROBOTIQ Icon" width="30">ROBOTIQ Gripper
   

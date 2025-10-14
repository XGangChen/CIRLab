# Cobot-Human Collision Avoidance (UR3 + Kinect V2 + RealSense D435f)  

> **Goal:** Design a safety algorithm for the Human-Robot workspace to avoid collisions. The experimental environment setup is using [UR3](https://www.universal-robots.com/download/manuals-cb-series/user/ur3/33/user-manual-ur3-cb-series-sw33-english-international/) as a collaborative robot, using Kinect V2 to detect human hand's 3D position, and using RealSense D435f to do human arm's pose estimation.

## Experimental Environment

### Using KinectV2 and Realsense D435f by ROS2 Humble
I installed the packages of KinectV2 and RealSense separately; you can see other projects in this repository. There are several issues when I tried to run two cameras side-by-side using ROS2, so here’s a clean, practical checklist + ready-to-run examples so you can run Kinect v2 (kinect2_bridge) and RealSense (realsense2_camera) side-by-side:
<details>
  <summary>
    test
  </summary>
[Troubleshooting](https://chatgpt.com/share/68de2bf1-5fbc-800f-8a53-534d66b82783)
</details>

# Cobot–Human Collision Avoidance (UR3 + Kinect V2 + RealSense D435f)

> **Goal:** Detect the UR3 end-effector pose, the human hand 3D position, and the human arm pose, then compute minimum separation and apply speed scaling/stop to avoid contact.

---

## 1) Repository Structure

```text
cobot_human_safety/
├─ README.md                           # This file
├─ LICENSE
├─ .gitignore
├─ ros2_ws/                            # ROS 2 workspace (Humble)
│  ├─ src/
│  │  ├─ ur3_bringup/                  # UR3 (CB3) bringup, TF, TCP config
│  │  ├─ robotiq_2f85_bringup/         # Gripper driver/wrapper, TCP offset
│  │  ├─ kinect2_bridge/               # Kinect V2 ROS2 bridge (libfreenect2)
│  │  ├─ realsense_bringup/            # D435f via realsense2_camera
│  │  ├─ people_perception/            # Perception meta-package
│  │  │  ├─ hand_tracker/              # 3D hand detection from Kinect V2
│  │  │  ├─ arm_pose_estimator/        # Human arm keypoints from D435f
│  │  │  ├─ tf_fusion/                 # Fuse multi-camera poses → common frame
│  │  ├─ collision_avoidance/          # Distance/zone logic + safety actions
│  │  │  ├─ collision_monitor/         # Computes min distance, publishes status
│  │  │  ├─ safety_supervisor/         # Speed scale / stop via RTDE/URScript
│  │  ├─ visualization_tools/          # RViz configs, markers
│  │  └─ utils_common/                 # Msgs, srvs, common math/time utils
│  ├─ install/                         # (colcon) generated
│  ├─ build/                           # (colcon) generated
│  └─ log/                             # (colcon) generated
├─ launch/                             # Top-level orchestration launch files
│  ├─ bringup_ur3_real.launch.py
│  ├─ bringup_ursim.launch.py
│  ├─ bringup_kinect2.launch.py
│  ├─ bringup_realsense.launch.py
│  ├─ perception.launch.py             # Hand + arm pose + TF fusion
│  ├─ safety_runtime.launch.py         # Collision monitor + supervisor
│  └─ full_system.launch.py            # One-shot end-to-end bringup
├─ config/
│  ├─ frames.yaml                      # Canonical frame names & parents
│  ├─ safety.yaml                      # Zones, thresholds, timeouts, actions
│  ├─ ur3/
│  │  ├─ controller.yaml               # Joint controllers (if used)
│  │  ├─ tcp_offset.yaml               # ROBOTIQ 2F-85 TCP → tool0 transform
│  │  └─ kin_params.yaml               # DH or kinematic chain params (if custom)
│  ├─ kinect2/
│  │  ├─ intrinsics.yaml
│  │  └─ extrinsics_to_base.yaml       # T_base_kinect
│  └─ realsense/
│     ├─ intrinsics.yaml
│     └─ extrinsics_to_base.yaml       # T_base_realsense
├─ calibration/
│  ├─ hand_eye/                        # Hand–eye or camera-to-base results
│  ├─ patterns/                        # Charuco/AprilTag boards used
│  └─ procedures.md                    # Step-by-step extrinsic/intrinsic guide
├─ scripts/
│  ├─ record_all.bash                  # ros2 bag record key topics
│  ├─ dump_tf_tree.bash
│  └─ sanity_checks.py                 # Quick checks for frames/topics
├─ docs/
│  ├─ system_overview.md
│  ├─ safety_cases.md
│  ├─ latency_budget.md
│  └─ troubleshooting.md
└─ bags/                                # Optional: sample/test rosbag files
```

---

## 2) System Overview

### 2.1 Data Flow (high level)

1. **UR3 (CB3) + Robotiq 2F-85** → joint states & `tool0/ee_link` TF.
2. **Kinect V2** → depth + RGB → **Hand Tracker** → 3D hand point(s) in `kinect_optical` → TF to `base`.
3. **RealSense D435f** → depth + RGB → **Arm Pose Estimator** → arm keypoints (shoulder/elbow/wrist) in `realsense_optical` → TF to `base`.
4. **TF Fusion** → unify all poses in **robot base** (`ur3/base`) or **world** (`world`).
5. **Collision Monitor** → compute min distance between **UR3 tool** and **human joints**.
6. **Safety Supervisor** → apply **speed scaling** or **protective stop** via RTDE/URScript when thresholds are crossed.

### 2.2 Coordinate Frames (REP-103 compliant)

* **Robot:** `ur3/base` → `...` → `tool0` (or `ee_link`) → `tcp` (Robotiq fingertip center).
* **Kinect V2:** `kinect_link` → `kinect_rgb_optical` / `kinect_ir_optical`.
* **RealSense D435f:** `realsense_link` → `realsense_color_optical` / `realsense_depth_optical`.
* **Human:** `human_hand`, `human_elbow`, `human_shoulder` (as TFs or custom msg frames).
* **World/Base:** Choose **`world`** (static) or **`ur3/base`** as the common root.

All camera-to-base extrinsics are stored in `config/*/extrinsics_to_base.yaml`.

---

## 3) Hardware & Software

### 3.1 Hardware

* **Robot:** UR3 (CB3 controller), **Robotiq 2F-85** gripper.
* **Cameras:** Kinect V2, RealSense D435f.
* **Compute:** Ubuntu 22.04 + CUDA-capable GPU (optional acceleration).

### 3.2 Core Software

* **ROS 2 Humble**
* **UR Interface:**

  * For **real UR3 CB3**: External Control URCap + RTDE/URScript bridge (ROS 2 wrapper in `ur3_bringup`).
  * For **URSim** (optional): `ursim` Docker or native install + ROS 2 bridge.
* **Robotiq 2F-85:** ROS 2 driver/wrapper; publishes gripper state & sets TCP offset.
* **Kinect V2:** `libfreenect2` + `kinect2_bridge` (ROS 2 port).
* **RealSense:** `realsense2_camera` ROS 2 node.
* **Perception:** PyTorch/ONNX/OpenVINO runtime for keypoints; custom ROS 2 nodes.
* **Visualization:** RViz2.

> Note: CB3 support in ROS 2 requires the External Control URCap and a network path for RTDE. Keep controller firmware consistent with your URCap.

---

## 4) Build & Setup

### 4.1 ROS 2 Workspace

```bash
# 1) Create workspace
mkdir -p ~/cobot_human_safety/ros2_ws/src
cd ~/cobot_human_safety/ros2_ws

# 2) Clone packages (examples)
# git clone ... ur3_bringup
# git clone ... robotiq_2f85_bringup
# git clone ... kinect2_bridge (ROS2 port)
# git clone ... realsense_bringup (wraps realsense2_camera)
# git clone ... people_perception
# git clone ... collision_avoidance

# 3) Install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# 4) Build
colcon build --symlink-install
source install/setup.bash
```

### 4.2 Network & UR3 (CB3)

* Robot and host on the **same subnet**.
* Set UR3 **External Control** program on the teach pendant: point to host IP + RTDE port.
* Verify dashboard (e.g., 29999), primary/secondary client ports if used.

### 4.3 Cameras

* **Kinect V2:** Install `libfreenect2`; verify with the viewer; then launch `kinect2_bridge`.
* **RealSense D435f:** Install `realsense2_camera` (and librealsense); verify depth/color streams.

### 4.4 Calibration (must-do)

1. **Intrinsics** for both cameras (factory or re-calibrate).
2. **Extrinsics** camera→base using AprilTag/Charuco board. Save to:

   * `config/kinect2/extrinsics_to_base.yaml`
   * `config/realsense/extrinsics_to_base.yaml`
3. **TCP offset** of Robotiq: update `config/ur3/tcp_offset.yaml` (finger center).

---

## 5) Launching

### 5.1 Individual Bringup

```bash
# UR3 (real)
ros2 launch launch/bringup_ur3_real.launch.py

# or URSim
ros2 launch launch/bringup_ursim.launch.py

# Kinect V2
ros2 launch launch/bringup_kinect2.launch.py

# RealSense D435f
ros2 launch launch/bringup_realsense.launch.py
```

### 5.2 Perception & Fusion

```bash
# Hand + Arm + TF fusion
ros2 launch launch/perception.launch.py
```

### 5.3 Safety Runtime

```bash
# Distance computation + supervisor actions
ros2 launch launch/safety_runtime.launch.py

# Or everything at once
ros2 launch launch/full_system.launch.py
```

---

## 6) Topics, TFs, and Messages

### 6.1 Key Topics (examples)

* `/joint_states` (sensor_msgs/JointState) — UR3 joints
* `/tf` and `/tf_static` — full frame tree
* `/kinect2/depth/image_raw`, `/kinect2/color/image_raw`
* `/realsense/depth/image_raw`, `/realsense/color/image_raw`
* `/hand_tracker/hand_points` (geometry_msgs/PointStamped[])
* `/arm_pose/keypoints` (custom msg: `ArmKeypoints`)
* `/collision_monitor/status` (custom msg: `SafetyStatus`)
* `/safety/speed_scale_cmd` (std_msgs/Float32)
* `/safety/stop_cmd` (std_msgs/Bool)

### 6.2 Representative Custom Messages

```text
# people_perception/ArmKeypoints.msg
Header header
geometry_msgs/PointStamped shoulder
geometry_msgs/PointStamped elbow
geometry_msgs/PointStamped wrist
float32 confidence

# collision_avoidance/SafetyStatus.msg
Header header
float32 min_distance_m
string nearest_pair           # e.g., "tool0 ↔ wrist"
string zone                   # NORMAL | WARNING | PROTECTIVE
float32 applied_speed_scale   # 1.0..0.0
```

---

## 7) Parameters & Zones

All runtime safety thresholds are defined in `config/safety.yaml`:

```yaml
# config/safety.yaml
common_frame: ur3/base     # or world

thresholds:
  warn_distance_m: 0.45    # enter caution zone
  stop_distance_m: 0.25    # protective stop
  hysteresis_m: 0.03       # avoid chattering

speed_scaling:
  enabled: true
  min_scale: 0.1           # never go below this unless stop is required
  function: linear         # linear | quadratic | custom

supervisor:
  action_on_stop: 'rtde_stop'  # rtde_stop | urscript_stop
  recovery: 'manual'           # manual | auto_on_clear
  debounce_ms: 120

targets:
  robot_frames: [tool0]
  human_frames: [human_hand, human_wrist, human_elbow]
```

---

## 8) Algorithms (short notes)

* **Hand Tracker (Kinect):** depth clustering + heuristics OR learned detector; outputs filtered 3D points.
* **Arm Pose Estimator (RealSense):** RGB-based keypoint model → depth back-projection for metric 3D.
* **TF Fusion:** `tf2` transforms all outputs to `common_frame`.
* **Distance:** pairwise minimum (robot frames × human frames); low-pass filter to reduce jitter.
* **Actions:** speed scaling proportional to distance; hard stop inside protective zone.

---

## 9) Visualization (RViz2)

* Pre-made RViz config under `visualization_tools/rviz/*.rviz` shows:

  * Robot model & `tool0` TF
  * Camera frustums
  * Human keypoints & lines (shoulder–elbow–wrist)
  * Distance text/markers & colorized zones

---

## 10) Testing & Demos

```bash
# Record a quick demo
./scripts/record_all.bash

# Dump the TF tree to check frame connectivity
./scripts/dump_tf_tree.bash

# Run unit tests (if provided)
colcon test --packages-select collision_avoidance people_perception
```

---

## 11) Troubleshooting (quick)

* **No TF to base:** Check `extrinsics_to_base.yaml` and static transform publishers.
* **Distance = NaN:** Missing depth or invalid keypoint confidence; enable fallbacks.
* **CB3 not responding:** Verify External Control URCap, host IP, firewall, and RTDE endpoint.
* **Two cameras drift:** Re-run extrinsics; ensure both see the calibration board simultaneously.

---

## 12) Safety Notes

* This is **assistive** software; you must validate with your risk assessment (ISO/TS 15066 context).
* Always test at low speeds first; verify zone boundaries with a dummy target before using with people.

---

## 13) Acknowledgments

* Universal Robots, Robotiq, Intel RealSense, libfreenect2, ROS 2 community.

## 14) License

* Choose an OSI-approved license appropriate for your project (e.g., Apache-2.0 or BSD-3-Clause).

---

## 15) TF2 Coordination & Calibration (Humble)

This section gives you **ready-to-run TF2 setup** for UR3 (CB3), Kinect V2, and RealSense D435f. It includes canonical frame names, a single launch file that publishes the static extrinsics, example YAML, and verification commands.

### 15.1 Canonical Frame Names

* **Robot (UR3 CB3)**: `ur3/base` → `...` → `tool0` → `tcp` (Robotiq 2F-85 finger-center)
* **Kinect V2**: `kinect2_link` → `kinect2_rgb_optical_frame` / `kinect2_ir_optical_frame`
* **RealSense D435f**: `realsense_link` → `realsense_color_optical_frame` / `realsense_depth_optical_frame`
* **World (optional)**: `world` (only if you need a lab/world origin distinct from the robot base)

> Notes
>
> * Use **`ur3/base` as the common root** unless you have a strong reason to use `world`.
> * Camera drivers already publish their internal chains (e.g., `*_link` → `*_optical_frame`). **Do not** duplicate those.
> * Optical frames conform to REP-103 (X right, Y down, Z forward).

### 15.2 Extrinsics YAML

Create `config/extrinsics.yaml` to hold all static transforms you want to publish from the robot base to each device and the TCP offset.

```yaml
# config/extrinsics.yaml
base_frame: ur3/base

cameras:
  kinect2:
    parent: ur3/base
    child: kinect2_link
    # Replace with your measured values (meters / radians)
    xyz: [0.45, -0.20, 1.20]
    rpy: [0.0, 0.5236, 3.1416]
  realsense:
    parent: ur3/base
    child: realsense_link
    xyz: [0.30, 0.40, 1.10]
    rpy: [0.0, 0.0, 1.5708]

# Robotiq 2F-85 TCP offset relative to UR's tool flange (`tool0`)
tcp:
  parent: tool0
  child: tcp
  xyz: [0.000, 0.000, 0.155]
  rpy: [0.0, 0.0, 0.0]

# Optional: publish world→base if you maintain a separate global origin
world:
  enabled: false
  parent: world
  child: ur3/base
  xyz: [0.0, 0.0, 0.0]
  rpy: [0.0, 0.0, 0.0]
```

> **How to obtain these numbers**: Use an AprilTag/Charuco board visible to each camera and measured relative to `ur3/base`. Solve camera→base with PnP (OpenCV), `apriltag_ros`, or a custom calibration node, then export as translation (m) and roll–pitch–yaw (rad).

### 15.3 TF Static Publisher Launch

Create `launch/tf_tree.launch.py` that reads the YAML and spawns **tf2_ros/static_transform_publisher** nodes.

```python
# launch/tf_tree.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os, yaml


def make_stf(xyz, rpy, parent, child, name_suffix):
    x,y,z = xyz
    r,p,yaw = rpy
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'stf_{name_suffix}',
        # Euler version: x y z roll pitch yaw parent child
        arguments=[str(x), str(y), str(z), str(r), str(p), str(yaw), parent, child],
        output='screen'
    )


def generate_launch_description():
    cfg_arg = DeclareLaunchArgument(
        'extrinsics',
        default_value=os.path.join(os.getcwd(), 'config', 'extrinsics.yaml'),
        description='Path to extrinsics YAML.'
    )
    cfg_path = LaunchConfiguration('extrinsics')

    # We need to load at generate time; do a small helper node list builder
    nodes = []
    try:
        # Resolve the path at runtime
        path = os.path.expanduser(str(cfg_path.perform({})))
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # Cameras
        for key, cam in data.get('cameras', {}).items():
            nodes.append(make_stf(cam['xyz'], cam['rpy'], cam['parent'], cam['child'], f'{cam["parent"].replace("/","_")}_to_{cam["child"].replace("/","_")'))

        # TCP offset
        tcp = data.get('tcp', None)
        if tcp:
            nodes.append(make_stf(tcp['xyz'], tcp['rpy'], tcp['parent'], tcp['child'], f'{tcp["parent"].replace("/","_")}_to_{tcp["child"].replace("/","_")'))

        # Optional world→base
        world = data.get('world', {})
        if world.get('enabled', False):
            nodes.append(make_stf(world['xyz'], world['rpy'], world['parent'], world['child'], f'{world["parent"].replace("/","_")}_to_{world["child"].replace("/","_")'))

    except Exception as e:
        # If YAML load fails, print a hint; LaunchDescription can still return empty
        print(f"[tf_tree.launch] Failed to load extrinsics YAML: {e}")

    return LaunchDescription([cfg_arg] + nodes)
```

> If you prefer the quaternion variant of `static_transform_publisher`, replace the Euler arguments with `x y z qx qy qz qw parent child` accordingly.

### 15.4 Bringing Up the TF Tree

```bash
# 1) Source your workspace
cd ~/cobot_human_safety/ros2_ws
source install/setup.bash

# 2) Start robot & cameras (examples; use your actual bringups)
ros2 launch launch/bringup_ur3_real.launch.py &
ros2 launch launch/bringup_kinect2.launch.py &
ros2 launch launch/bringup_realsense.launch.py &

# 3) Publish static extrinsics & TCP
ros2 launch launch/tf_tree.launch.py extrinsics:=config/extrinsics.yaml
```

### 15.5 Verification

Install tools if needed:

```bash
sudo apt install -y ros-humble-tf2-tools
```

Now verify:

```bash
# Print the TF between frames
ros2 run tf2_ros tf2_echo ur3/base tcp
ros2 run tf2_ros tf2_echo ur3/base kinect2_link
ros2 run tf2_ros tf2_echo ur3/base realsense_link

# Generate a graph (frames.pdf may be created in CWD)
ros2 run tf2_tools view_frames

# Visual check
rviz2  # Add RobotModel, TF, and camera frustums/point clouds
```

### 15.6 Human Frames Broadcasting (example)

If your perception nodes output 3D keypoints, you can broadcast them into TF for unified distance checks.

```python
# people_perception/nodes/human_tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from your_msgs.msg import ArmKeypoints  # shoulder, elbow, wrist as PointStamped

class HumanTF(Node):
    def __init__(self):
        super().__init__('human_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(ArmKeypoints, '/arm_pose/keypoints', self.cb, 10)
        self.common_frame = self.declare_parameter('common_frame', 'ur3/base').get_parameter_value().string_value

    def as_tf(self, name, ps):
        t = TransformStamped()
        t.header = ps.header
        t.header.frame_id = self.common_frame
        t.child_frame_id = name
        t.transform.translation.x = ps.point.x
        t.transform.translation.y = ps.point.y
        t.transform.translation.z = ps.point.z
        t.transform.rotation.w = 1.0
        return t

    def cb(self, msg):
        # Assume upstream already transformed into common_frame
        to_send = [
            self.as_tf('human_shoulder', msg.shoulder),
            self.as_tf('human_elbow', msg.elbow),
            self.as_tf('human_wrist', msg.wrist),
        ]
        self.br.sendTransform(to_send)

def main():
    rclpy.init()
    rclpy.spin(HumanTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Launch snippet:

```python
# launch/perception.launch.py (append)
Node(
  package='people_perception',
  executable='human_tf_broadcaster',
  name='human_tf_broadcaster',
  parameters=[{'common_frame': 'ur3/base'}]
)
```

### 15.7 Common Pitfalls

* **Wrong parent/child order**: Always `parent` then `child` in the CLI arguments.
* **Mixing degrees/radians**: The example uses **radians** for RPY.
* **Overlapping publishers**: Ensure only one node publishes a given TF.
* **Disconnected trees**: If `view_frames` shows multiple trees, your static extrinsics are missing or misnamed.
* **Optical vs link frames**: Publish **base→*_link** only; let the camera driver handle link→optical.

---

# Function Explanation
I'll explain what each script works for. The Python scripts need to be paired with ROS2 commands and tools.  
There's the [ROS2](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence/ROS2) folder under [RealSense](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence) in this repository.  
We need to launch the camera before running these scripts. Please check it out.  

> I run all the Python scripts in Python3.10-venv. Since we're using ROS, do not use Conda to avoid unnecessary troubles. My software setup is Ubuntu 22.04 LTS and ROS2 Humble, so we're using Python3.10. 

# Table of Contents
* [Camera](#-camera)
  * [D435f_mediapipe_wrist_debug.py](#-d435f_mediapipe_wrist_debugpy)
  * [D435f_cam_info_tf_check.py](#-d435f_cam_info_tf_checkpy)
  * [D435f_wrist_to_base.py](#-d435f_wrist_to_basepy)
* [UR3](#ur3)
  * [D435f_UR_human_detect.py](#-d435f_ur_human_detectpy)
  * [D435f_UR_pose_ROSnode.py](#-d435f_ur_pose_rosnodepy)
* [ROS2 Launch & Tools](#-ros2-launch--tools)
  * [Launch the Camera](#launch-the-camera)
  * [TFtree2 Environment Setup](#tftree2-environment-setup)
  * [Using RQT to See the Images](#using-rqt-to-see-the-images)
  * [Using RViz2 to Get the Whole Environment](#using-rviz2-to-get-the-whole-environment)



---

# <img src="https://www.intelrealsense.com/wp-content/uploads/2020/09/intel-realsense-logo-360px.png" alt="RealSense Logo" height="25"> Camera
## <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> D435f_mediapipe_wrist_debug.py

This ROS 2 node subscribes to a color stream, an **aligned depth-to-color** stream, and color **CameraInfo**, runs **MediaPipe Hands** to detect up to **two** hands, overlays wrist landmarks and metrics (depth `z` and Euclidean range `R`) on the color image, and publishes a debug image you can view in RViz or `rqt_image_view`.

> This node is **for visualization/debug**: it does **not** publish 3D points or TF. Use it to verify hands, depth alignment, and camera intrinsics.

### Features

- **Hands detection (max 2)** via MediaPipe with lightweight tracking.
- **Robust depth**: 7×7 patch median (ignores 0s) at the wrist pixel.
- **Deprojection** using **CameraInfo** (pinhole intrinsics) to compute camera-frame `(X, Y, Z)` and range `R = √(X²+Y²+Z²)`.
- **On-image overlays**: wrist pixel, hand connections, handedness label (“Left/Right”), `z` (meters), and `R` (meters).
- **Guardrails**:
  - Accepts depth in `uint16` (mm → m) or `float32` (m).
  - Warns if depth `frame_id` isn’t the expected **color optical frame** (misalignment hint).
  - Tolerant to missing depth/intrinsics (still draws landmarks, just omits numbers).

### Topics & Parameters

- **Parameters (declare via ROS 2 params)**  
  - `color_topic` (string, default: `/camera/cam2/color/image_raw`)
  - `depth_topic` (string, default: `/camera/cam2/aligned_depth_to_color/image_raw`)
  - `info_topic`  (string, default: `/camera/cam2/color/camera_info`)

- **Subscribed**
  - **Color**: `<color_topic>` (`sensor_msgs/Image`, BGR8 expected)
  - **Depth**: `<depth_topic>` (`sensor_msgs/Image`, `uint16` mm or `float32` m)
  - **Intrinsics**: `<info_topic>` (`sensor_msgs/CameraInfo`)

> **QoS**: Uses `QoSPresetProfiles.SENSOR_DATA` (BEST_EFFORT, KEEP_LAST).

- **Published**
  - `/wrist/debug_image` (`sensor_msgs/Image`, BGR8) — the annotated color image.

### Running

- Quick test (direct Python)
  ```bash
  python3 mediapipe_wrist_debug.py \
    --ros-args \
    -p color_topic:=/camera/cam2/color/image_raw \
    -p depth_topic:=/camera/cam2/aligned_depth_to_color/image_raw \
    -p info_topic:=/camera/cam2/color/camera_info
  ```
  > Make sure the camera has been launched and the TF tree has been set up.

### How it works (algorithm)

1. Convert color frame (BGR) → RGB, run **MediaPipe Hands** (`max_num_hands=2`).
2. For each detected hand:
   - Draw landmarks and connections.
   - Get wrist landmark (index **0**), map normalized coordinates to pixel `(u, v)`.
   - If depth & intrinsics available:
     - Extract **7×7** patch around `(u, v)`, discard zeros, take **median** as **`z`** (meters).
     - Deproject with pinhole model to `(X, Y, Z)` and compute **`R = √(X²+Y²+Z²)`**.
     - Overlay text: handedness + `z` and `R` (m).
   - If missing depth/intrinsics, overlay pixel and handedness only.

---

## <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> D435f_cam_info_tf_check.py

This tiny ROS 2 tool verifies **camera intrinsics** (from `CameraInfo`) and the **TF transform** between two frames. It prints the focal lengths and principal point **once**, then reports the transform **every second** so you can confirm your extrinsics are being published correctly.

### What it does

1. **Subscribes** to a `sensor_msgs/CameraInfo` topic (default **`/camera/cam2/color/image_raw/theora`** — you will almost certainly change this to your camera’s `.../camera_info` topic).
2. **Parses intrinsics** from the `K` matrix:
   - `fx = K[0,0]`, `fy = K[1,1]`, `cx = K[0,2]`, `cy = K[1,2]`.
3. **Looks up TF** from **`target_frame`** (default `platform_base`) to **`source_frame`** (default `cam2_color_optical_frame`) once per second.
4. **Logs** the translation `(x,y,z)` and quaternion `(x,y,z,w)` when available; otherwise, prints a warning until TF exists.

> This node is read-only: it does not publish topics, only logs to the console.

### Parameters (ROS 2)

| Name | Type | Default | Description |
|---|---|---|---|
| `info_topic` | string | `/camera/cam2/color/image_raw/theora` | The **CameraInfo** topic to read. Change this to your camera’s `.../camera_info`. |
| `source_frame` | string | `cam2_color_optical_frame` | The **child/source** frame (e.g., color optical frame). |
| `target_frame` | string | `platform_base` | The **parent/target** frame (e.g., robot/world base). |

> **QoS**: The subscription is created with `QoSProfile(depth=10)`. If you see no CameraInfo arriving, switch to `QoSPresetProfiles.SENSOR_DATA` to match camera drivers (see “Improvements” below).

### Running

```bash
python3 cam_info_tf_check.py \
  --ros-args \
  -p info_topic:=/camera/cam2/color/camera_info \
  -p source_frame:=cam2_color_optical_frame \
  -p target_frame:=platform_base
```

---

## <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> D435f_wrist_to_base.py

This node detects **hands** with **MediaPipe**, samples **aligned depth** at the **wrist** pixel, **deprojects** to 3D using the color camera intrinsics, transforms the point to a chosen **base frame** via **TF2**, and publishes both left/right `PointStamped` and RViz `Marker`(s). It also publishes an annotated **debug image**.

> Script: `wrist_to_base.py` (Python, ROS 2 rclpy).

### What it does

1. **Subscribe** (SENSOR_DATA QoS):
   - Color image: `color_topic` (default `/camera/cam2/color/image_raw`)
   - Aligned depth-to-color: `depth_topic` (default `/camera/cam2/aligned_depth_to_color/image_raw`)
   - Color `CameraInfo`: `info_topic` (default `/camera/cam2/color/camera_info`)
2. Run **MediaPipe Hands** (max 2). For each hand:
   - Extract **wrist** landmark (index 0) → pixel `(u, v)`.
   - Read a **robust depth** (meters) at `(u, v)` from an **adaptive window** (7, 11, 15) using **median of non‑zero** samples.
   - **Deproject** `(u, v, z)` → `(Xc, Yc, Zc)` from `CameraInfo` intrinsics.
   - **Transform** to base frame via TF2 (`target_frame` ← `source_frame`).
   - **Publish**:
     - `PointStamped` on `/wrist_left_point_base` or `/wrist_right_point_base`
     - A **sphere Marker** (0.12 m) and a `MarkerArray`
3. **Overlay** labels and depth on the debug image and publish to `/wrist/debug_image`.

The node warns once if the depth’s `frame_id` differs from `source_frame` (likely **not aligned to color**) and if TF is missing.

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `color_topic` | string | `/camera/cam2/color/image_raw` | Color image (BGR8 expected). |
| `depth_topic` | string | `/camera/cam2/aligned_depth_to_color/image_raw` | Depth aligned to **color** (uint16 mm or float32 m). |
| `info_topic` | string | `/camera/cam2/color/camera_info` | Color `CameraInfo` for intrinsics. |
| `source_frame` | string | `cam2_color_optical_frame` | Camera **optical** frame. |
| `target_frame` | string | `platform_base` | Base/world frame to transform into. |
| `min_det_conf` | double | `0.30` | MediaPipe detection confidence. |
| `min_trk_conf` | double | `0.30` | MediaPipe tracking confidence. |

**QoS:** Subscribers use `QoSPresetProfiles.SENSOR_DATA` (matches most camera drivers). Publishers use depth=10.

### Topics

**Subscribed**  
- `<color_topic>` — `sensor_msgs/Image` (BGR8)  
- `<depth_topic>` — `sensor_msgs/Image` (`uint16` mm or `float32` m)  
- `<info_topic>` — `sensor_msgs/CameraInfo`

**Published**  
- `/wrist_left_point_base`, `/wrist_right_point_base` — `geometry_msgs/PointStamped` (in `target_frame`)  
- `/wrist_marker` — `visualization_msgs/Marker` (one per hand)  
- `/wrist_marker_array` — `visualization_msgs/MarkerArray`  
- `/wrist/debug_image` — `sensor_msgs/Image` (BGR8) annotated

### Running

```bash
python3 wrist_to_base.py \
  --ros-args \
  -p color_topic:=/camera/cam2/color/image_raw \
  -p depth_topic:=/camera/cam2/aligned_depth_to_color/image_raw \
  -p info_topic:=/camera/cam2/color/camera_info \
  -p source_frame:=cam2_color_optical_frame \
  -p target_frame:=platform_base
```

### How it works (key internals)

- **Depth robustness** — `robust_depth_at(u,v,depth,h,w)` tries windows **7, 11, 15**; takes the **median of non‑zero** pixels; returns `(z_meters, k_used)`; if none valid, returns `NaN`.  
- **Deprojection** — Using `fx, fy, cx, cy` from `CameraInfo`:
  \[
    X_c = (u - c_x) \cdot z / f_x,\quad
    Y_c = (v - c_y) \cdot z / f_y,\quad
    Z_c = z
  \]
- **TF2** — Builds a `PointStamped` in `source_frame`, calls `lookup_transform(target, source, Time())`, then `do_transform_point` to get the **base** point.  
- **Left/Right mapping** — Uses MediaPipe `multi_handedness` to label hands “Left”/“Right” and publish to the corresponding topics.
- **Debug overlay** — Draws skeleton, wrist pixel, and prints `label` and `z` (meters).

### Notes / Known quirks

- The overlay message currently prints `"... z=...m )"` (an extra `)`); safe to ignore or fix in the source.  
- The log suggests `align_depth.enable:=true`; the typical RealSense flag is `align_depth:=true`.  
- Markers are **persistent** (`lifetime=0`) and large (`0.12 m`) for visibility.  
- Per‑hand markers are published **individually** *and* as a `MarkerArray`.

---

# <img src="https://github.com/XGangChen/CIRLab/blob/main/MyResearch/UR3/icon/Universal_robots_logo.svg" alt="Universal Robots Icon" width="25"> UR3

## <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> D435f_UR_human_detect.py

This node fuses **MediaPipe Hands** (human wrists) and **YOLO** (UR3 joint detections) with **aligned depth** and **camera intrinsics** to compute 3D points in the **camera optical frame**, transforms them to a **base/world** frame via **TF2**, and publishes both **points** and **RViz markers**. It also publishes an **annotated debug image** for quick verification.

### Features

- **Human wrist tracking**: MediaPipe Hands (max 2), robust depth sampling, 3D point projected to base.
- **UR3 joint detection**: YOLO model (configurable), best-per-class picking, 3D per-joint points and a **skeleton** line in base.
- **Robust depth** at a pixel via **adaptive window** (7→11→15) using median of non-zero samples.
- **Deprojection** using `CameraInfo` (pinhole `fx, fy, cx, cy`).
- **TF2 transform** from camera optical frame (**source**) to base/world (**target**).
- **Debug overlay**: hand skeleton, wrist dots, UR3 bboxes, 2D skeleton, and depth text.
- **QoS** tuned for sensors: subscribers use `SENSOR_DATA` profile.

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `color_topic` | string | `/camera/cam2/color/image_raw` | Color image (BGR8). |
| `depth_topic` | string | `/camera/cam2/aligned_depth_to_color/image_raw` | Depth aligned to **color** (`uint16` mm or `float32` m). |
| `info_topic` | string | `/camera/cam2/color/camera_info` | Camera intrinsics for **color**. |
| `source_frame` | string | `cam2_color_optical_frame` | Camera optical frame (child). |
| `target_frame` | string | `platform_base` | Base/world frame (parent). |
| `min_det_conf` | double | `0.30` | MediaPipe detection confidence. |
| `min_trk_conf` | double | `0.30` | MediaPipe tracking confidence. |
| `yolo_model_path` | string | `/media/.../UR_pose_best.pt` | YOLO weights for UR3 joints. |
| `yolo_conf` | double | `0.25` | YOLO confidence threshold. |
| `yolo_imgsz` | int | `640` | YOLO input size. |
| `ur3_class_names` | string | `UR_joints.csv` | **CSV string** of class names in link order, e.g. `base,shoulder,elbow1,elbow2,elbow3,wrist`. *Note:* despite the name, the script expects a **comma‑separated list**, not a file path. |

**QoS**: Subscriptions use `QoSPresetProfiles.SENSOR_DATA`. Publishers use a queue depth of 10.

### Topics

**Subscribed**
- `<color_topic>` — `sensor_msgs/Image` (BGR8)
- `<depth_topic>` — `sensor_msgs/Image` (`uint16` mm or `float32` m)
- `<info_topic>` — `sensor_msgs/CameraInfo`

**Published**
- **Human wrists**: `/wrist_left_point_base`, `/wrist_right_point_base` (`geometry_msgs/PointStamped`, in `target_frame`)
- **UR3 joints**: `/ur3/joint_markers` (`MarkerArray` spheres), `/ur3/skeleton` (`Marker` LINE_STRIP)
- **Debug**: `/wrist/debug_image` (`sensor_msgs/Image`, annotated BGR8)
- **(Optional)** per-joint `PointStamped`: `/ur3/{base,shoulder,elbow1,elbow2,elbow3,wrist}_point`

### How it works (pipeline)

1. **Sync state**: wait until **CameraInfo** initializes a `PinholeCameraModel` and a **depth** frame is cached.
2. **Human wrists (MediaPipe)**:
   - Detect up to 2 hands, get **wrist landmark** (index 0) → pixel `(u,v)`.
   - **Robust depth**: median of non-zero pixels over windows 7, 11, 15.
   - **Deproject** `(u,v,z)` → `(Xc,Yc,Zc)` in `source_frame` (camera optical).
   - **TF2** to `target_frame`; publish `PointStamped` and a persistent sphere `Marker` (Left/Right).
3. **UR3 joints (YOLO)**:
   - Run YOLO on the color frame; pick **best box per class** among the expected names.
   - For each joint: compute center pixel, robust depth, deproject, **TF2 to base**, publish a sphere `Marker`.
   - Build a **LINE_STRIP** skeleton through the available joints in order `base→shoulder→elbow1→elbow2→elbow3→wrist`.
   - Overlay 2D bboxes, confidences, and a 2D skeleton on the debug image.
4. **Debug image** is published every frame even if depth/intrinsics are not yet ready (draws what’s available).

### Running

```bash
python3 D435f_UR_human_detect.py \
  --ros-args \
  -p color_topic:=/camera/cam2/color/image_raw \
  -p depth_topic:=/camera/cam2/aligned_depth_to_color/image_raw \
  -p info_topic:=/camera/cam2/color/camera_info \
  -p source_frame:=cam2_color_optical_frame \
  -p target_frame:=platform_base \
  -p yolo_model_path:=/path/to/UR_pose_best.pt \
  -p ur3_class_names:="base,shoulder,elbow1,elbow2,elbow3,wrist"
```

### Camera & TF prerequisites

- **Aligned depth** to color must be enabled (RealSense example):
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
    align_depth:=true pointcloud.enable:=false color0.enable:=true depth0.enable:=true
  ```
- Provide TF from **`target_frame`** (parent) to **`source_frame`** (child) (URDF + `robot_state_publisher` or static):
  ```bash
  ros2 run tf2_ros static_transform_publisher \
    0 0 0  0 0 0  platform_base cam2_color_optical_frame
  ```

> If you see a warning like “Depth frame_id != source_frame”, your depth isn’t aligned to the color optical frame; enable alignment or change `source_frame` to match.

---

## <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> D435f_UR_pose_ROSnode.py

This ROS 2 node subscribes to a **color image** stream, runs an **Ultralytics YOLO (pose)** model to detect the **UR3 robot joints** in 2D, overlays a skeleton on the image, and publishes:
- an **annotated image** you can view in RViz or `rqt_image_view`,
- a **Float32MultiArray** containing the **6×3 keypoints** (`x, y, confidence`) for the **best** detected UR3 instance.

> This script does **not** compute depth, 3D points, or TF. It is a light, real‑time **2D pose** publisher meant to feed downstream nodes or for visualization.

### What it does

1. Subscribes to a color image (default **`/cam2/color/image_raw`**) with a camera‑friendly QoS (BEST_EFFORT, KEEP_LAST, depth=5).
2. Loads an Ultralytics **YOLO pose** model (default path comes from `--model` or `UR_MODEL` env) and runs inference per frame.
3. If multiple detections exist, it **selects the best instance** using the **mean keypoint confidence** and **publishes only one** set of 6 keypoints.
4. Publishes:
   - **Annotated image** (`sensor_msgs/Image`) on `/ur3_pose/annotated` (same timestamp/header as input).
   - **Keypoints** (`std_msgs/Float32MultiArray`) on `/ur3_pose/keypoints` with a layout describing a `(6,3)` array.
5. Applies a simple **FPS throttle** (max ~30 FPS) so inference doesn’t backlog.

### Topics

**Subscribed**
- `Image` — **color** image (BGR8): default **`/cam2/color/image_raw`**

**Published**
- `Image` — annotated color image: default **`/ur3_pose/annotated`**
- `Float32MultiArray` — keypoints (6 joints × 3 fields): default **`/ur3_pose/keypoints`**

**Joint order / skeleton**
- `JOINT_NAMES = [base, shoulder, elbow1, elbow2, elbow3, wrist]`
- `SKELETON = [(0,1), (1,2), (2,3), (3,4), (4,5)]` (drawn only when both endpoint confidences > 0.5)

### Message format — keypoints

- Type: `std_msgs/Float32MultiArray`
- Layout (`layout.dim`):
  - `dim[0]`: label=`"joints"`, size=`6`,  stride=`18`
  - `dim[1]`: label=`"fields(x,y,conf)"`, size=`3`, stride=`3`
- Data: flattened list of 18 `float32`: `[x0, y0, c0, x1, y1, c1, ..., x5, y5, c5]`

If **no detection**, an empty array (`data=[]`) is published.

### Parameters / CLI / Environment

You can set options via **CLI args** (preferred) or **environment variables**. The parser tolerates extra `--ros-args` so you can launch it like any ROS node.

| CLI arg | Env var | Default | Meaning |
|---|---|---:|---|
| `--model` | `UR_MODEL` | `"/media/.../UR_pose_best.pt"` | Path to an **Ultralytics YOLO pose** weights file. |
| `--image-topic` | `UR_IMAGE_TOPIC` | `"/cam2/color/image_raw"` | Input color image topic. |
| `--out-image-topic` | `UR_OUT_IMG` | `"/ur3_pose/annotated"` | Output annotated image topic. |
| `--out-kpt-topic` | `UR_OUT_KPT` | `"/ur3_pose/keypoints"` | Output keypoints topic. |
| `--conf` | `UR_CONF` | `0.30` | YOLO detection confidence threshold. |

> Device selection is automatic: **CUDA** if available, otherwise **CPU**.

### QoS

The subscriber uses a camera‑friendly **BEST_EFFORT** / **KEEP_LAST** / **depth=5** profile to match most camera drivers and avoid reliability mismatches. The annotated image is published with the same QoS; the keypoints publisher uses a small queue (depth=10).

### Running

Direct (handy for VS Code)
```bash
python3 D435f_UR_pose_ROSnode.py \
  --model /path/to/UR_pose_best.pt \
  --image-topic /cam2/color/image_raw \
  --out-image-topic /ur3_pose/annotated \
  --out-kpt-topic /ur3_pose/keypoints \
  --conf 0.30
# Extra flags like --ros-args are tolerated (ignored by the parser)
```

### Performance tips

- Use a smaller input image (camera resolution) or a lighter YOLO model to raise FPS.
- Increase `--conf` to filter weak detections.
- Prefer **CUDA** if available; make sure PyTorch is installed with the right CUDA version.
- The node already throttles to ~30 FPS; if your camera publishes faster, you won’t process every frame.

---

# <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/ros/ros-original.svg" alt="ROS2 Icon" width="30"> ROS2 Launch & Tools

This project is using ROS2 Humble to launch the RealSense D435f camera. I'll go through all the commands and tools I used in this document.

## Launch the Camera
```
ros2 launch realsense2_camera rs_launch.py   camera_name:=cam2 enable_color:=true enable_depth:=true   align_depth.enable:=true initial_reset:=true
```
This command runs the ROS2 RealSense driver with the exact options we need for wrist-depth.  
- `camera_name:=cam2`: names/namespace for the node and topics (e.g. we put the node under a camera namespace -> `/camera/cam2/...`).
- `enable_color:=true`: publish RGB frames (e.g. `/.../color/image_raw` and `/.../color/camera_info`).
- `enable_depth:=true`: publish depth frames.
- `align_depth.enable:=true`: **aligns the depth map to the color camera** and creates the topic `/.../aligned_depth_to_color/image_raw` with `frame_id: cam2_color_optical_frame`.
  - This is critical so the wrist pixel `(u,v)` from the color image corresponds to the same pixel in the depth image.
- `initial_reset:=true`: hard-resets the camera at startup (helpful if the device was left in a bad state or after USB hiccups), ensuring clean streaming and correct intrinsics.

---

## Using RQT to See the Images
```
ros2 run rqt_image_view rqt_image_view
```
This command launches the rqt Image View GUI so you can preview any sensor_msgs/Image topic in real time.
- Pick a topic from the dropdown (e.g. `/camera/cam2/color/image_raw`, `/camera/cam2/aligned_depth_to_color/image_raw`)
- It subscribes and shows the frames live—great for checking if your pipeline is actually publishing and what it looks like.
- To check the topic exists and is active: `ros2 topic list | grep image` and `ros2 topic hz <topic>`.

---

## Using RViz2 to Get the Whole Environment
```
rviv2
# or
ros2 run rviz2 rviz2
```
Launch the RViz2 and set up:  
1. In RViz, set Fixed Frame to `platform_base` (top left → “Global Options”).
2. Adds displays:
   - TF (to see frames)
   - Marker with topic `/wrist_marker` (enable namespace `wrist`)
   - (optional) MarkerArray with topic `/wrist_marker_array`
   - (optional) Image with topic `/wrist/debug_image` to see the overlay

---

## <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> ROS_scene_builder.py

A lightweight ROS 2 Python node that **publishes static TFs** and **draws simple RViz markers** for a lab “scene.”  
It anchors a UR3 robot’s root frame under a platform, places a camera above the platform, and draws a platform slab, a cube, and camera axes.

### What it does

- Publishes static transforms on `/tf_static`:
  - `world → platform_base` (identity)
  - `platform_base → cam2_color_optical_frame`
  - `platform_base → base_link` (UR3 root; change to `base` if your UR driver uses that)
- Publishes a `visualization_msgs/MarkerArray` on `/visualization_marker_array` at 2 Hz:
  - A gray platform slab with an outline
  - A blue “cube” object
  - RGB arrows that depict the camera axes, **computed in the platform frame** so they render without adding a separate TF in RViz

> The math uses a roll‑pitch‑yaw → quaternion helper and a quaternion‑vector rotation routine to obtain the axis directions in platform coordinates.

### Requirements

- ROS 2 (tested with Humble/Foxy–Galactic should also work)
- Python 3 with these ROS 2 packages available:
  - `rclpy`
  - `geometry_msgs`
  - `visualization_msgs`
  - `tf2_ros`

> Make sure your environment is sourced:  
> `source /opt/ros/<distro>/setup.bash`

### Quick start

1. Place `scene_builder.py` somewhere in your workspace (it can also run as a standalone script).
2. Source your ROS 2 setup:  
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```
3. Run the node:
   ```bash
   python3 scene_builder.py
   ```
4. Start RViz2:
   ```bash
   rviz2
   ```
   In RViz2:
   - **Fixed Frame**: `world` (or `platform_base`, since world→platform is identity)
   - **Add** → **TF** (optional, to see TF tree)
   - **Add** → **MarkerArray**, set **Topic** to `/visualization_marker_array`

You should see:
- A 0.80 × 0.56 × 0.02 m platform at the origin (`platform_base`)
- A blue cube centered at `(0.0, −0.35, 0.25)` with size `0.40 × 0.14 × 0.50 m`
- Camera axes originating at `(0.0, 0.0, 1.25)` (in platform coordinates)

### Integrating with `ur_robot_driver`

Launch your driver **without RViz**:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=<IP> launch_rviz:=false
```
Then run **this** node and your **own RViz2** as above.  
This script sets a static transform `platform_base → base_link` so your UR3 TF tree hangs under the platform.

> If your URDF uses `base` (not `base_link`) as the root, change `self.ur_root_frame` in the script accordingly.

### Customization knobs (edit in the script)

- **Camera pose wrt platform**
  ```py
  cam_xyz = (0.0, 0.0, 1.25)
  cam_rpy = (math.pi, 0, 0.0)
  ```
- **UR3 root pose wrt platform**
  ```py
  ur_xyz = (-0.20, -0.35, 0.38)
  ur_rpy = (math.pi/2, 0, -math.pi/2)  # if use_quaternion_for_base == False
  ```
  - To rotate the robot **about the Z‑axis by +90°** (yaw):  
    `ur_rpy = (0, 0, math.pi/2)`
  - To rotate **about Z by 180°**:  
    `ur_rpy = (0, 0, math.pi)`
  - To use a quaternion directly, set:
    ```py
    use_quaternion_for_base = True
    ur_quat_cli = (qx, qy, qz, qw)
    ```
- **Geometry**
  ```py
  platform_size = (0.80, 0.56, 0.02)
  cube_size     = (0.40, 0.14, 0.50)
  cube_center   = (0.0, -0.35, 0.25)
  ```

### Why static timestamps are 0

The node stamps TFs and markers with **time = 0** to avoid TF extrapolation issues if your clock is not running or when bags are used. RViz treats time 0 as “latest available.”

### Topics and frames

- Publishes:
  - `/tf_static` (`tf2_msgs/TFMessage`, via `StaticTransformBroadcaster`)
  - `/visualization_marker_array` (`visualization_msgs/MarkerArray`, 2 Hz)
- Frames created/used:
  - `world`, `platform_base`, `cam2_color_optical_frame`, and `base_link` (or `base`)




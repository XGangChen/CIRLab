# Function Explanation
I'll explain what each script works for. The Python scripts need to be paired with ROS2 commands and tools.  
There's the [ROS2](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence/ROS2) folder under [RealSense](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence) in this repository.  
We need to launch the camera before running these scripts. Please check it out.  

> I run all the Python scripts in Python3.10-venv. Since we're using ROS, do not use Conda to avoid unnecessary troubles. My software setup is Ubuntu 22.04 LTS and ROS2 Humble, so we're using Python3.10. 

# Table of Contents
* [Python Scripts](#python-scripts)
  * [Camera](#camera)
    * [mediapipe_wrist_debug.py](#-mediapipe_wrist_debugpy)
    * [cam_info_tf_check.py](#-cam_info_tf_checkpy)
    * [wrist_to_base.py](#-wrist_to_basepy)
* [ROS2 Launch & Tools](#ros2-launch--tools)
  * [Launch the Camera](#launch-the-camera)
  * [TFtree2 Environment Setup](#tftree2-environment-setup)
  * [Using RQT to See the Images](#using-rqt-to-see-the-images)
  * [Using RViz2 to Get the Whole Environment](#using-rviz2-to-get-the-whole-environment)



---

# Python Scripts
## Camera
### <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> mediapipe_wrist_debug.py

This ROS 2 node subscribes to a color stream, an **aligned depth-to-color** stream, and color **CameraInfo**, runs **MediaPipe Hands** to detect up to **two** hands, overlays wrist landmarks and metrics (depth `z` and Euclidean range `R`) on the color image, and publishes a debug image you can view in RViz or `rqt_image_view`.

> This node is **for visualization/debug**: it does **not** publish 3D points or TF. Use it to verify hands, depth alignment, and camera intrinsics.

#### Features

- **Hands detection (max 2)** via MediaPipe with lightweight tracking.
- **Robust depth**: 7×7 patch median (ignores 0s) at the wrist pixel.
- **Deprojection** using **CameraInfo** (pinhole intrinsics) to compute camera-frame `(X, Y, Z)` and range `R = √(X²+Y²+Z²)`.
- **On-image overlays**: wrist pixel, hand connections, handedness label (“Left/Right”), `z` (meters), and `R` (meters).
- **Guardrails**:
  - Accepts depth in `uint16` (mm → m) or `float32` (m).
  - Warns if depth `frame_id` isn’t the expected **color optical frame** (misalignment hint).
  - Tolerant to missing depth/intrinsics (still draws landmarks, just omits numbers).

#### Topics & Parameters

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

#### Running

- Quick test (direct Python)
  ```bash
  python3 mediapipe_wrist_debug.py \
    --ros-args \
    -p color_topic:=/camera/cam2/color/image_raw \
    -p depth_topic:=/camera/cam2/aligned_depth_to_color/image_raw \
    -p info_topic:=/camera/cam2/color/camera_info
  ```
  > Make sure the camera has been launched and the TF tree has been set up.

#### How it works (algorithm)

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

### <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> cam_info_tf_check.py

This tiny ROS 2 tool verifies **camera intrinsics** (from `CameraInfo`) and the **TF transform** between two frames. It prints the focal lengths and principal point **once**, then reports the transform **every second** so you can confirm your extrinsics are being published correctly.

#### What it does

1. **Subscribes** to a `sensor_msgs/CameraInfo` topic (default **`/camera/cam2/color/image_raw/theora`** — you will almost certainly change this to your camera’s `.../camera_info` topic).
2. **Parses intrinsics** from the `K` matrix:
   - `fx = K[0,0]`, `fy = K[1,1]`, `cx = K[0,2]`, `cy = K[1,2]`.
3. **Looks up TF** from **`target_frame`** (default `platform_base`) to **`source_frame`** (default `cam2_color_optical_frame`) once per second.
4. **Logs** the translation `(x,y,z)` and quaternion `(x,y,z,w)` when available; otherwise, prints a warning until TF exists.

> This node is read-only: it does not publish topics, only logs to the console.

#### Parameters (ROS 2)

| Name | Type | Default | Description |
|---|---|---|---|
| `info_topic` | string | `/camera/cam2/color/image_raw/theora` | The **CameraInfo** topic to read. Change this to your camera’s `.../camera_info`. |
| `source_frame` | string | `cam2_color_optical_frame` | The **child/source** frame (e.g., color optical frame). |
| `target_frame` | string | `platform_base` | The **parent/target** frame (e.g., robot/world base). |

> **QoS**: The subscription is created with `QoSProfile(depth=10)`. If you see no CameraInfo arriving, switch to `QoSPresetProfiles.SENSOR_DATA` to match camera drivers (see “Improvements” below).

#### Running

```bash
python3 cam_info_tf_check.py \
  --ros-args \
  -p info_topic:=/camera/cam2/color/camera_info \
  -p source_frame:=cam2_color_optical_frame \
  -p target_frame:=platform_base
```

---

### <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg" alt="Python icon" width="20"> wrist_to_base.py

This node detects **hands** with **MediaPipe**, samples **aligned depth** at the **wrist** pixel, **deprojects** to 3D using the color camera intrinsics, transforms the point to a chosen **base frame** via **TF2**, and publishes both left/right `PointStamped` and RViz `Marker`(s). It also publishes an annotated **debug image**.

> Script: `wrist_to_base.py` (Python, ROS 2 rclpy).

#### What it does

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

#### Parameters

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

#### Topics

**Subscribed**  
- `<color_topic>` — `sensor_msgs/Image` (BGR8)  
- `<depth_topic>` — `sensor_msgs/Image` (`uint16` mm or `float32` m)  
- `<info_topic>` — `sensor_msgs/CameraInfo`

**Published**  
- `/wrist_left_point_base`, `/wrist_right_point_base` — `geometry_msgs/PointStamped` (in `target_frame`)  
- `/wrist_marker` — `visualization_msgs/Marker` (one per hand)  
- `/wrist_marker_array` — `visualization_msgs/MarkerArray`  
- `/wrist/debug_image` — `sensor_msgs/Image` (BGR8) annotated

#### Running

```bash
python3 wrist_to_base.py \
  --ros-args \
  -p color_topic:=/camera/cam2/color/image_raw \
  -p depth_topic:=/camera/cam2/aligned_depth_to_color/image_raw \
  -p info_topic:=/camera/cam2/color/camera_info \
  -p source_frame:=cam2_color_optical_frame \
  -p target_frame:=platform_base
```

#### How it works (key internals)

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

#### Notes / Known quirks

- The overlay message currently prints `"... z=...m )"` (an extra `)`); safe to ignore or fix in the source.  
- The log suggests `align_depth.enable:=true`; the typical RealSense flag is `align_depth:=true`.  
- Markers are **persistent** (`lifetime=0`) and large (`0.12 m`) for visibility.  
- Per‑hand markers are published **individually** *and* as a `MarkerArray`.

---

# <img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/ros/ros-original.svg" alt="ROS2 Icon" width="30"> ROS2 Launch & Tools

This project is using ROS2 Humble to launch the RealSense D435f camera. I'll go through all the commands and tools I used in this document.

## Launch the Camera
```
ros2 launch realsense2_camera rs_launch.py   camera_name:=cam2 enable_color:=true enable_depth:=true   align_depth.enable:=true initial_reset:=true
```

## TFtree2 Environment Setup
```
ros2 run tf2_ros static_transform_publisher   0 0 1.25 3.14159265 -3.14159265 0  platform_base cam2_color_optical_frame
```

## Using RQT to See the Images
```
ros2 run rqt_image_view rqt_image_view
```

## Using RViz2 to Get the Whole Environment
```
rviv2
```


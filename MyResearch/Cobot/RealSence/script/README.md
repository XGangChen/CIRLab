# Function Explanation
I'll explain what each script works for. The Python scripts need to be paired with ROS2 commands and tools.  
There's the [ROS2](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence/ROS2) folder under [RealSense](https://github.com/XGangChen/CIRLab/tree/main/MyResearch/Cobot/RealSence) in this repository.  
We need to launch the camera before running these scripts. Please check it out.  

> I run all the Python scripts in Python3.10-venv. Since we're using ROS, do not use Conda to avoid unnecessary troubles. My software setup is Ubuntu 22.04 LTS and ROS2 Humble, so we're using Python3.10. 

## mediapipe_wrist_debug.py

This ROS 2 node subscribes to a color stream, an **aligned depth-to-color** stream, and color **CameraInfo**, runs **MediaPipe Hands** to detect up to **two** hands, overlays wrist landmarks and metrics (depth `z` and Euclidean range `R`) on the color image, and publishes a debug image you can view in RViz or `rqt_image_view`.

> This node is **for visualization/debug**: it does **not** publish 3D points or TF. Use it to verify hands, depth alignment, and camera intrinsics.

---

### Features

- **Hands detection (max 2)** via MediaPipe with lightweight tracking.
- **Robust depth**: 7×7 patch median (ignores 0s) at the wrist pixel.
- **Deprojection** using **CameraInfo** (pinhole intrinsics) to compute camera-frame `(X, Y, Z)` and range `R = √(X²+Y²+Z²)`.
- **On-image overlays**: wrist pixel, hand connections, handedness label (“Left/Right”), `z` (meters) and `R` (meters).
- **Guardrails**:
  - Accepts depth in `uint16` (mm → m) or `float32` (m).
  - Warns if depth `frame_id` isn’t the expected **color optical frame** (misalignment hint).
  - Tolerant to missing depth/intrinsics (still draws landmarks, just omits numbers).

---

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

---

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

---

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

## cam_info_tf_check.py

## wrist_to_base.py

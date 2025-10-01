# RealSense D435f with YOLOv11 Integration

This directory contains scripts to integrate Intel RealSense D435f camera with YOLOv11 object detection and pose estimation.

## Files

1. **`realsense_yolo11.py`** - Simple integration script
2. **`realsense_yolo11_configurable.py`** - Advanced script with configurable options
3. **`test_environment.py`** - Environment test script
4. **`opencv_viewer.py`** - Original RealSense viewer script

## Requirements

All required packages are already installed in your conda environment:
- `pyrealsense2` (2.56.5.9235)
- `ultralytics` (8.3.179)
- `torch` (2.8.0)
- `opencv-python` (4.12.0.88)

## Usage

### 1. Test Your Environment First

```bash
python test_environment.py
```

This will verify that:
- RealSense camera is connected and working
- YOLOv11 models are accessible
- CUDA is available (if you have a GPU)

### 2. Simple Usage

```bash
python realsense_yolo11.py
```

This runs with default settings:
- Uses `yolo11n-pose.pt` model for pose detection
- 1280x720 resolution at 30 FPS
- Displays RGB camera feed with YOLO detections + depth colormap

### 3. Advanced Usage (Configurable Script)

```bash
# Basic pose detection
python realsense_yolo11_configurable.py --model pose_nano

# Object detection instead of pose
python realsense_yolo11_configurable.py --model detection_nano

# High-quality pose detection
python realsense_yolo11_configurable.py --model pose_large --confidence 0.7

# Custom resolution and FPS
python realsense_yolo11_configurable.py --model pose_medium --resolution 1920x1080 --fps 15

# Use your trained TM robot pose model
python realsense_yolo11_configurable.py --model tm_pose_best --confidence 0.8
```

### Available Models

The configurable script supports these models:

- **Pose Detection:**
  - `pose_nano` - YOLOv11n-pose (fastest)
  - `pose_medium` - YOLOv11m-pose (balanced)
  - `pose_large` - YOLOv11l-pose (more accurate)
  - `pose_xlarge` - YOLOv11x-pose (most accurate)

- **Object Detection:**
  - `detection_nano` - YOLOv11n (general objects)

- **Custom Trained Models:**
  - `tm_pose_best` - Your trained TM robot pose model
  - `ur3_pose` - UR3 robot pose model

### Command Line Options

```bash
python realsense_yolo11_configurable.py --help
```

Options:
- `--model` - Choose model type (see above)
- `--confidence` - Detection confidence threshold (0.0-1.0, default: 0.5)
- `--resolution` - Camera resolution (default: 1280x720)
- `--fps` - Frame rate (default: 30)

## Controls

- **Press 'q'** to quit the application
- The display shows:
  - Left side: RGB camera feed with YOLO detections
  - Right side: Depth colormap
  - Top overlay: Model info, device info, and controls

## Output

### Pose Detection
- Green circles: Detected keypoints
- Blue lines: Skeleton connections
- Numbers: Keypoint indices (0-16 for COCO format)

### Object Detection
- Green rectangles: Bounding boxes
- Labels: Class name and confidence score

## Troubleshooting

1. **"No RealSense device found"**
   - Check USB connection
   - Make sure camera is powered
   - Try a different USB port (USB 3.0 recommended)

2. **"YOLO model not found"**
   - Check if the model file exists at the specified path
   - Use `--model detection_nano` for a different model

3. **Low performance**
   - Use smaller resolution: `--resolution 640x480`
   - Use nano model: `--model pose_nano`
   - Lower FPS: `--fps 15`

4. **CUDA not working**
   - The script will automatically fall back to CPU
   - Check your CUDA installation if you want GPU acceleration

## Example Commands

```bash
# Quick test
python test_environment.py

# Basic pose detection
python realsense_yolo11.py

# High-quality pose detection with custom settings
python realsense_yolo11_configurable.py --model pose_large --confidence 0.7 --resolution 1920x1080

# Object detection mode
python realsense_yolo11_configurable.py --model detection_nano --confidence 0.6

# Use your custom trained model
python realsense_yolo11_configurable.py --model tm_pose_best --confidence 0.8
```

Enjoy real-time pose estimation and object detection with your RealSense camera!

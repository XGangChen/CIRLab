# coding: utf-8
"""
TM Robot Arm Real-time Pose Detection using Kinect v2
This script integrates Kinect v2 real-time video capture with trained YOLO TM robot arm pose detection.

Features:
- Real-time Kinect v2 color and depth capture
- TM robot arm pose detection using custom trained YOLO model
- Skeleton visualization with joint names
- Performance monitoring (FPS, frame count)
- GPU acceleration support

Controls:
- Press 'q' to quit
"""
import torch
import numpy as np
import cv2
import sys
import time
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

# Load trained TM robot arm pose detection model
from ultralytics import YOLO
import torch

# Check GPU availability and load model accordingly
if torch.cuda.is_available():
    print(f"CUDA is available! Using GPU: {torch.cuda.get_device_name(0)}")
    cuda_device = 'cuda'
else:
    print("CUDA not available, using CPU")
    cuda_device = 'cpu'

# Use your trained TM robot arm model (you can change this path to use different model versions)
# Available models:
# - tm_pose_yolov11/weights/best.pt
# - tm_pose_yolov112/weights/best.pt  
# - tm_pose_yolov11_20250624/weights/best.pt
model_path = '/media/xgang/XGang-1T/CIRLab/MyResearch/TM_pose/runs/pose/tm_pose_yolov112/weights/best.pt'
model = YOLO(model_path)
model.to(cuda_device)
print(f"TM robot arm pose model loaded on {cuda_device.upper()}")
print(f"Model path: {model_path}")

# TM robot arm joint configuration
joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3', 'gripper']
joint_colors = [
    (200, 0, 200), (255, 150, 50), (0, 100, 255),
    (255, 255, 0), (255, 255, 0), (255, 255, 0), (255, 0, 255)
]
skeleton = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6)] 

# Select an available packet pipeline
try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except Exception:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except Exception:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

# Initialize Kinect device
fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)
serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

# Set up multi-frame listener for Color, IR, and Depth
listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)
device.start()

# Registration must be created after device.start()
registration = Registration(
    device.getIrCameraParams(), device.getColorCameraParams())

# Pre-allocate frames for registration output
undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

# Optional: bigdepth for full-resolution depth
need_bigdepth = False            # Use full-resolution depth
need_color_depth_map = False     # Use color-depth map
bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512), np.int32).ravel() if need_color_depth_map else None

# Performance monitoring
import time
frame_count = 0
start_time = time.time()

# Main loop
while True:
    frames = listener.waitForNewFrame()
    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    # Align depth to color
    registration.apply(
        color, depth, undistorted, registered,
        bigdepth=bigdepth,
        color_depth_map=color_depth_map)

    # Handle different channel layouts: 4->BGRA, 3->BGR, 1->Gray
    c = color.asarray()
    if c.ndim == 3 and c.shape[2] == 4:
        img_rgb = cv2.cvtColor(c, cv2.COLOR_BGRA2RGB)
    elif c.ndim == 3 and c.shape[2] == 3:
        img_rgb = c[:, :, ::-1]
    else:
        img_rgb = cv2.cvtColor(c, cv2.COLOR_GRAY2RGB)

    # Run TM robot arm pose detection
    results = model.predict(source=img_rgb, conf=0.3, save=False, show=False)
    keypoints = results[0].keypoints

    # Check if keypoints exist and process TM robot arm keypoints
    if keypoints is not None and keypoints.conf is not None and len(keypoints.conf) > 0:
        # Select most confident keypoint set
        avg_conf = keypoints.conf.mean(dim=1)
        best_idx = int(torch.argmax(avg_conf))
        best_kpt = keypoints.data[best_idx].cpu().numpy()  # shape: (7, 3)

        # Draw skeleton connections
        for i, j in skeleton:
            if best_kpt[i][2] > 0.5 and best_kpt[j][2] > 0.5:
                pt1 = tuple(map(int, best_kpt[i][:2]))
                pt2 = tuple(map(int, best_kpt[j][:2]))
                cv2.line(img_rgb, pt1, pt2, color=joint_colors[i], thickness=3)

        # Draw keypoints with labels
        for i, (x, y, conf) in enumerate(best_kpt):
            if conf > 0.5:  # Only draw high-confidence keypoints
                color = joint_colors[i]
                # Convert BGR to RGB for drawing
                color_rgb = (color[2], color[1], color[0])
                cv2.circle(img_rgb, (int(x), int(y)), radius=8, color=color_rgb, thickness=-1)
                # Add joint label
                cv2.putText(img_rgb, joint_names[i], (int(x) + 10, int(y) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    else:
        # Display "No TM robot detected" message
        cv2.putText(img_rgb, "No TM robot detected", (30, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Prepare display frame (RGB->BGR) - ensure contiguous array
    display_frame = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

    # Calculate FPS
    frame_count += 1
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time if elapsed_time > 0 else 0

    # Add performance and detection info
    cv2.putText(display_frame, f"Device: {cuda_device.upper()}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(display_frame, f"Frame: {frame_count}", (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Show views
    cv2.imshow("TM Robot Arm Detection", cv2.resize(display_frame, (640, 360)))
    cv2.imshow("depth", depth.asarray() / 4500.)
    # cv2.imshow("registered", registered.asarray(np.uint8))
    if need_bigdepth:
        bd = bigdepth.asarray(np.float32)
        cv2.imshow("bigdepth", cv2.resize(bd, (640, 360)))
    if need_color_depth_map:
        # cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

        # Convert to uint8 or float32 for display
        cdm_display = (color_depth_map.reshape(424, 512) / color_depth_map.max() * 255).astype(np.uint16)
        cv2.imshow("color_depth_map", cdm_display)

    listener.release(frames)
    
    # Handle keyboard input
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):  # Reset frame counter
        frame_count = 0
        start_time = time.time()
        print("Performance counters reset")
    elif key == ord('h'):  # Show help
        print("\nKeyboard controls:")
        print("  'q' - Quit")
        print("  'r' - Reset FPS counter")
        print("  'h' - Show this help")

# Clean up
device.stop()
device.close()
cv2.destroyAllWindows()
print(f"\nSession summary:")
print(f"Total frames processed: {frame_count}")
print(f"Average FPS: {frame_count / elapsed_time:.1f}")
sys.exit(0)

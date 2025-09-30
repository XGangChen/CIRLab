# coding: utf-8
import torch
import numpy as np
import cv2
import sys
import os
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

# Load YOLOv11-pose model
from ultralytics import YOLO
import torch

# Check GPU availability and load model accordingly
if torch.cuda.is_available():
    print(f"CUDA is available! Using GPU: {torch.cuda.get_device_name(0)}")
    device = 'cuda'
else:
    print("CUDA not available, using CPU")
    device = 'cpu'

model = YOLO('yolo11n-pose.pt')
model.to(device)
print(f"YOLO model loaded on {device.upper()}") 

# Use CPU pipeline to avoid OpenGL issues over SSH
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
need_bigdepth = False
need_color_depth_map = False
bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512), np.int32).ravel() if need_color_depth_map else None

# Create output directory
output_dir = "kinect_yolo_output"
os.makedirs(output_dir, exist_ok=True)
frame_count = 0

print(f"Starting capture... Output will be saved to {output_dir}/")
print("Press Ctrl+C to stop")

# Main loop
try:
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

        # Run YOLOv11-pose inference
        results = model(img_rgb, device=device)

        # Draw detected keypoints (x, y, confidence)
        for res in results:
            if hasattr(res, 'keypoints') and res.keypoints is not None:
                pts = res.keypoints.data.cpu().numpy()
                # Process keypoints for each person detected
                for person_keypoints in pts:
                    # Each keypoint has format [x, y, confidence]
                    for kp in person_keypoints:
                        x, y, conf = kp[0], kp[1], kp[2]
                        if conf > 0.3:  # Only draw high-confidence keypoints
                            cv2.circle(img_rgb, (int(x), int(y)), 4, (0, 255, 0), -1)

        # Prepare display frame (RGB->BGR)
        display_frame = img_rgb[:, :, ::-1]

        # Save frames instead of displaying (every 5th frame to reduce file count)
        if frame_count % 5 == 0:
            # Save depth frame
            depth_normalized = (depth.asarray() / 4500. * 255).astype(np.uint8)
            cv2.imwrite(f"{output_dir}/depth_{frame_count:06d}.jpg", depth_normalized)
            
            # Save color frame with keypoints
            cv2.imwrite(f"{output_dir}/color_{frame_count:06d}.jpg", cv2.resize(display_frame, (640, 360)))
            
            print(f"Saved frame {frame_count}")

        listener.release(frames)
        frame_count += 1
        
        # Stop after 200 frames (adjust as needed)
        if frame_count > 200:
            print("Reached frame limit, stopping...")
            break

except KeyboardInterrupt:
    print("\nStopping capture...")

# Clean up
device.stop()
device.close()
print(f"Saved {frame_count} frames to {output_dir}/")
sys.exit(0)
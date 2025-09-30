# coding: utf-8
import torch
import numpy as np
import cv2
import sys
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
                        cv2.circle(img_rgb, (int(x), int(y)), 10, (0, 255, 0), -1) # Draw keypoint

    # Prepare display frame (RGB->BGR)
    display_frame = img_rgb[:, :, ::-1]

    # Show views
    # cv2.imshow("ir", ir.asarray() / 65535.)
    cv2.imshow("depth", depth.asarray() / 4500.)
    cv2.imshow("color", cv2.resize(display_frame, (640, 360)))
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
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
device.stop()
device.close()
cv2.destroyAllWindows()
sys.exit(0)

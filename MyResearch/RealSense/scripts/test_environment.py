#!/usr/bin/env python3
"""
Test script to verify RealSense and YOLOv11 integration
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import os

def test_environment():
    """Test if all required components are available"""
    print("=== Environment Test ===")
    
    # Test PyTorch
    print(f"PyTorch version: {torch.__version__}")
    print(f"CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"CUDA device: {torch.cuda.get_device_name(0)}")
    
    # Test OpenCV
    print(f"OpenCV version: {cv2.__version__}")
    
    # Test RealSense
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        print(f"RealSense devices found: {len(devices)}")
        for i, device in enumerate(devices):
            print(f"  Device {i}: {device.get_info(rs.camera_info.name)}")
    except Exception as e:
        print(f"RealSense error: {e}")
        return False
    
    # Test YOLO model availability
    model_path = '/media/xgang/XGang-1T/CIRLab/MyResearch/yolo11n-pose.pt'
    if os.path.exists(model_path):
        print(f"YOLO model found: {model_path}")
        try:
            model = YOLO(model_path)
            print("YOLO model loaded successfully")
        except Exception as e:
            print(f"YOLO model loading error: {e}")
            return False
    else:
        print(f"YOLO model not found: {model_path}")
        return False
    
    print("=== All tests passed! ===")
    return True

def quick_test():
    """Quick test with a simple frame"""
    print("\n=== Quick RealSense Test ===")
    
    # Configure pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        # Start streaming
        pipeline.start(config)
        print("RealSense streaming started")
        
        # Capture a few frames
        for i in range(5):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                print(f"Frame {i+1}: {color_image.shape}")
        
        print("RealSense test completed successfully")
        
    except Exception as e:
        print(f"RealSense test error: {e}")
        return False
    finally:
        pipeline.stop()
    
    return True

if __name__ == "__main__":
    if test_environment():
        if quick_test():
            print("\n✅ All systems ready! You can now run the main script.")
        else:
            print("\n❌ RealSense test failed.")
    else:
        print("\n❌ Environment test failed.")

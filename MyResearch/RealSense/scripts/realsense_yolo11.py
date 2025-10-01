## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##   RealSense D435f with YOLOv11 Detection ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from ultralytics import YOLO

# Check GPU availability and load model accordingly
if torch.cuda.is_available():
    print(f"CUDA is available! Using GPU: {torch.cuda.get_device_name(0)}")
    device = 'cuda'
else:
    print("CUDA not available, using CPU")
    device = 'cpu'

# Load YOLOv11 model (you can change this to any available model)
model_path = '/media/xgang/XGang-1T/CIRLab/MyResearch/yolo11n-pose.pt'  # pose detection model
# Alternative models you can use:
# model_path = '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/TM_pose/script/yolo11n.pt'  # object detection
model = YOLO(model_path)
model.to(device)
print(f"YOLO model loaded on {device.upper()}")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device_rs = pipeline_profile.get_device()
device_product_line = str(device_rs.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device_rs.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Define COCO pose skeleton connections for visualization
SKELETON_CONNECTIONS = [
    (5, 6),   # left_shoulder -> right_shoulder
    (5, 7),   # left_shoulder -> left_elbow
    (7, 9),   # left_elbow -> left_wrist
    (6, 8),   # right_shoulder -> right_elbow
    (8, 10),  # right_elbow -> right_wrist
    (5, 11),  # left_shoulder -> left_hip
    (6, 12),  # right_shoulder -> right_hip
    (11, 12), # left_hip -> right_hip
    (11, 13), # left_hip -> left_knee
    (13, 15), # left_knee -> left_ankle
    (12, 14), # right_hip -> right_knee
    (14, 16), # right_knee -> right_ankle
]

def draw_pose_keypoints(image, keypoints, confidence_threshold=0.5):
    """Draw pose keypoints and skeleton on the image"""
    if len(keypoints) == 0:
        return image
    
    # keypoints shape: [num_people, 17, 3] (x, y, confidence)
    for person_keypoints in keypoints:
        # Draw keypoints
        for i, (x, y, conf) in enumerate(person_keypoints):
            if conf > confidence_threshold:
                cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)
                cv2.putText(image, str(i), (int(x), int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        # Draw skeleton connections
        for connection in SKELETON_CONNECTIONS:
            kpt1_idx, kpt2_idx = connection
            if (kpt1_idx < len(person_keypoints) and kpt2_idx < len(person_keypoints)):
                kpt1 = person_keypoints[kpt1_idx]
                kpt2 = person_keypoints[kpt2_idx]
                
                if kpt1[2] > confidence_threshold and kpt2[2] > confidence_threshold:
                    cv2.line(image, (int(kpt1[0]), int(kpt1[1])), 
                            (int(kpt2[0]), int(kpt2[1])), (255, 0, 0), 2)
    
    return image

def draw_detection_boxes(image, results):
    """Draw bounding boxes and labels for object detection"""
    for result in results:
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                
                # Get class name
                class_name = result.names[class_id]
                
                # Draw bounding box
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # Draw label
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(image, label, (int(x1), int(y1-10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return image

try:
    print("Starting RealSense capture with YOLOv11 detection...")
    print("Press 'q' to quit")
    
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLOv11 inference on color image
        results = model(color_image, verbose=False)
        
        # Create a copy of color image for drawing
        annotated_image = color_image.copy()
        
        # Check if this is pose detection or object detection model
        if 'pose' in model_path:
            # Handle pose detection
            for result in results:
                if hasattr(result, 'keypoints') and result.keypoints is not None:
                    keypoints = result.keypoints.xy.cpu().numpy()  # Get (x, y) coordinates
                    confidences = result.keypoints.conf.cpu().numpy()  # Get confidence scores
                    
                    # Combine coordinates and confidences
                    keypoints_with_conf = np.concatenate([keypoints, confidences[..., np.newaxis]], axis=-1)
                    annotated_image = draw_pose_keypoints(annotated_image, keypoints_with_conf)
        else:
            # Handle object detection
            annotated_image = draw_detection_boxes(annotated_image, results)

        # Apply colormap on depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Resize images to match for display
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = annotated_image.shape

        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(annotated_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((annotated_image, depth_colormap))

        # Add text overlay with information
        cv2.putText(images, f"Model: {model_path.split('/')[-1]}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(images, f"Device: {device.upper()}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(images, "Press 'q' to quit", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Show images
        cv2.namedWindow('RealSense + YOLOv11', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense + YOLOv11', images)
        
        # Break loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Stopped RealSense capture")

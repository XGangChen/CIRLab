## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##   RealSense D435f with YOLOv11 Detection ##
##   Configurable Model Selection            ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import argparse
import os

# Available model configurations
MODEL_CONFIGS = {
    'pose_nano': '/media/xgang/XGang-1T/CIRLab/MyResearch/yolo11n-pose.pt',
    'pose_medium': '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/TM_pose/script/yolo11m-pose.pt',
    'pose_large': '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/TM_pose/script/yolo11l-pose.pt',
    'pose_xlarge': '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/TM_pose/script/yolo11x-pose.pt',
    'detection_nano': '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/TM_pose/script/yolo11n.pt',
    'ur3_pose': '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/UR3_pose/script/yolo11n.pt',
    'tm_pose_best': '/media/xgang/XGang-1T/CIRLab/MyResearch/Robot_Pose/TM_pose/script/runs/pose/tm_pose_yolov11_202508193/weights/best.pt',
}

def parse_arguments():
    parser = argparse.ArgumentParser(description='RealSense D435f with YOLOv11 Detection')
    parser.add_argument('--model', type=str, default='pose_nano', 
                       choices=list(MODEL_CONFIGS.keys()),
                       help='Choose YOLO model configuration')
    parser.add_argument('--confidence', type=float, default=0.5,
                       help='Confidence threshold for detection')
    parser.add_argument('--resolution', type=str, default='1280x720',
                       help='Camera resolution (widthxheight)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Camera frame rate')
    return parser.parse_args()

class RealSenseYOLO:
    def __init__(self, model_path, confidence_threshold=0.5, resolution=(1280, 720), fps=30):
        self.confidence_threshold = confidence_threshold
        
        # Check GPU availability and load model
        if torch.cuda.is_available():
            print(f"CUDA is available! Using GPU: {torch.cuda.get_device_name(0)}")
            self.device = 'cuda'
        else:
            print("CUDA not available, using CPU")
            self.device = 'cpu'

        # Load YOLOv11 model
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        self.model = YOLO(model_path)
        self.model.to(self.device)
        self.model_path = model_path
        print(f"YOLO model loaded from {model_path} on {self.device.upper()}")

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Get device information
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device_rs = pipeline_profile.get_device()
        
        # Check for RGB camera
        found_rgb = False
        for s in device_rs.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            raise RuntimeError("The demo requires Depth camera with Color sensor")

        # Configure streams
        width, height = resolution
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Define pose skeleton connections
        self.skeleton_connections = [
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

    def start_streaming(self):
        """Start the RealSense pipeline"""
        self.pipeline.start(self.config)
        print("RealSense pipeline started")

    def stop_streaming(self):
        """Stop the RealSense pipeline"""
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("RealSense pipeline stopped")

    def draw_pose_keypoints(self, image, keypoints):
        """Draw pose keypoints and skeleton on the image"""
        if len(keypoints) == 0:
            return image
        
        # keypoints shape: [num_people, 17, 3] (x, y, confidence)
        for person_keypoints in keypoints:
            # Draw keypoints
            for i, (x, y, conf) in enumerate(person_keypoints):
                if conf > self.confidence_threshold:
                    cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)
                    cv2.putText(image, str(i), (int(x), int(y)-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            
            # Draw skeleton connections
            for connection in self.skeleton_connections:
                kpt1_idx, kpt2_idx = connection
                if (kpt1_idx < len(person_keypoints) and kpt2_idx < len(person_keypoints)):
                    kpt1 = person_keypoints[kpt1_idx]
                    kpt2 = person_keypoints[kpt2_idx]
                    
                    if kpt1[2] > self.confidence_threshold and kpt2[2] > self.confidence_threshold:
                        cv2.line(image, (int(kpt1[0]), int(kpt1[1])), 
                                (int(kpt2[0]), int(kpt2[1])), (255, 0, 0), 2)
        
        return image

    def draw_detection_boxes(self, image, results):
        """Draw bounding boxes and labels for object detection"""
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())
                    
                    if confidence > self.confidence_threshold:
                        # Get class name
                        class_name = result.names[class_id]
                        
                        # Draw bounding box
                        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        
                        # Draw label
                        label = f"{class_name}: {confidence:.2f}"
                        cv2.putText(image, label, (int(x1), int(y1-10)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image

    def process_frame(self, color_image):
        """Process a single frame with YOLO detection"""
        # Run YOLOv11 inference
        results = self.model(color_image, verbose=False)
        
        # Create annotated image
        annotated_image = color_image.copy()
        
        # Check if this is pose detection or object detection model
        if 'pose' in self.model_path:
            # Handle pose detection
            for result in results:
                if hasattr(result, 'keypoints') and result.keypoints is not None:
                    keypoints = result.keypoints.xy.cpu().numpy()  # Get (x, y) coordinates
                    confidences = result.keypoints.conf.cpu().numpy()  # Get confidence scores
                    
                    # Combine coordinates and confidences
                    keypoints_with_conf = np.concatenate([keypoints, confidences[..., np.newaxis]], axis=-1)
                    annotated_image = self.draw_pose_keypoints(annotated_image, keypoints_with_conf)
        else:
            # Handle object detection
            annotated_image = self.draw_detection_boxes(annotated_image, results)
        
        return annotated_image, results

    def run(self):
        """Main execution loop"""
        try:
            self.start_streaming()
            print("Starting RealSense capture with YOLOv11 detection...")
            print("Press 'q' to quit")
            
            while True:
                # Wait for frames
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Process with YOLO
                annotated_image, results = self.process_frame(color_image)

                # Create depth colormap
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Resize for display
                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = annotated_image.shape

                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(annotated_image, 
                                                   dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), 
                                                   interpolation=cv2.INTER_AREA)
                    display_image = np.hstack((resized_color_image, depth_colormap))
                else:
                    display_image = np.hstack((annotated_image, depth_colormap))

                # Add information overlay
                model_name = os.path.basename(self.model_path)
                cv2.putText(display_image, f"Model: {model_name}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_image, f"Device: {self.device.upper()}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_image, f"Confidence: {self.confidence_threshold}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_image, "Press 'q' to quit", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Display
                cv2.namedWindow('RealSense + YOLOv11', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense + YOLOv11', display_image)
                
                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.stop_streaming()

def main():
    args = parse_arguments()
    
    # Parse resolution
    width, height = map(int, args.resolution.split('x'))
    
    # Get model path
    model_path = MODEL_CONFIGS[args.model]
    
    print(f"Using model: {args.model}")
    print(f"Model path: {model_path}")
    print(f"Confidence threshold: {args.confidence}")
    print(f"Resolution: {width}x{height}")
    print(f"FPS: {args.fps}")
    
    # Create and run RealSenseYOLO instance
    rs_yolo = RealSenseYOLO(
        model_path=model_path,
        confidence_threshold=args.confidence,
        resolution=(width, height),
        fps=args.fps
    )
    
    rs_yolo.run()

if __name__ == "__main__":
    main()

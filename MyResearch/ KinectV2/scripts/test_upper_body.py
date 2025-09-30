#!/usr/bin/env python3
"""
Test script to verify upper body only detection
"""

import cv2
import numpy as np
from ultralytics import YOLO

# Test configuration
UPPER_BODY_KEYPOINTS = [5, 6, 7, 8, 9, 10]  # shoulders and arms only
SKELETON_CONNECTIONS = [
    (5, 6),   # left_shoulder -> right_shoulder
    (5, 7),   # left_shoulder -> left_elbow
    (7, 9),   # left_elbow -> left_wrist
    (6, 8),   # right_shoulder -> right_elbow
    (8, 10),  # right_elbow -> right_wrist
]

KEYPOINT_NAMES = [
    'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
    'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
]

def test_upper_body_detection():
    """Test upper body detection with webcam"""
    
    try:
        # Load YOLO model
        model = YOLO('yolo11n-pose.pt')
        print("✓ YOLO model loaded successfully")
        
        # Test with webcam
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("Could not open webcam. Skipping webcam test.")
            return
        
        print("Press 'q' to quit the test")
        print("Only shoulders and arms will be detected and displayed")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Run YOLO inference
            results = model(frame)
            
            # Draw upper body keypoints only
            for res in results:
                if hasattr(res, 'keypoints') and res.keypoints is not None:
                    pts = res.keypoints.data.cpu().numpy()
                    
                    for person_keypoints in pts:
                        # Draw skeleton connections for upper body only
                        for connection in SKELETON_CONNECTIONS:
                            kp1_idx, kp2_idx = connection
                            if kp1_idx < len(person_keypoints) and kp2_idx < len(person_keypoints):
                                x1, y1, conf1 = person_keypoints[kp1_idx]
                                x2, y2, conf2 = person_keypoints[kp2_idx]
                                
                                if conf1 > 0.3 and conf2 > 0.3:
                                    cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 3)
                        
                        # Draw keypoints for upper body only
                        for i, (x, y, conf) in enumerate(person_keypoints):
                            if i in UPPER_BODY_KEYPOINTS and conf > 0.3:
                                cv2.circle(frame, (int(x), int(y)), 8, (0, 255, 0), -1)
                                
                                # Label the keypoint
                                text = KEYPOINT_NAMES[i]
                                cv2.putText(frame, text, (int(x) + 10, int(y) - 10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow("Upper Body Detection Test", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print("✓ Upper body detection test completed successfully!")
        
    except Exception as e:
        print(f"Error during test: {e}")

if __name__ == "__main__":
    test_upper_body_detection()

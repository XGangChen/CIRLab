#!/usr/bin/env python3
"""
Test script to verify MediaPipe installation and basic functionality
"""

import cv2
import mediapipe as mp
import numpy as np

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

def test_mediapipe():
    """Test MediaPipe hand detection with webcam"""
    
    # Configure MediaPipe Hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5
    )
    
    print("MediaPipe Hands initialized successfully!")
    print("Press 'q' to quit the test")
    
    # Test with webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Could not open webcam. Testing with static image instead.")
        # Create a test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(test_image, "MediaPipe Test", (200, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Process the test image
        results = hands.process(cv2.cvtColor(test_image, cv2.COLOR_BGR2RGB))
        print("MediaPipe processing completed successfully!")
        
        cv2.imshow("MediaPipe Test", test_image)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()
    else:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = hands.process(rgb_frame)
            
            # Draw results
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style()
                    )
            
            cv2.imshow("MediaPipe Hand Test", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    hands.close()
    print("MediaPipe test completed successfully!")

if __name__ == "__main__":
    test_mediapipe()

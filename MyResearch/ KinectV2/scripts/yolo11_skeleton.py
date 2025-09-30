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

# MediaPipe for gesture recognition
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

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

# Initialize MediaPipe Hands and Gesture Recognition
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Configure MediaPipe Hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

print("MediaPipe Hands initialized for gesture recognition") 

# Define COCO pose skeleton connections - ONLY shoulders and arms
SKELETON_CONNECTIONS = [
    # Only arm and shoulder connections
    (5, 6),   # left_shoulder -> right_shoulder
    (5, 7),   # left_shoulder -> left_elbow
    (7, 9),   # left_elbow -> left_wrist
    (6, 8),   # right_shoulder -> right_elbow
    (8, 10),  # right_elbow -> right_wrist
    # Remove all body and leg connections
]

# COCO keypoint names for reference (keeping all for indexing purposes)
KEYPOINT_NAMES = [
    'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
    'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
]

# Upper body keypoints only (shoulders and arms)
UPPER_BODY_KEYPOINTS = [5, 6, 7, 8, 9, 10]  # left_shoulder, right_shoulder, left_elbow, right_elbow, left_wrist, right_wrist

# Upper body keypoint names for display
UPPER_BODY_NAMES = [
    'left_shoulder',   # index 5
    'right_shoulder',  # index 6
    'left_elbow',      # index 7
    'right_elbow',     # index 8
    'left_wrist',      # index 9
    'right_wrist'      # index 10
]

def recognize_gesture(hand_landmarks):
    """Simple gesture recognition based on hand landmarks"""
    if not hand_landmarks:
        return "No hand detected"
    
    # Get landmark positions
    landmarks = hand_landmarks.landmark
    
    # Helper function to check if finger is extended
    def is_finger_extended(tip_id, pip_id, mcp_id, landmarks):
        return landmarks[tip_id].y < landmarks[pip_id].y < landmarks[mcp_id].y
    
    # Check thumb (different logic due to orientation)
    def is_thumb_extended(landmarks):
        return landmarks[4].x > landmarks[3].x > landmarks[2].x
    
    # Count extended fingers
    fingers_up = []
    
    # Thumb
    fingers_up.append(is_thumb_extended(landmarks))
    
    # Other fingers
    finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky
    finger_pips = [6, 10, 14, 18]
    finger_mcps = [5, 9, 13, 17]
    
    for tip, pip, mcp in zip(finger_tips, finger_pips, finger_mcps):
        fingers_up.append(is_finger_extended(tip, pip, mcp, landmarks))
    
    # Count total fingers extended
    total_fingers = sum(fingers_up)
    
    # Gesture recognition
    if total_fingers == 0:
        return "Fist"
    elif total_fingers == 1 and fingers_up[1]:  # Only index finger
        return "Pointing"
    elif total_fingers == 2 and fingers_up[1] and fingers_up[2]:  # Index and middle
        return "Peace/Victory"
    elif total_fingers == 3 and fingers_up[1] and fingers_up[2] and fingers_up[3]:
        return "Three"
    elif total_fingers == 4 and not fingers_up[0]:  # All except thumb
        return "Four"
    elif total_fingers == 5:
        return "Open Hand"
    elif total_fingers == 1 and fingers_up[0]:  # Only thumb
        return "Thumbs Up"
    else:
        return f"{total_fingers} fingers"

def draw_hand_landmarks_and_gestures(img, results, depth_frame):
    """Draw hand landmarks and recognize gestures with depth information"""
    if not results.multi_hand_landmarks:
        return
    
    depth_array = depth_frame.asarray()
    
    for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
        # Draw hand landmarks
        mp_drawing.draw_landmarks(
            img, hand_landmarks, mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style()
        )
        
        # Recognize gesture
        gesture = recognize_gesture(hand_landmarks)
        
        # Get wrist position for text placement
        wrist = hand_landmarks.landmark[0]
        h, w, _ = img.shape
        wrist_x, wrist_y = int(wrist.x * w), int(wrist.y * h)
        
        # Get depth at wrist position
        depth_x = int(wrist_x * depth_array.shape[1] / w)
        depth_y = int(wrist_y * depth_array.shape[0] / h)
        
        if 0 <= depth_x < depth_array.shape[1] and 0 <= depth_y < depth_array.shape[0]:
            depth_value = depth_array[depth_y, depth_x]
            
            # Display gesture and depth
            text = f"Hand {hand_idx + 1}: {gesture} ({depth_value:.0f}mm)"
            
            # Position text
            text_y = wrist_y - 30 - (hand_idx * 60)  # Offset for multiple hands
            if text_y < 30:
                text_y = wrist_y + 30 + (hand_idx * 60)
            
            # Draw background for text
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            cv2.rectangle(img, (wrist_x - 5, text_y - text_size[1] - 5), 
                         (wrist_x + text_size[0] + 5, text_y + 5), (0, 0, 0), -1)
            
            # Draw text
            cv2.putText(img, text, (wrist_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

def draw_upper_body_skeleton(img, keypoints, depth_frame, confidence_threshold=0.9):
    """Draw upper body skeleton connections and keypoints on the image with depth information - shoulders and arms only"""
    # Get depth array
    depth_array = depth_frame.asarray()
    
    # Draw skeleton connections
    for connection in SKELETON_CONNECTIONS:
        kp1_idx, kp2_idx = connection
        if kp1_idx < len(keypoints) and kp2_idx < len(keypoints):
            x1, y1, conf1 = keypoints[kp1_idx]
            x2, y2, conf2 = keypoints[kp2_idx]
            
            # Only draw if both keypoints have sufficient confidence
            if conf1 > confidence_threshold and conf2 > confidence_threshold:
                cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 5)
    
    # Draw keypoints with depth information (upper body keypoints only)
    for i, (x, y, conf) in enumerate(keypoints):
        # Only show shoulders and arms (5-10: left_shoulder, right_shoulder, left_elbow, right_elbow, left_wrist, right_wrist)
        if i not in UPPER_BODY_KEYPOINTS or conf <= confidence_threshold:
            continue
            
        # Color for upper body parts (arms and shoulders)
        color = (0, 255, 0)    # Green for all upper body parts
        
        # Draw keypoint
        cv2.circle(img, (int(x), int(y)), 10, color, -1)

        # Get depth value at keypoint location
        # Scale coordinates from color frame to depth frame
        depth_x = int(x * depth_array.shape[1] / img.shape[1])
        depth_y = int(y * depth_array.shape[0] / img.shape[0])
        
        # Ensure coordinates are within depth frame bounds
        if 0 <= depth_x < depth_array.shape[1] and 0 <= depth_y < depth_array.shape[0]:
            depth_value = depth_array[depth_y, depth_x]
            depth_mm = depth_value  # Depth is in millimeters
            
            # Display depth information
            text = f"{KEYPOINT_NAMES[i]}: {depth_mm:.0f}mm"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
            
            # Position text to avoid overlap
            text_x = int(x) + 15
            text_y = int(y) - 10
            
            # Adjust text position if it goes out of bounds
            if text_x + text_size[0] > img.shape[1]:
                text_x = int(x) - text_size[0] - 15
            if text_y < text_size[1]:
                text_y = int(y) + text_size[1] + 15
            
            # Draw background for text
            cv2.rectangle(img, (text_x - 3, text_y - text_size[1] - 3), 
                         (text_x + text_size[0] + 3, text_y + 3), (0, 0, 0), -1)
            
            # Draw text
            cv2.putText(img, text, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
kinect_device = fn.openDevice(serial, pipeline=pipeline)

# Set up multi-frame listener for Color, IR, and Depth
listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)
kinect_device.setColorFrameListener(listener)
kinect_device.setIrAndDepthFrameListener(listener)
kinect_device.start()

# Registration must be created after device.start()
registration = Registration(
    kinect_device.getIrCameraParams(), kinect_device.getColorCameraParams())

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

    # Draw detected keypoints and skeleton with depth information (upper body only)
    for res in results:
        if hasattr(res, 'keypoints') and res.keypoints is not None:
            pts = res.keypoints.data.cpu().numpy()
            # Process keypoints for each person detected
            for person_keypoints in pts:
                # Draw upper body skeleton and keypoints with depth for this person
                draw_upper_body_skeleton(img_rgb, person_keypoints, depth, confidence_threshold=0.3)

    # Run MediaPipe hand detection and gesture recognition
    hand_results = hands.process(img_rgb)
    
    # Draw hand landmarks and recognize gestures
    if hand_results.multi_hand_landmarks:
        draw_hand_landmarks_and_gestures(img_rgb, hand_results, depth)

    # Prepare display frame (RGB->BGR)
    display_frame = img_rgb[:, :, ::-1]

    # Show views
    # cv2.imshow("ir", ir.asarray() / 65535.)
    cv2.imshow("depth", depth.asarray() / 4500.)
    cv2.imshow("color", cv2.resize(display_frame, (1280, 720)))
    # cv2.imshow("registered", registered.asarray(np.uint8))
    if need_bigdepth:
        bd = bigdepth.asarray(np.float32)
        cv2.imshow("bigdepth", cv2.resize(bd, (640, 360)))
    if need_color_depth_map:
        cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

        # Convert to uint8 or float32 for display
        cdm_display = (color_depth_map.reshape(424, 512) / color_depth_map.max() * 255).astype(np.uint16)
        cv2.imshow("color_depth_map", cdm_display)

    listener.release(frames)
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
hands.close()
kinect_device.stop()
kinect_device.close()
cv2.destroyAllWindows()
sys.exit(0)
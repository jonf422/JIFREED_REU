import cv2
import numpy as np
import os
import time
import urllib.request
from ultralytics import YOLO
import mediapipe as mp

MODEL_PATH = "pose_landmarker_full.task"
if not os.path.exists(MODEL_PATH):
    print("Downloading pose landmarker model...")
    url = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"
    urllib.request.urlretrieve(url, MODEL_PATH)
    print("Done.")

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

yolo_model = YOLO("yolo26n.pt")

# MediaPipe pose landmark indices
NOSE        = 0
L_SHOULDER  = 11
R_SHOULDER  = 12
L_WRIST     = 15
R_WRIST     = 16
L_HIP       = 23
R_HIP       = 24
L_KNEE      = 25
R_KNEE      = 26
L_ANKLE     = 27
R_ANKLE     = 28

POSE_CONNECTIONS = [
    (11,12),(11,13),(13,15),(12,14),(14,16),
    (11,23),(12,24),(23,24),
    (23,25),(25,27),(24,26),(26,28),
    (27,29),(28,30),(29,31),(30,32),
    (0,1),(1,2),(2,3),(3,7),
    (0,4),(4,5),(5,6),(6,8),(9,10),
]

def draw_pose(frame, landmarks, h, w):
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in POSE_CONNECTIONS:
        if a < len(pts) and b < len(pts):
            cv2.line(frame, pts[a], pts[b], (0, 200, 0), 2)
    for pt in pts:
        cv2.circle(frame, pt, 4, (255, 255, 255), -1)

def classify_pose(lm):
    """
    Classify pose using normalized landmark positions.
    In MediaPipe, y=0 is top of frame, y=1 is bottom (y increases downward).
    So 'higher' in the image means smaller y value.
    """
    shoulder_y = (lm[L_SHOULDER].y + lm[R_SHOULDER].y) / 2
    hip_y      = (lm[L_HIP].y    + lm[R_HIP].y)    / 2
    knee_y     = (lm[L_KNEE].y   + lm[R_KNEE].y)   / 2
    ankle_y    = (lm[L_ANKLE].y  + lm[R_ANKLE].y)  / 2

    # Arms up: wrists are above (smaller y than) shoulders
    both_arms_up = (lm[L_WRIST].y < lm[L_SHOULDER].y - 0.05 and
                    lm[R_WRIST].y < lm[R_SHOULDER].y - 0.05)
    one_arm_up   = (lm[L_WRIST].y < lm[L_SHOULDER].y - 0.05 or
                    lm[R_WRIST].y < lm[R_SHOULDER].y - 0.05)

    # Lying down: small vertical span from nose to ankles
    body_span = ankle_y - lm[NOSE].y
    lying_down = body_span < 0.35

    # Crawling: hips and knees at similar height, not lying flat
    hip_knee_diff = abs(hip_y - knee_y)
    crawling = hip_knee_diff < 0.12 and hip_y > 0.4 and not lying_down

    if both_arms_up:
        return "ARMS UP", (0, 0, 255)        # red  - suspicious
    elif crawling:
        return "CRAWLING", (0, 0, 255)        # red  - suspicious
    elif lying_down:
        return "LYING DOWN", (0, 100, 255)    # orange - suspicious
    elif one_arm_up:
        return "ONE ARM UP", (0, 255, 255)    # yellow - watch
    else:
        return "NORMAL", (0, 220, 0)          # green  - ok


options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=VisionRunningMode.VIDEO,
    num_poses=5,
    min_pose_detection_confidence=0.5,
    min_pose_presence_confidence=0.5,
    min_tracking_confidence=0.5,
)

VIDEO_PATH = 'lunges.mp4'
cap = cv2.VideoCapture(VIDEO_PATH)
start_time = time.time()

with PoseLandmarker.create_from_options(options) as landmarker:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        timestamp_ms = int((time.time() - start_time) * 1000)

        # YOLO person detection
        yolo_results = yolo_model(frame, classes=[0], verbose=False)
        for box in yolo_results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
            cv2.putText(frame, f"person {conf:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)

        # MediaPipe pose estimation
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result = landmarker.detect_for_video(mp_image, timestamp_ms)

        if result.pose_landmarks:
            for landmarks in result.pose_landmarks:
                draw_pose(frame, landmarks, h, w)

                label, color = classify_pose(landmarks)

                # Draw label near the shoulders
                label_x = int(landmarks[L_SHOULDER].x * w)
                label_y = int(landmarks[L_SHOULDER].y * h) - 20
                cv2.putText(frame, label, (label_x, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        cv2.imshow("Pose Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

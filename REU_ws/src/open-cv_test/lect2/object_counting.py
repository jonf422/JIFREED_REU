import cv2
from ultralytics import YOLO

path = 'skiers.mp4'
cap = cv2.VideoCapture(path)

model = YOLO('yolo26n')
unique_ids = set()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        pass

    results = model.track(frame, classes=[0], persist=True, verbose=False)

    annotated_frame = results[0].plot()

    for r in results:
        
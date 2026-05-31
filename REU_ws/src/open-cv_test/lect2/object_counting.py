import cv2
from ultralytics import YOLO
import numpy

path = 'skiers.mp4'
cap = cv2.VideoCapture(path)

model = YOLO('yolo26n.pt')
unique_ids = set()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        pass

    results = model.track(frame, classes=[0], persist=True, verbose=False)
    annotated_frame = results[0].plot()

    if results[0].boxes and results[0].boxes.id is not None:
        ids = numpy.asarray(results[0].boxes.id)
        for oid in ids:
            unique_ids.add(oid)
        cv2.putText(annotated_frame, f'Count: {len(unique_ids)}', (10,30), cv2.FONT_ITALIC, 1, (0,255,0), 2)
    cv2.imshow('Object Counting', annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == 27:  # press ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
        
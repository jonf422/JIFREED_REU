import cv2
from ultralytics import YOLO
import numpy
from collections import defaultdict, deque

path = 'skiers.mp4'
cap = cv2.VideoCapture(path)

model = YOLO('yolo26n')

id_map = {}
next_id = 1

trails = defaultdict(lambda: deque(maxlen=30))
appear = defaultdict(int)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        pass

    result = model.track(frame, classes=[0], persist=True, verbose=False)
    annotated_frame = frame.copy()

    if result[0].boxes and result[0].boxes.id is not None:
        boxes = numpy.asarray(result[0].boxes.xyxy)
        ids = numpy.asarray(result[0].boxes.id)

        for box,oid in zip(boxes, ids):
            x1,y1,x2,y2 = map(int, box)
            cx, cy = (x1+x2)//2, (y1+y2)//2

            appear[oid] += 1

            if appear[oid] >= 5 and oid not in id_map:
                id_map[oid] = next_id
                next_id += 1

            if oid in id_map:
                sid = id_map[oid]
                trails[oid].append((cx, cy))
                cv2.rectangle(annotated_frame, (x1,y1), (x2,y2), (0,0,255), 2)
                cv2.putText(annotated_frame, f'id:{sid}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,0,255), 2)

                cv2.circle(annotated_frame, (cx, cy), 3, (0,0,255), -1)
                trail_pts = list(trails[oid])
                for i in range(1, len(trail_pts)):
                    cv2.line(annotated_frame, trail_pts[i-1], trail_pts[i], (0,0,255), 1)
                

    cv2.imshow('Tracking', annotated_frame)

    if cv2.waitKey(1) & 0xFF == 27:  # press ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
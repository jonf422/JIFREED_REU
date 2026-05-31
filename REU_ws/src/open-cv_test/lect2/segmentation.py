import cv2
from ultralytics import YOLO
import numpy

path = 'walking.mp4'
cap = cv2.VideoCapture(path)

model = YOLO('yolo26n-seg.pt')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    result = model.track(source=frame, classes=[0], persist=True, verbose=False)
    annotated_frame = frame.copy()

    for r in result:

        if r.masks and r.boxes  and r.boxes.id is not None:
            masks = numpy.asarray(r.masks.data)
            boxes = numpy.asarray(r.boxes.xyxy)
            ids = numpy.asarray(r.boxes.id)

            for i, mask in enumerate(masks):
                pers_id = ids[i]
                x1,y1,x2,y2 = boxes[i]

                resise_mask = cv2.resize(mask.astype(numpy.uint8)*255, (frame.shape[1], frame.shape[0]))
                contours, _ = cv2.findContours(resise_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(annotated_frame, contours, -1, (0,0,255), 1)
                cv2.putText(annotated_frame, f'ID: {pers_id}', (int(x1),int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,0,255), 2)
    
    cv2.imshow('Segmentation', annotated_frame)

    if cv2.waitKey(1) & 0xFF == 27:  # press ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
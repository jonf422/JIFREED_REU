import cv2
from ultralytics import YOLO

# Initialize YOLO model
model = YOLO("yolo26n.pt")

# Open video file or camera stream ("0" for webcam)
video_path = "skiers.mp4" 
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        pass

    # Run YOLO object detection on the frame
    results = model(frame, classes=[0])
    
    #annotate bounding box onto frame
    annotated_img = results[0].plot()

    # Display the frame
    cv2.imshow("Video", annotated_img)

    if cv2.waitKey(1) & 0xFF == 27:  # press ESC to exit
        break

cap.release()
cv2.destroyAllWindows()

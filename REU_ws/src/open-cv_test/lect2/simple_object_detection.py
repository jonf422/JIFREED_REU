import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO('yolo26n.pt')

image = cv2.imread('motorcycle.jpg')
image = cv2.resize(image, (640,480))

results = model(image)

annotated_image = results[0].plot()

cv2.imshow('Annotated Image', annotated_image)

cv2.waitKey(0)
cv2.destroyAllWindows()
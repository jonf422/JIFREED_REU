import cv2

#read in image
img = cv2.imread('dandelion.jpg')

#resize
resize = cv2.resize(img, (640, 480))

#grayscale
gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY)

#blurring
blur = cv2.GaussianBlur(resize, (3,3), 0)

#edge detection using Canny algorithm
edge = cv2.Canny(resize, 100, 200)

#show processed imgs
cv2.imshow('Resize', resize)
cv2.imshow('Gray', gray)
cv2.imshow('Blur', blur)
cv2.imshow('Edge', edge)
cv2.waitKey(1)
cv2.destroyAllWindows()

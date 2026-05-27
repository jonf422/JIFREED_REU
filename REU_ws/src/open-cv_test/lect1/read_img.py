import cv2

#read image
img = cv2.imread('dandelion.jpg')

#show image in window
cv2.imshow('dandelion_img', img)
cv2.waitKey(0)

#save copy of image
cv2.imwrite('saved_img.jpg', img)
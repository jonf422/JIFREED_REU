import cv2
import numpy as np

#create 512x512x3 array (3->b,g,r)
canvas = np.zeros((512,512,3), np.uint8)

#draw line: canvas, (pt1)->(pt2), (b,g,r), (line_thickness)
cv2.line(canvas, (0,0), (511,511), (0,0,255), 3)

#draw rect: canvas, (top_left)->(bot_right), (b,g,r), (line_thickness)
cv2.rectangle(canvas, (411,0), (511,100), (0,255,0), 3)
cv2.rectangle(canvas, (0,411), (100,511), (255,0,0), -1)

#draw text: canvas, "text", (top_left), cv2.FONT, font scale, (b,g,r), line thickness 
cv2.putText(canvas, "Annotated Text", (10,500), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)


cv2.imshow("Canvas", canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()


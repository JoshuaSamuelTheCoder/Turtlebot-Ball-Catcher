import numpy as np
import cv2


cam = cv2.VideoCapture(0)

while True:
	ret, frame = cam.read()
	img = cv2.GaussianBlur(frame,(25,25),0)
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	lower_range = np.array([150, 50, 50],dtype = np.uint8)
	upper_range = np.array([200, 255, 255], dtype = np.uint8)

	img = cv2.inRange(hsv, lower_range, upper_range)
	cv2.imshow("Image", img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
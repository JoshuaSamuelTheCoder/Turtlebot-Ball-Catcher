import cv2
import numpy as np
#import cv2 as cv
cap = cv2.VideoCapture(0)

coords = []
while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV 
    lower_red = np.array([160,140,50]) 
    upper_red = np.array([180,255,255])

    imgThreshHigh = cv2.inRange(hsv, lower_red, upper_red)
    thresh = imgThreshHigh.copy()

    countours= cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in countours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt

    M = cv2.moments(best_cnt)
    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    coord = cx, cy #This are your coordinates for the circle
    # area = moments['m00'] save the object area
    #perimeter = cv2.arcLength(best_cnt,True) is the object perimeter

    #Save the coords every frame on a list
    #Here you can make more conditions if you don't want repeated coordinates
    points.append(coord) 

    cv2.imshow('frame',frame)
    cv2.imshow('Object',thresh)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
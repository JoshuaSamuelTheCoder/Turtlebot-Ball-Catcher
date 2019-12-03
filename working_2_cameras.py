import numpy as np
import cv2
import threading
from threading import Thread, Event, ThreadError

hue = 10
sat = 10
val = 10
hue2 = 180
sat2 = 255  
val2 = 255

class Cam():
  def nothing():
    pass
  def run(self):

    points = []
    global hue
    global sat
    global val
    cap = cv2.VideoCapture(0)

    while True:
      try:
        _,frame = cap.read()

        h = cv2.getTrackbarPos('H','Objeto')
        s = cv2.getTrackbarPos('S','Objeto')
        v = cv2.getTrackbarPos('V','Objeto')
        h2 = cv2.getTrackbarPos('H','Camera')
        s2 = cv2.getTrackbarPos('S','Camera')
        v2 = cv2.getTrackbarPos('V','Camera')

        lower = np.array([h,s,v])
        upper = np.array([h2,s2,v2])

        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv2 = hsv.copy()
        thresh = cv2.inRange(hsv,lower, upper)
        thresh = cv2.medianBlur(thresh,7)
        thresh2 = thresh.copy()

        inverted_image = cv2.bitwise_not(thresh2)
        kernel = np.ones((3,3),np.uint8)
        erosion = cv2.erode(inverted_image,kernel,iterations = 7)
        dilation = cv2.dilate(erosion,kernel,iterations = 15)
        #opening = cv2.morphologyEx(inverted_image, cv2.MORPH_OPEN, kernel)
        #dilation_2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        edged = cv2.Canny(dilation, 30, 200) 
        
        _, contours, _ = cv2.findContours(edged,  
                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        print(len(contours))
        #cv2.drawContours(dilation_2, contours, -1, (0, 255, 0), -1) 
        for c in contours:
          x,y,w,h = cv2.boundingRect(c)
          cv2.rectangle(dilation, (x, y), (x+w, y+h), (255, 0, 255), 2)
        cv2.imshow('Objeto', dilation)

        if cv2.waitKey(1) ==1048603:
          exit(0)
          f.close()

      except ThreadError:
        self.thread_cancelled = True

  cv2.namedWindow('Camera')
  cv2.namedWindow('Objeto')
  cv2.createTrackbar('H', 'Objeto', hue, 180, nothing)
  cv2.createTrackbar('S', 'Objeto', sat, 255, nothing)
  cv2.createTrackbar('V', 'Objeto', val, 255, nothing)
  cv2.createTrackbar('H', 'Camera', hue2, 180, nothing)
  cv2.createTrackbar('S', 'Camera', sat2, 255, nothing)
  cv2.createTrackbar('V', 'Camera', val2, 255, nothing)
        
 
  
    
if __name__ == "__main__":
  cam = Cam()
  cam.run()
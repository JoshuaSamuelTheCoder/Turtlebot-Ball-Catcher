import numpy as np
import cv2
import threading
from threading import Thread, Event, ThreadError
import sys

hue = 10
sat = 10
val = 10
hue2 = 180
sat2 = 255  
val2 = 255
camera_num = int(sys.argv[1])
camera_string = 'Camera_' + str(camera_num)
objeto_string = 'Objeto_' + str(camera_num)

class Cam():
  def nothing():
    pass
  def run(self):
    points = []
    global hue
    global sat
    global val
   
    cap = cv2.VideoCapture(camera_num)

    while True:
      try:
        _,frame = cap.read()

        h = cv2.getTrackbarPos('H',objeto_string)
        s = cv2.getTrackbarPos('S',objeto_string)
        v = cv2.getTrackbarPos('V',objeto_string)
        h2 = cv2.getTrackbarPos('H',camera_string)
        s2 = cv2.getTrackbarPos('S',camera_string)
        v2 = cv2.getTrackbarPos('V',camera_string)

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
        cv2.imshow(objeto_string, dilation)

        if cv2.waitKey(1) ==1048603:
          exit(0)
          f.close()

      except ThreadError:
        self.thread_cancelled = True

  cv2.namedWindow(camera_string)
  cv2.namedWindow(objeto_string)
  cv2.createTrackbar('H', objeto_string, hue, 180, nothing)
  cv2.createTrackbar('S', objeto_string, sat, 255, nothing)
  cv2.createTrackbar('V', objeto_string, val, 255, nothing)
  cv2.createTrackbar('H', camera_string, hue2, 180, nothing)
  cv2.createTrackbar('S', camera_string, sat2, 255, nothing)
  cv2.createTrackbar('V', camera_string, val2, 255, nothing)
        
 
  
    
if __name__ == "__main__":
  cam = Cam()
  cam.run()
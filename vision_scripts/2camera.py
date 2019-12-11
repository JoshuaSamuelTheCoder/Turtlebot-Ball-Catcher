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

global_hue = 3
global_sat = 0
global_val = 0

global_max_hue = 255
global_max_sat1 = 189
global_max_sat2 = 223
global_max_value = 255



class Cam():
  def nothing():
    pass
  def run(self):
    points = []
    global hue
    global sat
    global val
    #189
    #223

    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(1)

    while True:
      try:
        _,frame0 = cap0.read()
        _,frame1 = cap1.read()

        s = cv2.getTrackbarPos('Max Sat',objeto_string)
        if s == 0:
        	s = global_max_sat2

        lower = np.array([global_hue,global_sat,global_val])
        upper_left = np.array([global_max_hue,global_max_sat1,global_max_value])
        upper_right = np.array([global_max_hue,s,global_max_value])

        frame0_hsv = cv2.cvtColor(frame0,cv2.COLOR_BGR2HSV)
        frame0_hsv2 = frame0_hsv.copy()
        frame0_thresh = cv2.inRange(frame0_hsv,lower, upper_left)
        frame0_thresh = cv2.medianBlur(frame0_thresh,7)
        frame0_thresh2 = frame0_thresh.copy()

        frame0_inverted_image = cv2.bitwise_not(frame0_thresh2)
        frame0_kernel = np.ones((2,2),np.uint8)
        frame0_erosion = cv2.erode(frame0_inverted_image,frame0_kernel,iterations = 7)
        frame0_dilation = cv2.dilate(frame0_erosion,frame0_kernel,iterations = 15)

        frame0_edged = cv2.Canny(frame0_dilation, 30, 200)         
        _,frame0_contours,_ = cv2.findContours(frame0_edged,  
                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
  		
        maxArea = 0
        max_x = 0
        max_y = 0
        max_w = 0
        max_h = 0
        for c in frame0_contours:
          frame0_x,frame0_y,frame0_w,frame0_h = cv2.boundingRect(c)
          area = frame0_w*frame0_h
          if area > maxArea:
          	max_x = frame0_x
          	max_y = frame0_y
          	max_w = frame0_w
          	max_h = frame0_h
        
        cv2.rectangle(frame0_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 1", frame0_dilation)

        #camera 2:

        frame1_hsv = cv2.cvtColor(frame1,cv2.COLOR_BGR2HSV)
        frame1_hsv2 = frame1_hsv.copy()
        frame1_thresh = cv2.inRange(frame1_hsv,lower, upper_right)
        frame1_thresh = cv2.medianBlur(frame1_thresh,7)
        frame1_thresh2 = frame1_thresh.copy()

        frame1_inverted_image = cv2.bitwise_not(frame1_thresh2)
        frame1_kernel = np.ones((3,3),np.uint8)
        frame1_erosion = cv2.erode(frame1_inverted_image,frame1_kernel,iterations = 7)
        frame1_dilation = cv2.dilate(frame1_erosion,frame1_kernel,iterations = 15)

        frame1_edged = cv2.Canny(frame1_dilation, 30, 200)         
        _, frame1_contours,_ = cv2.findContours(frame1_edged,  
                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
  
        
        maxArea = 0
        max_x = 0
        max_y = 0
        max_w = 0
        max_h = 0
        for c in frame1_contours:
          frame1_x,frame1_y,frame1_w,frame1_h = cv2.boundingRect(c)
          area = frame1_w*frame1_h
          if area > maxArea:
          	max_x = frame1_x
          	max_y = frame1_y
          	max_w = frame1_w
          	max_h = frame1_h
        
        cv2.rectangle(frame1_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 2", frame1_dilation)


        if cv2.waitKey(1) ==1048603:
          exit(0)
          f.close()

      except ThreadError:
        self.thread_cancelled = True

  #cv2.namedWindow(camera_string)
  cv2.namedWindow(objeto_string)

  #cv2.createTrackbar('H', objeto_string, global_hue, 180, nothing)
  cv2.createTrackbar('Max Sat', objeto_string, global_sat, 255, nothing)
  #cv2.createTrackbar('V', objeto_string, global_val, 255, nothing)
  #cv2.createTrackbar('H', camera_string, global_max_hue, 180, nothing)
  #cv2.createTrackbar('S', camera_string, global_max_sat, 255, nothing)
  #cv2.createTrackbar('V', camera_string, global_max_value, 255, nothing)
        
  #Hue: 7
  #S:0
  #V: 0

  #Max Hue: 180
  #Max S: 133
  #V: 255
    
if __name__ == "__main__":
  cam = Cam()
  input = int(sys.argv[1])
  cam.run()
  #t1 = threading.Thread(target= cam.run, args=(input,))

  #t1.start()
  #t1.join()
  #cam.run(int(sys.argv[1]))
  #cam.run(int(sys.argv[2]))
  #if(len(sys.argv) == 3):
  #	cam.run(sys.argv[2])

  #starting fuc

import rospy
import numpy as np
import cv2
import threading
from threading import Thread, Event, ThreadError
import sys
import pprint

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from mpl_toolkits import mplot3d
#matplotlib inline
import matplotlib.pyplot as plt

hue = 10
sat = 10
val = 10
hue2 = 180
sat2 = 159  
val2 = 255
e_its = 10
d_its = 14
kern = 7

global_hue = 3
global_sat = 0
global_val = 0

global_max_hue = 255
global_max_sat1 = 199
global_max_sat2 = 224
global_max_value = 255
#189
#224

left_max_x = 0
left_max_y = 0
right_max_x = 0
right_max_y = 0


camera_string = 'Camera_'
objeto_string = 'Objeto_'


class Cam():
  #points_graph = np.array([0,0,0]).reshape((3,1))
  #counter = 0

  def nothing():
    pass


  def update_left(self,msg):
    self.frame0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

  def update_right(self,msg):
    self.frame1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

  def findBall(self):
    #points = []
    global hue
    global sat
    global val
    points = []
    global hue
    global sat
    global val
    self.frame0 = None

    rate = rospy.Time(30)

    self.bridge = CvBridge()
    rospy.Subscriber('/camera_left/left/image_raw', Image, self.update_left)
    #rospy.Subscriber('/camera_right/right/image_raw', Image, self.update_right)
    #pub = rospy.Publisher('/ball_position', PointStamped, queue_size=10)
    rospy.init_node('findBall')

    while True:
      # if not self.frame0:
      #   print(":(")
      #   continue
      try:
        frame = self.frame0

        h = cv2.getTrackbarPos('H',objeto_string)
        s = cv2.getTrackbarPos('S',objeto_string)
        v = cv2.getTrackbarPos('V',objeto_string)
        h2 = cv2.getTrackbarPos('H',camera_string)
        s2 = cv2.getTrackbarPos('S',camera_string)
        v2 = cv2.getTrackbarPos('V',camera_string)
        e_its = cv2.getTrackbarPos('Erosion',camera_string)
        d_its = cv2.getTrackbarPos('Dilation',camera_string)
        kern = cv2.getTrackbarPos('Kernel', camera_string)

        lower = np.array([h,s,v])
        upper = np.array([h2,s2,v2])

        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv2 = hsv.copy()
        thresh = cv2.inRange(hsv,lower, upper)
        thresh = cv2.medianBlur(thresh,7)
        thresh2 = thresh.copy()

        inverted_image = cv2.bitwise_not(thresh2)
        kernel = np.ones((kern,kern),np.uint8)
        erosion = cv2.erode(inverted_image,kernel,iterations = e_its)
        dilation = cv2.dilate(erosion,kernel,iterations = d_its)
        #opening = cv2.morphologyEx(inverted_image, cv2.MORPH_OPEN, kernel)
        #dilation_2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        edged = cv2.Canny(dilation, 30, 200) 
        
        _, contours, _ = cv2.findContours(edged,  
                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

        #cv2.drawContours(dilation_2, contours, -1, (0, 255, 0), -1) 
        for c in contours:
          x,y,w,h = cv2.boundingRect(c)
          cv2.rectangle(dilation, (x, y), (x+w, y+h), (255, 0, 255), 2)
          print("center", (x, y))
        cv2.imshow(objeto_string, dilation)

        if cv2.waitKey(1) == 1048603:
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
  cv2.createTrackbar('Erosion', camera_string, e_its, 255, nothing)
  cv2.createTrackbar('Dilation', camera_string, d_its, 255, nothing)
  cv2.createTrackbar('Kernel', camera_string, kern, 255, nothing)
    #plot the 3D points

  #cv2.namedWindow(objeto_string)

  #cv2.createTrackbar('Max Sat', objeto_string, global_sat, 255, nothing)

    
if __name__ == "__main__":
  cam = Cam()
  cam.findBall()



"""
At time 1576037452.137
- Translation: [0.401, -0.761, 1.816]
- Rotation: in Quaternion [0.956, 0.034, -0.232, -0.178]
            in RPY (radian) [-2.735, 0.446, 0.165]
            in RPY (degree) [-156.729, 25.529, 9.471]


At time 1576037490.037
- Translation: [-0.592, -0.513, 1.596]
- Rotation: in Quaternion [0.955, -0.043, 0.261, -0.135]
            in RPY (radian) [-2.815, -0.508, -0.175]
            in RPY (degree) [-161.261, -29.107, -10.051]
source: target
"""
"""
left: ar
At time 1576037712.126
- Translation: [0.483, 0.150, 1.465]
- Rotation: in Quaternion [0.983, -0.024, -0.179, -0.019]
            in RPY (radian) [-3.111, 0.360, -0.043]
            in RPY (degree) [-178.270, 20.613, -2.485]
right :ar

At time 1576037761.638
- Translation: [-0.135, 0.127, 1.588]
- Rotation: in Quaternion [0.966, -0.056, 0.253, 0.015]
            in RPY (radian) [3.142, -0.512, -0.116]
            in RPY (degree) [179.998, -29.325, -6.643]

"""


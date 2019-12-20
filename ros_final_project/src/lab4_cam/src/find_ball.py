import rospy
import numpy as np
import cv2
import threading
from threading import Thread, Event, ThreadError
import sys

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from mpl_toolkits import mplot3d
#matplotlib inline
import matplotlib.pyplot as plt
from tf.transformations import quaternion_matrix

#Left values
# hue = 10
# sat = 10
# val = 10
# hue2 = 180
# sat2 = 154  
# val2 = 255
global_hue_l = 3
global_sat_l = 0
global_val_l = 0

global_max_hue_l = 180
global_max_sat_l = 181
global_max_val_l = 255

global_max_hue_r = 180
global_max_sat_r= 154
global_max_val_r = 255

e_its_l = 10
d_its_l = 14
kern_l = 7

e_its_r = 10
d_its_r = 14
kern_r = 7
#-0.241, 0.874, 4.583
#-0.648, 0.665, -0.363, -0.080

#Translation: [0.211, 1.034, 3.595]
#- Rotation: in Quaternion [0.668, -0.605, 0.331, 0.279]

#189
#224

left_max_x = 0
left_max_y = 0
right_max_x = 0
right_max_y = 0


class Cam():
  #points_graph = np.array([0,0,0]).reshape((3,1))
  #counter = 0

  def nothing():
    pass

  def get_3d_coords(self, left_max_x, left_max_y, right_max_x, right_max_y, imageSize):

    CMatr1 = np.matrix([[954.855635, 0.000000, 624.429606],
    [0.000000, 952.447679, 369.699299],
    [0.000000, 0.000000, 1.000000]]).astype(np.float)

    CMatr2 = np.matrix([[974.873243, 0.000000, 661.429672],
      [0.000000, 977.204120, 380.550973],
      [0.000000, 0.000000, 1.000000]]).astype(np.float)

    projPoints1 = np.array([[left_max_x],[left_max_y]]).astype(np.float)

    projPoints2 = np.array([[right_max_x],[right_max_y]]).astype(np.float)

    distort_left = np.array([0.039887, -0.113282, 0.000864, 0.000831, 0.000000]).astype(np.float)
    distort_right = np.array([0.010096, -0.070900, 0.002032, -0.013537, 0.000000]).astype(np.float)

    R_lt = quaternion_matrix(np.array([0.588, -0.529, 0.390, 0.471]))
    # R_lt = quaternion_matrix(np.array([0.596, -0.519, 0.382, -0.479]))
    R_rt = quaternion_matrix(np.array([-0.352, 0.738, -0.496, -0.292]))
    # R_rt = quaternion_matrix(np.array([-0.511, 0.746, 0.202, -0.375]))

    R_lt = R_lt[:3,:3]
    R_rt = R_rt[:3,:3]
    R_tl = np.linalg.inv(R_lt)
    R_tr = np.linalg.inv(R_rt)
    T_lt_l = np.array([-0.027, 0.832, 3.088]).astype(np.float) #--------------------CHANGE
    # T_lt_l = np.array([-2.735, -0.500, 1.565]).astype(np.float) #--------------------CHANGE
    T_rt_r = np.array([-0.750, 0.782, 3.081]).astype(np.float) #--------------------CHANGE
    # T_rt_r = np.array([2.688, -0.886, 1.726]).astype(np.float) #--------------------CHANGE

    g_lt_1 = np.hstack((R_lt, T_lt_l.reshape((3,1)))) 
    g_lt_2 = np.array([0,0,0,1])
    g_lt = np.vstack((g_lt_1,g_lt_2.T))
    g_tl = np.linalg.inv(g_lt)


    g_rt_1 = np.hstack((R_rt, T_rt_r.reshape((3,1))))
    g_rt_2 = np.array([0,0,0,1])
    g_rt = np.vstack((g_rt_1,g_rt_2.T)) 
    g_tr = np.linalg.inv(g_rt)

    g_lr = np.dot(g_lt,g_tr)

    R_lr = np.dot(R_lt,R_tr)

    R1,R2,P1,P2,Q, a,b = cv2.stereoRectify(CMatr1, distort_left, CMatr2, distort_right, (1280,720), R_lr, g_lr[:, 3][:3].reshape((3, 1)), alpha=-1)
    

    points4D = cv2.triangulatePoints(P1, P2, projPoints1, projPoints2)

    #Converts 4D to 3D by [x,y,z,w] -> [x/w, y/w, z/w]
    points3D = np.array([points4D[0]/points4D[3] / 10, points4D[1]/points4D[3] / 10, points4D[2]/points4D[3] / 10, 1])
    # points3D = np.array([-0.595, 0.438, 5.262, 1])
    #point_t = np.dot(g_tl,points3D).reshape((4,1))
    #print(point_t)
    point_t = points3D
    return Point(point_t[0],point_t[1],point_t[2])


  def update_left(self,msg):
    self.frame0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

  def update_right(self,msg):
    self.frame1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

  def findBall(self):
    #points = []
    global hue
    global sat
    global val
    rate = rospy.Time(30)

    self.bridge = CvBridge()
    rospy.Subscriber('/camera_left/left/image_raw', Image, self.update_left)
    rospy.Subscriber('/camera_right/right/image_raw', Image, self.update_right)
    pub = rospy.Publisher('/ball_position', PointStamped, queue_size=10)
    rospy.init_node('findBall')

    while not rospy.is_shutdown():
      try:

        #_,frame0 = cap0.read()
        #_,frame1 = cap1.read()

        frame0 = self.frame0
        frame1 = self.frame1
        #print(frame0.shape)
        #cv2.imshow("camera 1", frame0)

        lower_left = np.array([global_hue_l,global_sat_l,global_val_l])
        lower_right = lower_left
        upper_left = np.array([global_max_hue_l,global_max_sat_l,global_max_val_l])
        upper_right = np.array([global_max_hue_r,global_max_sat_r,global_max_val_r])

        frame0_hsv = cv2.cvtColor(frame0,cv2.COLOR_BGR2HSV)
        frame0_thresh = cv2.inRange(frame0_hsv,lower_left, upper_left)
        frame0_thresh = cv2.medianBlur(frame0_thresh,7)
        frame0_inverted = cv2.bitwise_not(frame0_thresh)
        frame0_kernel = np.ones((kern_l,kern_l),np.uint8)
        frame0_erosion = cv2.erode(frame0_inverted,frame0_kernel,iterations = e_its_l)
        frame0_dilation = cv2.dilate(frame0_erosion,frame0_kernel,iterations = d_its_l)

        frame0_edged = cv2.Canny(frame0_dilation, 30, 200)         
        _,frame0_contours,_ = cv2.findContours(frame0_edged,  
                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
  		
        maxArea = 0
        max_x = None
        max_y = None
        max_w = 0
        max_h = 0
        for c in frame0_contours:
          frame0_x,frame0_y,frame0_w,frame0_h = cv2.boundingRect(c)
          area = frame0_w*frame0_h
          if area > maxArea:
            max_x = frame0_x + frame0_w/2
            max_y = frame0_y + frame0_h/2
            max_w = frame0_w
            max_h = frame0_h
            maxArea = area
      

        left_max_x = max_x
        left_max_y = max_y

        if (left_max_x == None or left_max_y == None):
          print("No ball") #TODO: Fix this to publish NaN or equivalent
          pos = PointStamped()
          pos.point = Point(np.nan,np.nan,np.nan)
          #pub.publish(pos)
          continue
        #else:
          print("left", (left_max_x, left_max_y))

        
        cv2.rectangle(frame0_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        #cv2.imshow("camera 1", frame0_dilation)
        

        frame1_hsv = cv2.cvtColor(frame1,cv2.COLOR_BGR2HSV)
        frame1_thresh = cv2.inRange(frame1_hsv,lower_left, upper_right)
        frame1_thresh = cv2.medianBlur(frame1_thresh,7)

        frame1_inverted = cv2.bitwise_not(frame1_thresh)
        frame1_kernel = np.ones((kern_r,kern_r),np.uint8)
        frame1_erosion = cv2.erode(frame1_inverted,frame1_kernel,iterations = e_its_r)
        frame1_dilation = cv2.dilate(frame1_erosion,frame1_kernel,iterations = d_its_r)

        frame1_edged = cv2.Canny(frame1_dilation, 30, 200)         
        _, frame1_contours,_ = cv2.findContours(frame1_edged,  
                          cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        
        maxArea = 0
        max_x = None
        max_y = None
        max_w = 0
        max_h = 0
        for c in frame1_contours:
          frame1_x,frame1_y,frame1_w,frame1_h = cv2.boundingRect(c)
          area = frame1_w*frame1_h
          if area > maxArea:
            max_x = frame1_x + frame1_w/2
            max_y = frame1_y + frame1_h/2
            max_w = frame1_w
            max_h = frame1_h
            maxArea = area

        right_max_x = max_x
        right_max_y = max_y
        
        if (right_max_x == None or right_max_y == None):
          pos = PointStamped()
          pos.point = Point(np.nan,np.nan,np.nan)
          #pub.publish(pos)
          continue
        else:
          print("right", (right_max_x, right_max_y))

        cv2.rectangle(frame1_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 2", frame1_dilation)

        pos = PointStamped()

        pos.point = self.get_3d_coords(left_max_x, left_max_y, right_max_x, right_max_y, frame1_dilation.shape[::-1])
        pos.header.frame_id = 'left'
        pub.publish(pos)
        # if self.counter == 30:


        #   fig = plt.figure()
        #   ax = plt.axes(projection='3d')
        #   x = self.points_graph[0]
        #   y = self.points_graph[1]
        #   z = self.points_graph[2]
          
        #   print(self.points_graph)

        #   ax.scatter3D(x, y, z)
        #   plt.show()

        # if cv2.waitKey(1) ==1048603:
        #   exit(0)
        #   f.close()

      except ThreadError:
        self.thread_cancelled = True

    
if __name__ == "__main__":
  cam = Cam()
  cam.findBall()


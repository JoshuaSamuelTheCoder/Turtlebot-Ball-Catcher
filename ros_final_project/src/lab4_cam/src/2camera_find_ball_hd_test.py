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
global_max_sat_l = 154
global_max_val_l = 255

global_max_hue_r = 180
global_max_sat_r= 154
global_max_val_r = 255

e_its_l = 7
d_its_l = 15
kern_l = 5

e_its_r = 9
d_its_r = 15
kern_r = 5


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

  def quatToRot(self, q):
      sqw = q[3]*q[3]
      sqx = q[0]*q[0]
      sqy = q[1]*q[1]
      sqz = q[2]*q[2]

      # invs (inverse square length) is only required if quaternion is not already normalised
      invs = 1 / (sqx + sqy + sqz + sqw)
      m00 = ( sqx - sqy - sqz + sqw)*invs # since sqw + sqx + sqy + sqz =1/invs*invs
      m11 = (-sqx + sqy - sqz + sqw)*invs
      m22 = (-sqx - sqy + sqz + sqw)*invs
      
      tmp1 = q[0]*q[1]
      tmp2 = q[2]*q[3]
      m10 = 2.0 * (tmp1 + tmp2)*invs
      m01 = 2.0 * (tmp1 - tmp2)*invs
      
      tmp1 = q[0]*q[2]
      tmp2 = q[1]*q[3]
      m20 = 2.0 * (tmp1 - tmp2)*invs 
      m02 = 2.0 * (tmp1 + tmp2)*invs 
      tmp1 = q[1]*q[2]
      tmp2 = q[0]*q[3]
      m21 = 2.0 * (tmp1 + tmp2)*invs
      m12 = 2.0 * (tmp1 - tmp2)*invs  
      R = np.matrix([[m00, m01, m02],
               [m10, m11, m12],
              [m20,m21,m22]])
      return R

  def get_3d_coords(self, left_max_x, left_max_y, right_max_x, right_max_y, imageSize):
          #monocular calibration 1 = left 2 = right
    #CMatr1 = np.matrix([[2531.915668, 0.000000, 615.773452],  
    #[0.000000, 2594.436434, 344.505755],
    #[0.000000, 0.000000, 1.000000]]).astype(np.float)

    CMatr1 = np.matrix([[1199.0608765416198, 0.0, 614.5124090033408],
    [0.0, 1216.3796974224015, 304.6894283956403],
    [0.000000, 0.000000, 1.000000]]).astype(np.float)

    # print("CMatr1", CMatr1)
    #left distortion parameters: 1.281681 -15.773048 -0.010428 0.012822 0.000000

    CMatr2 = np.matrix([[1094.037974, 0.000000, 626.163067],
      [0.000000, 1104.123453, 409.408438],
      [0.000000, 0.000000, 1.000000]]).astype(np.float)


    # print("CMatr2", CMatr2)

    projPoints1 = np.array([[left_max_x],[left_max_y]]).astype(np.float)

    projPoints2 = np.array([[right_max_x],[right_max_y]]).astype(np.float)

    distort_left = np.array([-0.014898312961319021, -0.02745621059658203, 0.0005535088809163774, -0.00707317997328806, 0.0]).astype(np.float)
    distort_right = np.array([0.007916259563114801, -0.07977699604512571, -0.0015394879448593559, -0.004113051306898727, 0.000000]).astype(np.float)
    # print("distort_left", distort_left)
    # print("distort_right", distort_right)

    """"
    At time 1576037712.126
 Translation: [0.483, 0.150, 1.465]
 Rotation: in Quaternion [0.983, -0.024, -0.179, -0.019]
            in RPY (radian) [-3.111, 0.360, -0.043]
            in RPY (degree) [-178.270, 20.613, -2.485]
  """

    # R_lt = self.quatToRot(np.array([0.677, -0.673, 0.169, 0.245]))
    # R_rt = self.quatToRot(np.array([-0.042, 0.940, -0.302, 0.157]))
    R_lt = quaternion_matrix(np.array([0.719, 0.694, -0.030, 0.032]))
    R_rt = quaternion_matrix(np.array([0.727, 0.680, -0.097, 0.023]))
    R_lt = R_lt[:3,:3]
    R_rt = R_rt[:3,:3]
    R_tl = np.linalg.inv(R_lt)
    R_tr = np.linalg.inv(R_rt)
    T_lt_l = np.array([-0.352, 0.289, 1.774]).astype(np.float) #--------------------CHANGE
    T_rt_r = np.array([-0.260, 0.183, 2.137]).astype(np.float) #--------------------CHANGE

    # R_lt = self.quatToRot(np.array([-0.246, 0.805, -0.492, 0.224]))
    # R_rt = self.quatToRot(np.array([-0.095, 0.884, -0.458, -0.000]))
    # R_tl = np.linalg.inv(R_lt)
    # R_tr = np.linalg.inv(R_rt)

    # T_lt_l = np.array([-1.300, 1.832, 1.323]).astype(np.float) #--------------------CHANGE
    # T_rt_r = np.array([-0.307, 2.070, 1.287]).astype(np.float) 

    g_lt_1 = np.hstack((R_lt, T_lt_l.reshape((3,1)))) 
    g_lt_2 = np.array([0,0,0,1])
    g_lt = np.vstack((g_lt_1,g_lt_2.T))
    g_tl = np.linalg.inv(g_lt)


    g_rt_1 = np.hstack((R_rt, T_rt_r.reshape((3,1))))
    g_rt_2 = np.array([0,0,0,1])
    g_rt = np.vstack((g_rt_1,g_rt_2.T)) 
    g_tr = np.linalg.inv(g_rt)
    # print("g_rt", g_rt)
    # print("R_rt", R_rt)
    # print("T_rt_r", T_rt_r)

    g_lr = np.dot(g_lt,g_tr)
    print("g_lr", g_lr)

    R_lr = np.dot(R_lt,R_tr)

    # print(g_lr[:, 3][:3])
    # print("predicted value: ", np.dot(R_lt, np.array(g_tr[:, 3][:3])) + T_lt_l.reshape(3,1))
    #print('glr col: ', g_lr[:, 3])
    #print('glr:', g_lr)
    # T_rt_r_hom = np.array([T_rt_r[0],T_rt_r[1],T_rt_r[2],1])
    # T_rt_l_hom = np.dot(g_lr,T_rt_r_hom)
    # T_rt_l = np.array([T_rt_l_hom[0,0],T_rt_l_hom[0,1],T_rt_l_hom[0,2]])
    # T_lr_l = T_lt_l - T_rt_l
    #print('T_lr: ', T_lr_l)

    #print("RFinal", RFinal)
    #print("T_final", T_final)
    #print(imageSize)

    R1,R2,P1,P2,Q, a,b = cv2.stereoRectify(CMatr1, distort_left, CMatr2, distort_right, (1280,720), R_lr, g_lr[:, 3][:3].reshape((3, 1)),  alpha=-1)
    

    #print("R1",R1)
    #print("R2",R2)CMatr1
    #print("P1",P1)
    #print("P2",P2)

    #pnt1 = cv2.undistortPoints(projPoints1, CMatr1, distort_left, R=RMat1, P=P1)
    #pnt2 = cv2.undistortPoints(projPoints2, CMatr2, distort_right, R=RMat2, P=P2)

    print("left:",projPoints1)
    print("right:", projPoints2)

    points4D = cv2.triangulatePoints(P1, P2, projPoints1, projPoints2)
    #points3D = cv2.convertPointsFromHomogeneous(points4D)
    #print(points4D)

    #Converts 4D to 3D by [x,y,z,w] -> [x/w, y/w, z/w]
    #print(Point(points4D[0]/points4D[3], points4D[1]/points4D[3], points4D[2]/points4D[3]))
    points3D = np.array([points4D[0]/points4D[3], points4D[1]/points4D[3], points4D[2]/points4D[3], 1])
    # point_t = np.dot(g_tl,points3D).reshape((4,1))
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
    #pub = rospy.Publisher('/ball_position', PointStamped, queue_size=10)
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
            max_x = frame0_x + frame0_w / 2
            max_y = frame0_y + frame0_h / 2
            max_w = frame0_w
            max_h = frame0_h
            maxArea = area
      

        left_max_x = max_x
        left_max_y = max_y

        if (left_max_x == None or left_max_y == None):
          #print("No ball") #TODO: Fix this to publish NaN or equivalent
          #pos = PointStamped()
          #pos.point = Point(np.nan,np.nan,np.nan)
          #pub.publish(pos)
          continue
        #else:
          #print(left_max_x, left_max_y)
        
        cv2.rectangle(frame0_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 1", frame0_dilation)
        

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
            max_x = frame1_x + frame1_w / 2
            max_y = frame1_y + frame1_h / 2
            max_w = frame1_w
            max_h = frame1_h
            maxArea = area

        right_max_x = max_x
        right_max_y = max_y
        
        if (right_max_x == None or right_max_y == None):
          #pos = PointStamped()
          #pos.point = Point(np.nan,np.nan,np.nan)
          #pub.publish(pos)
          continue

        cv2.rectangle(frame1_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 2", frame1_dilation)

        #pos = PointStamped()

        #pos.point = self.get_3d_coords(left_max_x, left_max_y, right_max_x, right_max_y, frame1_dilation.shape[::-1])
        #pos.header.frame_id = 'ar_marker_5'
        #pub.publish(pos)
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

    #plot the 3D points

  cv2.namedWindow("objeto_string")

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


import numpy as np
import cv2
import threading
from threading import Thread, Event, ThreadError
import sys
import pprint


from mpl_toolkits import mplot3d
#matplotlib inline
import matplotlib.pyplot as plt




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
global_max_sat = 180
global_max_value = 255

left_max_x = 0
left_max_y = 0
right_max_x = 0
right_max_y = 0


class Cam():
  points_graph = np.array([0,0,0]).reshape((3,1))
  counter = 0
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

  def publish_3d_coords(self, left_max_x, left_max_y, right_max_x, right_max_y, imageSize):
          #monocular calibration 1 = left 2 = right
    CMatr1 = np.matrix([[2531.915668, 0.000000, 615.773452],  
    [0.000000, 2594.436434, 344.505755],
    [0.000000, 0.000000, 1.000000]]).astype(np.float)

    pp = pprint.PrettyPrinter(indent=4)

    # print("CMatr1", CMatr1)
    #left distortion parameters: 1.281681 -15.773048 -0.010428 0.012822 0.000000

    CMatr2 = np.matrix([[1539.714285, 0.000000, 837.703760],
    [0.000000, 1506.265655, 391.687374],
    [0.000000, 0.000000, 1.000000]]).astype(np.float)

    # print("CMatr2", CMatr2)

    projPoints1 = np.array([[left_max_x],[left_max_y]]).astype(np.float)

    projPoints2 = np.array([[right_max_x],[right_max_y]]).astype(np.float)

    distort_left = np.array([1.281681, -15.773048, -0.010428, 0.012822, 0.000000]).astype(np.float)
    distort_right = np.array([0.091411, -0.461269, 0.021006, 0.040117, 0.000000]).astype(np.float)
    # print("distort_left", distort_left)
    # print("distort_right", distort_right)

    RMat1 = self.quatToRot(np.array([-0.029, 0.992, -0.110, 0.053]))
    RMat2 = self.quatToRot(np.array([-0.019, 0.997, -0.038, -0.071]))



    RFinal = np.matmul(np.linalg.inv(RMat1),RMat2)

    T_left = np.array([-0.268, 0.516, 3.283]).astype(np.float) #--------------------CHANGE

    T_right = np.array([0.018, -0.184, 1.255]).astype(np.float) #--------------------CHANGE

    T_final = T_right - T_left

    #print("RFinal", RFinal)
    #print("T_final", T_final)
    #print(imageSize)

    R1,R2,P1,P2,Q, a,b = cv2.stereoRectify(CMatr1, distort_left, CMatr2, distort_right, (1280,720), RFinal, T_final,  alpha=-1)
    

    #print("R1",R1)
    #print("R2",R2)CMatr1
    #print("P1",P1)
    #print("P2",P2)

    #pnt1 = cv2.undistortPoints(projPoints1, CMatr1, distort_left, R=RMat1, P=P1)
    #pnt2 = cv2.undistortPoints(projPoints2, CMatr2, distort_right, R=RMat2, P=P2)

    #print("left:",projPoints1)
    #print("right:", projPoints2)

    P1_stereo = np.array([[4890.538324810042, 0.0, -1734.3179817199707, 0.0],[ 0.0, 4890.538324810042, 398.04181480407715, 0.0],[ 0.0, 0.0, 1.0, 0.0]])
    P2_stereo = np.array([[4890.538324810042, 0.0, -1734.3179817199707, 8092.200252104331],[ 0.0, 4890.538324810042, 398.04181480407715, 0.0],[ 0.0, 0.0, 1.0, 0.0]])


    points4D = cv2.triangulatePoints(P1_stereo, P2_stereo, projPoints1, projPoints2)
    #points3D = cv2.convertPointsFromHomogeneous(points4D)
    #print(points4D)

    #Converts 4D to 3D by [x,y,z,w] -> [x/w, y/w, z/w]
    
    points3D = np.array([  points4D[0]/points4D[3], points4D[1]/points4D[3], points4D[2]/points4D[3] ])




  def run(self):
    points = []
    global hue
    global sat
    global val
   
    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(1)

    while True:
      try:
        _,frame0 = cap0.read()
        _,frame1 = cap1.read()

        s = cv2.getTrackbarPos('Max Sat',objeto_string)
        if s == 0:
        	s = global_max_sat

        lower = np.array([global_hue,global_sat,global_val])
        upper = np.array([global_max_hue,s,global_max_value])

        frame0_hsv = cv2.cvtColor(frame0,cv2.COLOR_BGR2HSV)
        frame0_hsv2 = frame0_hsv.copy()
        frame0_thresh = cv2.inRange(frame0_hsv,lower, upper)
        frame0_thresh = cv2.medianBlur(frame0_thresh,7)
        frame0_thresh2 = frame0_thresh.copy()

        frame0_inverted_image = cv2.bitwise_not(frame0_thresh2)
        frame0_kernel = np.ones((3,3),np.uint8)
        frame0_erosion = cv2.erode(frame0_inverted_image,frame0_kernel,iterations = 7)
        frame0_dilation = cv2.dilate(frame0_erosion,frame0_kernel,iterations = 15)

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
          	max_x = frame0_x
          	max_y = frame0_y
          	max_w = frame0_w
          	max_h = frame0_h

        left_max_x = max_x
        left_max_y = max_y

        if (left_max_x == None or left_max_y == None):
          print("No ball") #TODO: Fix this to publish NaN or equivalent
          continue
        
        cv2.rectangle(frame0_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 1", frame0_dilation)
        

        frame1_hsv = cv2.cvtColor(frame1,cv2.COLOR_BGR2HSV)
        frame1_hsv2 = frame1_hsv.copy()
        frame1_thresh = cv2.inRange(frame1_hsv,lower, upper)
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
        max_x = None
        max_y = None
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

        right_max_x = max_x
        right_max_y = max_y
        
        if (right_max_x == None or right_max_y == None):
          print("No ball") #TODO: Fix this to publish NaN or equivalent
          continue

        cv2.rectangle(frame1_dilation, (max_x, max_y), (max_x+max_w, max_y+max_h), (255, 0, 255), 2)
        cv2.imshow("camera 2", frame1_dilation)

        
        self.publish_3d_coords(left_max_x, left_max_y, right_max_x, right_max_y, frame1_dilation.shape[::-1])


        if cv2.waitKey(1) ==1048603:
          exit(0)
          f.close()

      except ThreadError:
        self.thread_cancelled = True

    #plot the 3D points

  cv2.namedWindow(objeto_string)

  cv2.createTrackbar('Max Sat', objeto_string, global_sat, 255, nothing)

    
if __name__ == "__main__":
  cam = Cam()
  input = int(sys.argv[1])
  cam.run()

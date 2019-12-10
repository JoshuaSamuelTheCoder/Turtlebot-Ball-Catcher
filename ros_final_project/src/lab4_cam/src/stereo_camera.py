import numpy as np 
import cv2



def quatToRot(q):
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






# from stereo calibration
#projMatr1 = np.matrix([[353.457135, 0.000000, -12954.704506, 0.000000],
#[0.000000, 353.457135, 729.827810, 0.000000],
#[0.000000, 0.000000, 1.000000, 0.000000]])

#projMatr2 = np.matrix([[353.457135, 0.000000, -12954.704506, 1226.682635],
#[0.000000, 353.457135, 729.827810, 0.000000],
#[0.000000, 0.000000, 1.000000, 0.000000]])


#monocular calibration 1 = left 2 = right
CMatr1 = np.matrix([[2531.915668, 0.000000, 615.773452],	
[0.000000, 2594.436434, 344.505755],
[0.000000, 0.000000, 1.000000]])


#left distortion parameters: 1.281681 -15.773048 -0.010428 0.012822 0.000000

CMatr2 = np.matrix([[1539.714285, 0.000000, 837.703760],
[0.000000, 1506.265655, 391.687374],
[0.000000, 0.000000, 1.000000]])



projPoints1 = np.array([[0.819,0.862,0.837,0.813,0.784,0.727,0.885,0.911,0.634],[0.853,0.936,0.718,0.420,0.266,0.229,0.19,0.229,0.183]])

projPoints2 = np.array([[0.468,0.424,0.469,0.494,0.469,0.458,0.522,0.606,0.527],[0.449,0.936,0.795,0.645,0.741,0.743,0.415,0.190,0.102]])

#points4D = np.matrix([[1,1,1,1,1,1,1,1,1,1]])

distort_left = np.array(np.array([1.281681, -15.773048, -0.010428, 0.012822, 0.000000]))
distort_right = np.array(np.array([0.091411, -0.461269, 0.021006, 0.040117, 0.000000]))

RMat1 = quatToRot(np.array([0.995476024346,-0.0101395947738, -0.0919869711741,-0.0215190776772]))
RMat2 = quatToRot(np.array([0.987134912131,-0.00767041961091, 0.0671661540625,-0.144894919385]))



RFinal = np.matmul(np.linalg.inv(RMat1),RMat2)

T_left = np.array([0.133166411513, 0.215015282037, 0.74458424227]) #--------------------CHANGE

T_right = np.array([-0.123669420489, 0.0784183383604, 1.9679756281]) #--------------------CHANGE

T_final = T_left - T_right



R1,R2,P1,P2,Q = cv2.stereoRectify(CMatr1, CMatr2, distort_left, distort_right, (1280, 720), RFinal, T_final)
points4D = cv2.triangulatePoints(P1, P2, projPoints1, projPoints2)
#points4D_sol = cv2.triangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2)

print(points4D)

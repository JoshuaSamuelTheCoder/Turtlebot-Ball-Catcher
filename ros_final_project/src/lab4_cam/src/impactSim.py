import rospy
import numpy as np
from numpy import linalg as LA
from geometry_msgs.msg import Point, PointStamped
import threading
from threading import Thread, Event, ThreadError

C_D = 0.47  # Coefficient of Drag of Ball
rho = 1.225 #	Air Density (kg/m^3) 
m = 0.05# Mass of Ball (kg)
dt = 0.03333333
g = np.array([0, 0, -9.81])
pos = PointStamped()
def updatePos(data):
	pos = data

def impactSim():
	r_last = np.array([np.nan, np.nan, np.nan])
	rate = rospy.Time(30)
	rospy.Subscriber('/ball_position',PointStamped, updatePos)
	pub = rospy.Publisher('/point_of_impact',PointStamped, queue_size=10)
	rospy.init_node('impactSim')
	while not rospy.is_shutdown():
		try:
			# Set a condition for if the ball is not seen.
			# - Set r_last back to [nan,nan,nan]
			# - Don't publish
			x = pos.point.x
			y = pos.point.y
			z = pos.point.z
			if (np.isnan(x)):
				r_last = np.array([np.nan, np.nan, np.nan])	
				continue
			elif (np.isnan(r_last[0])):
				r_last = np.array([x, y, z])
				continue
			r = np.array([x, y, z])
			v = (r-r_last)/dt
			r_last = r
			# Step forward in time until the ball's vertical position is 0.
			while (r[2] > 0):
				F_d = -(1/2)*rho*C_D*v*LA.norm(v)
				F_g = m*g
				F = F_d + F_g
				a = F/m
				r = r + v*dt 
				v = v + a*dt
			point = Point()
			point.x = x
			point.y = y
			point.z = z
			poi = PointStamped()
			poi.point = point
			poi.header.frame_id = 'ar_marker_5'
			pub.publish(poi)
		except ThreadError:
			self.thread_cancelled = True

if __name__ == '__main__':
	try:
		impactSim()
	except rospy.ROSInterruptException:
		pass
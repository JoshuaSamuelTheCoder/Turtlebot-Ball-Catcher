import rospy
import numpy as np
from numpy import linalg as LA
from geometry_msgs.msg import Point

int fps = 30
float C_D = 0.47  # Coefficient of Drag of Ball
float rho = 1.225 #	Air Density (kg/m^3) 
float m = # Mass of Ball (kg)
float dt = 1/float(fps)
g = np.array([0, 0, -9.81])
r_last = np.array([np.nan, np.nan, np.nan])

def updatePos()
	self.pos = msg.data

def impactSim():
	rate = rospy.rate(fps)
	rospy.Subscriber('/ball_position',Point, updatePos)
	pub = rospy.Publisher('/point_of_impact',Point, queue_size=10)
	rospy.init_node('impactSim')
	while not rospy.is_shutdown()
		# Set a condition for if the ball is not seen.
		# - Set r_last back to [nan,nan,nan]
		# - Don't publish
		if (...)
			r_last = np.array([np.nan, np.nan, np.nan])	
			continue
		# We need a valid r_last to calculate v, so skip the first frame when the ball is seen.
		if (np.isnan(r_last[0]))
			continue
		float x = pos.x
		float y = pos.y
		float z = pos.z
		r = np.array([x, y, z])
		v = (r-r_last)/dt
		r_last = r
		# Step forward in time until the ball's vertical position is 0.
		while (r[2] > 0)
			F_d = -(1/2)*rho*C_D*v*LA.norm(v)
			F_g = m*g
			F = F_d + F_g
			a = F/m
			r = r + v*dt 
			v = v + a*dt
		poi = Point(x,y,z)
		last_poi = poi
		pub.publish(poi)

if __name__ == '__main__':
	try:
		impactSim()
	except rospy.ROSInterruptException:
		pass
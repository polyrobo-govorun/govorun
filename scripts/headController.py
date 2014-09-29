#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 
from geometry_msgs.msg import Point
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetSpeed

from math import pi, sin, cos, sqrt, atan2, acos, degrees, radians
#from numpy import arange


def cart2sph(x,y,z):
	XsqPlusYsq = x**2 + y**2
	r = sqrt(XsqPlusYsq + z**2)               # r
	theta = atan2(sqrt(XsqPlusYsq), z)     # theta
	phi = atan2(y,x)                           # phi
	if r == 0.0:
		theta = pi/2;	
	return r, theta, phi

def sph2cart(r, theta, phi):
	x = r*sin(theta)*cos(phi)
	y = r*sin(theta)*sin(phi)
	z = r*cos(theta)
	return x, y, z

def headLookAtPoint():
	global curr_phi_neck 
	global curr_theta_neck 
	global order_bottom_neck 
	global order_upper_neck 
	global order_bottom_eyes 
	global order_upper_eyes
	global min_bottom_neck;
	global max_bottom_neck;
	global min_upper_neck;
	global max_upper_neck;
	global destination_point

	global neckMoving;

	p = Point(0.0, 0.0, 0.0);
	

	# shift origin
	# coordinates of  neck origin with respect to camera 
	# for the current values camera is expected to be 10cm back and right after the top of the head
	x0 = 15; 
	y0 = 0; 
	z0 = -35;
	# now we are in neck CS
	
	rospy.logwarn("input: %g %g %g", destination_point.x, destination_point.y, destination_point.z);

	
	if destination_point.z > 0:
		p.x = 2700.0/destination_point.z+x0; # conversion from face size to distance
		px = 7.0/destination_point.z; # cm in 1 px at current distance
		p.y = (destination_point.x-300)*px+y0;
		p.z = (destination_point.y-200)*px+z0;
	else:
		p.x = 0.0+x0; # conversion from face size to distance
		p.y = 0.0+y0;
		p.z = 0.0+z0;

	#p.x = p.x;
	#p.y = -p.y;
	#p.z = -p.z;


	rospy.logwarn("converted: %g %g %g", p.x, p.y, p.z);


	# Neck position
	(R, theta, phi) = cart2sph(p.x, p.y, p.z); # radians
	phi_neck = degrees(phi);
	theta_neck = degrees(theta); # degrees
	#rospy.logwarn("sperical: R = %g th = %g phi = %g", R, theta, phi);

	# order for Eyes
	dist_phi = phi_neck - curr_phi_neck;
	dist_theta = theta_neck - curr_theta_neck;

	order_bottom_eyes = -dist_phi;
	order_upper_eyes = dist_theta;


	#if(neckMoving and abs(dist_theta) < 3.0 and  abs(dist_phi) < 3.0):
	#	neckMoving = False; # stop neck 
	
	#if(not neckMoving and (abs(dist_theta) >= 20.0 and  abs(dist_phi) >= 20.0) ):
	#	neckMoving = True; # move neck

	#if(not neckMoving):
	#	phi_neck = curr_phi_neck;
	#	theta_neck = curr_theta_neck;


	
	# put neck positions to global vars
	order_upper_neck = theta_neck/theta_neck_coeff - theta_neck_zero; # deg
	order_bottom_neck = phi_neck/phi_neck_coeff - phi_neck_zero;
	if(order_upper_neck > max_upper_neck):
		order_upper_neck = max_upper_neck;
	if(order_upper_neck < min_upper_neck):
		order_upper_neck = min_upper_neck;
	if(order_bottom_neck > max_bottom_neck):
		order_bottom_neck = max_bottom_neck;
	if(order_bottom_neck < min_bottom_neck):
		order_bottom_neck = min_bottom_neck;
	rospy.logwarn("neck: bottom = %g upper = %g ", order_bottom_neck, order_upper_neck);
	rospy.logwarn("eyes: bottom = %g upper = %g ", order_bottom_eyes, order_upper_eyes);


theta_neck_zero = -0.3 + 90.0/50.0; # zero position in motor units
phi_neck_zero = 0.0;
theta_neck_coeff = 50.0; # how many degrees in one motor unit
phi_neck_coeff = 25.0;

curr_phi_neck = 0.0; # deg
curr_theta_neck = 0.0; 
curr_bottom_neck = 0.0;
curr_upper_neck = 0.0;
order_bottom_neck = 0.0;
order_upper_eyes = 0.0;
order_bottom_eyes = 0.0;

min_bottom_neck = -3.0;
max_bottom_neck = 3.0;
min_upper_neck = 0.0;
max_upper_neck = 1.0;

neckMoving = False;

# min order vert = 0 max = 1
# order hor -3:3


destination_point = Point(0,0,0);

def destination_update(data):
	global destination_point
	destination_point = data

def neckHorisontalUpdate(data):
	global curr_phi_neck
	global curr_theta_neck
	global curr_bottom_neck 
	global curr_upper_neck 
	curr_phi_neck = (data.current_pos+phi_neck_zero)*phi_neck_coeff; # in [-150 150] deg
	curr_bottom_neck = data.current_pos

def neckVerticalUpdate(data):
	global curr_phi_neck
	global curr_theta_neck
	global curr_bottom_neck 
	global curr_upper_neck 
	curr_theta_neck = (data.current_pos+theta_neck_zero)*theta_neck_coeff; # in [-150 150] deg
	curr_upper_neck = data.current_pos
	

def main():
	#global curr_phi_neck
	#global curr_theta_neck
	global curr_bottom_neck 
	global curr_upper_neck 
	global order_bottom_neck 
	global order_upper_neck 
	global destination_point
	global order_bottom_eyes 
	global order_upper_eyes

	speedCoeff = 2.0;
	eyeOrder = Point(0,0,0);

	rospy.init_node('headController', anonymous=True)

	# set max speed for neck motors

	# neck control topics
	# angle in range [-3, 3] [-150 deg to 150 deg] 
	neckVerticalPub = rospy.Publisher('upper_tilt_controller/command', Float64)#, queue_size=10)
	neckHorisontalPub = rospy.Publisher('bottom_tilt_controller/command', Float64)#, queue_size=10)
	eyesPub = rospy.Publisher("eyesDirection", Point)

	#headLookAtPoint()	
	
	r = rospy.Rate(30) # hz

	while not rospy.is_shutdown():

		rospy.Subscriber("upper_tilt_controller/state", JointState, neckVerticalUpdate) 
		rospy.Subscriber("bottom_tilt_controller/state", JointState, neckHorisontalUpdate)
		rospy.Subscriber("/neckControlCamera/servosDirection", Point, destination_update)
		


		# perform calculations
		headLookAtPoint()

		# calculate appropriate speed
		speedUpper = abs(order_upper_neck - curr_upper_neck)*speedCoeff;
		speedBottom = abs(order_bottom_neck - curr_bottom_neck)*speedCoeff;
		
		try:
			setSpeedUpper = rospy.ServiceProxy('/upper_tilt_controller/set_speed', SetSpeed)
			setSpeedBottom = rospy.ServiceProxy('/bottom_tilt_controller/set_speed', SetSpeed)
			setSpeedUpper(speedUpper);
			setSpeedBottom(speedBottom);
			#rospy.logwarn("speed upper: %g bottom: %g", speedUpper, speedBottom);
		except rospy.ServiceException, e:
			print "SetSpeed call failed: %s" %e
			rospy.logwarn("SetSpeed call failed: %s", e);

		# publish order to head
		neckHorisontalPub.publish(order_bottom_neck)
		neckVerticalPub.publish(order_upper_neck)

		# Eyes
		eyeOrder.x = order_bottom_eyes;
		eyeOrder.y = order_upper_eyes;
		eyesPub.publish(eyeOrder);


		#rospy.logwarn("current neck: bot = %g up = %g", curr_bottom_neck, curr_upper_neck);
		#rospy.logwarn("order neck: bot = %g up = %g", order_bottom_neck, order_upper_neck);


		#rospy.spin()
		r.sleep()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass





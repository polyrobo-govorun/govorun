#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 
from geometry_msgs.msg import Point
from dynamixel_msgs.msg import JointState
from random import randint

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


def main():
	order = Point(100,10,10);
	eyeOrder = Point(0,0,0);


	rospy.init_node('headTestPublisher', anonymous=True)

	directionPub = rospy.Publisher("/neckControlCamera/servosDirection", Point)
	
	r = rospy.Rate(5) # hz	

	while not rospy.is_shutdown():
		order.x = order.x + randint(-1,1)*1;
		order.y = order.y + randint(-1,1)*10;
		order.z = order.z + randint(-1,1)*10;

		# publish data
		directionPub.publish(order);

		# for eyes
		#(R, theta, phi) = cart2sph(order.x, order.y, order.z);
		#eyeOrder.x = degrees(phi); eyeOrder.y = degrees(theta);
		#directionPub.publish(eyeOrder);
		#rospy.logwarn("eyes order: bot = %g up = %g", eyeOrder.x, eyeOrder.y);

		r.sleep()





if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass





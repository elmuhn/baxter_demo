#!/usr/bin/env python
#import math
import numpy
import rospy
import std_msgs
import rosbag

from math import cos, sin, pi, sqrt
from geometry_msgs.msg import Twist, Vector3
from turtlesim.srv import TeleportAbsolute
from bondpy import bondpy

# Per = input('The period of the function (T): ')
# print("T=", Per)
#rosbag record /turtle1/cmd_vel -l Per

Per = 5

def velocity_input(time):
	Vx = -((4.0*pi)/Per)*sin(((2.0*pi)/Per)*time)
	Vy = ((4.0*pi)/Per)*cos(((2.0*pi)/Per)*time)

	ax = -((8.0*pi**2)/Per**2)*cos(((2.0*pi)/Per)*time)
	ay = -((8.0*pi**2)/Per**2)*sin(((2.0*pi)/Per)*time)

	V = (sqrt(Vx**2 + Vy**2))
	o = ((Vx*ay - Vy*ax) / (Vx**2 + Vy**2))
	
	return [V, o]

def velocity_output():
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(75)
	t0 = rospy.get_time()
	start_time = rospy.Time.now().secs

	while rospy.Time.now().secs - start_time < 10:

	# while not rospy.is_shutdown():
		time = rospy.get_time()-t0
		x = 3.0*sin(((4.0*pi)/Per)*time)
		y = 3.0*sin(((2.0*pi)/Per)*time)
		velocities = velocity_input(time)
		V = velocities[0]
		o = velocities[1]

		vel_cmd_out = Twist(Vector3(V,0,0),Vector3(0,0,o))
		rospy.loginfo(vel_cmd_out)
		pub.publish(vel_cmd_out)

		r.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('velocity_output', anonymous=True)
		rospy.wait_for_service('turtle1/teleport_absolute')
		bond = bondpy.Bond("turtlesim_circle", "turtlesim_circle")
		bond.start()
		turtle_sp = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
		turtle_sp = (3.54,5.54,0)
		velocity_output()
	except rospy.ROSInterruptException: pass

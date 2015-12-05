#!/usr/bin/env python

import rospy
import baxter_interface

from baxter_interface import CHECK_VERSION
from std_msgs.msg import UInt16

#Create a class for Baxter robot description
class Baxter(object):
	def __init__(self):
		self._pub_rate = rospy.Publisher('robot/joint_state_rate', UInt16, queue_size = 10)
		self._left_arm = baxter_interface.limb.Limb("left")
		self._right_arm = baxter_interface.limb.Limb("right")
		self._left_joint_names = self._left_arm.joint_names()
		self._right_joint_names = self._right_arm.joint_names()

		#control parameters
		self._rate = 500.0 #Hz

		print "Getting robot state..."
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled

		print "Enabling robot..."
		self._rs.enable()

		#Set joint state publishing to 500 Hz
		self._pub_rate.publish(self._rate)

	def _reset_control_modes(self):
		rate = rospy.Rate(self._rate)
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self._left_arm.exit_control_mode()
			self._right_arm.exit_control_mode()
			self._pub_rate.publish(100) # 100 Hz default joint state rate
			rate.sleep()
		return True

	def set_neutral(self):
		print "Moving to neutral pose..."
		self._left_arm.move_to_neutral()
		self._right_arm.move_to_neutral()
		return True

	def disable_robot(self):
		self._rs.disable()
		return True


	def clean_shutdown(self):
		print "\nExiting node..."
		#return to normal
		self._reset_control_modes()
		self.set_neutral()
		# if self._init_state:
		# 	print "Disabling robot..."
		self._rs.disable()
		return True

	def set_neutral_and_disable(self):
		print "Moving to neutral pose..."
		self._left_arm.move_to_neutral()
		self._right_arm.move_to_neutral()
		print "Disabling robot..."
		self._rs.disable()
		return True

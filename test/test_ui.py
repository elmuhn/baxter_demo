#!/usr/bin/env python

import rospy
import smach, smach_ros
import sys, select
import actionlib
import baxter_demo.msg as bdm 

from baxter_demo import YamlExtractor
from baxter_demo import kbhit
from std_msgs.msg import String


class SetCancellation(object):
	""" Cancel State Controller

	A class used in Cancel State of ui_sm state machine. This class waits for the 
	user to press 'c' key on keyboard using KBHit class. It also activates a Subscriber
	checking whether a currently running demo terminates. The Subscriber is to a topic
	called "termination_status"
	"""

	def __init__(self, goal):
		self.goal = goal
		self.is_requested = False
		self.kb = kbhit.KBHit()
		self.sub = rospy.Subscriber("termination_status", String, self.sub_cb)
		self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.key_cb)

	def sub_cb(self, data):
		#rospy.loginfo("Demo has just terminated!")
		self.goal.ui_command.command = bdm.GetUserCommand.CANCEL_DEMO
		self.is_requested = True
		self.kb.set_normal_term()
		return

	def key_cb(self, tdat):
		if self.kb.kbhit():
			c = self.kb.getch()
			if c == 'c':
				rospy.loginfo("You pressed 'c', cancelling demo...")
				self.goal.ui_command.command = bdm.GetUserCommand.CANCEL_DEMO
				self.is_requested = True
				self.kb.set_normal_term()

			print "Undefined input! Please press 'c' to cancel demo"
		return

def main():

	rospy.init_node('keyboard_ui_client_node')

	ui_sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
	ui_sm.set_initial_state(['START_SYSTEM'])

	with ui_sm:

		ui_goal = bdm.UserInterfaceGoal()
		ui_result = bdm.UserInterfaceResult()
		ui_fb = bdm.UserInterfaceFeedback()

		def get_ui_goal_cb(userdata, goal):
			print "Press enter to start the system: "
			i, o, e = select.select([sys.stdin], [], [])

			if i:
				user_input = sys.stdin.readline().strip()
				if user_input == '':
					goal.ui_command.command = bdm.GetUserCommand.MOVE_TO_HOME_POSITION
					print "Baxter is moving to home position...\n"
				else:
					print "Undefined input!\n"
					return get_ui_goal_cb(userdata, goal)
				
			return goal

		def show_hp_status_cb(userdata, status, result):
			if status == actionlib.GoalStatus.SUCCEEDED and result.sys_result.result == bdm.GetResult.IS_IN_HOME_POSE:
				print "Baxter has successfully moved to home position!\n"
				rospy.sleep(2)

		smach.StateMachine.add('START_SYSTEM', 
									smach_ros.SimpleActionState('user_input_action', 
																	 bdm.UserInterfaceAction, 
																	 goal_cb = get_ui_goal_cb, 
																	 result_cb = show_hp_status_cb), 

									transitions = {'succeeded':'SEND_SELECTION', 
													'aborted':'aborted', 
													'preempted':'preempted'})


		def get_selection_cb(userdata, goal):
			print "  d1 : joint velocity wobbler \n  d2 : joint velocity puppet\n  d3 : joint position keyboard\n  d4 : head wobbler\n  d5 : gripper cuff control\n d6: joint position joystick \n test1 : turtle_sim infinity drawer\n test2 : turtle_sim circle drawer\n"
			print "Select a demo to run (e.g. d1): "
			inp, out, err = select.select([sys.stdin], [], [], 15)

			if inp:

				usr_inp = sys.stdin.readline().strip()
				x = YamlExtractor(usr_inp)
				demo_req = x.get_name()

				if demo_req is None:
					print "Undefined input!\n"
					return get_selection_cb(userdata, goal)
				else:
					goal.ui_command.command = demo_req

			else:
				goal.ui_command.command = bdm.GetUserCommand.MOVE_TO_IDLE_MODE
				print "Baxter is moving to idle mode...\n"

			return goal 

		@smach.cb_interface(outcomes = ['no_action'])
		def show_selection_status_cb(userdata, status, result):
			if status == actionlib.GoalStatus.SUCCEEDED and result.sys_result.result == bdm.GetResult.IS_IN_IDLE_MODE:
				print "Baxter is in idle mode...\n"
				rospy.sleep(2)
				return 'no_action'

			elif status == actionlib.GoalStatus.SUCCEEDED and result.sys_result.result == bdm.GetResult.DEMO_EXECUTED:
				print "Running demo...\n"

		smach.StateMachine.add('SEND_SELECTION', 
									smach_ros.SimpleActionState('user_input_action', 
																	 bdm.UserInterfaceAction, 
																	 goal = ui_goal, 
																	 goal_cb = get_selection_cb, 
																	 result_cb = show_selection_status_cb),

									transitions = {'succeeded':'CANCEL_DEMO', 
													'aborted':'aborted', 
													'preempted':'preempted', 
													'no_action':'START_SYSTEM'})


		def get_cancel_cb(userdata, goal):

			cancel = SetCancellation(goal)
			print_cond = True

			while not cancel.is_requested:
				if print_cond:
					print "Press c to cancel the current demo: "
					print_cond = False

			kb = kbhit.KBHit()
			kb.set_normal_term()
			return goal



			# print "Press c+enter to cancel the current demo: "
			# inp, out, err = select.select([sys.stdin], [], [])

			# def callback(data, goal):
			# 	print data.data
			# 	goal.ui_command.command = bdm.GetUserCommand.CANCEL_DEMO
			# 	return
			# 	# return goal

			# # def termination_subscriber():
			# # 	sub = rospy.Subscriber("termination_status", String, callback)

			# # termination_subscriber()
			# sub = rospy.Subscriber("termination_status", String, callback, callback_args=goal)

			# if inp:
			# 	usr_inp = sys.stdin.readline().strip()

			# 	if usr_inp == 'c':
			# 		goal.ui_command.command = bdm.GetUserCommand.CANCEL_DEMO										
			# 	else:
			# 		print "Undefined input!"
			# 		return get_cancel_cb(userdata, goal)

			# # elif out:
			# # 	termination_subscriber()
			# # 	print "sikinti burda!"
			# # 	goal.ui_command.command = bdm.GetUserCommand.CANCEL_DEMO

		def show_cancel_status_cb(userdata, status, result):
			if status == actionlib.GoalStatus.SUCCEEDED and result.sys_result.result == bdm.GetResult.DEMO_TERMINATED:
				print "Demo successfully terminated!"

		smach.StateMachine.add('CANCEL_DEMO', 
									smach_ros.SimpleActionState('user_input_action', 
																	 bdm.UserInterfaceAction, 
																	 goal = ui_goal, 
																	 goal_cb = get_cancel_cb, 
																	 result_cb = show_cancel_status_cb), 

									transitions = {'succeeded':'SEND_SELECTION', 
													'aborted':'aborted', 
													'preempted':'preempted'})

	sis = smach_ros.IntrospectionServer('ui_sm_viewer', ui_sm, 'USER_INTERFACE')

	sis.start()
	ui_sm.execute()
	rospy.spin()
	sis.stop()

if __name__ =='__main__':
	main()
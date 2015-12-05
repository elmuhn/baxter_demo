#!/usr/bin/env python

import rospy
import smach, smach_ros
import sys, select
import actionlib
import baxter_demo.msg as bdm 
import yaml
import rospkg


def yaml_loader(filepath):
	""" Loads the configuration file that includes the demo name options to send
	to the demo manager """
	with open(filepath, "r") as file_descriptor:
		data = yaml.load(file_descriptor)
	return data


def get_demo_name(inp):
	""" Extracts the chosen demo name from the configuration file based on the
	user input """
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('baxter_demo')
	ui_filepath = pkg_path + "/config/user_input.yaml"
	ui_data = yaml_loader(ui_filepath)
	names = ui_data.get("input")
	
	if inp in names.keys():
		name_req = names.get(inp)
		return name_req
	else:
		return None


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
			print "  d1 : joint velocity wobbler \n  d2 : joint velocity puppet\n  d3 : joint position keyboard\n  d4 : head wobbler\n  d5 : gripper cuff control\n test1 : turtle_sim infinity drawer\n test2 : turtle_sim circle drawer\n"
			print "Select a demo to run (e.g. d1): "
			inp, out, err = select.select([sys.stdin], [], [], 15)

			if inp:

				usr_inp = sys.stdin.readline().strip()
				demo_req = get_demo_name(usr_inp)

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
			print "Press c+enter to cancel the current demo: "
			inp, out, err = select.select([sys.stdin], [], [])

			if inp:
				usr_inp = sys.stdin.readline().strip()

				if usr_inp == 'c':
					goal.ui_command.command = bdm.GetUserCommand.CANCEL_DEMO										
				else:
					print "Undefined input!"
					return get_cancel_cb(userdata, goal)

			return goal

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
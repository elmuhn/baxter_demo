#!/usr/bin/env python

import rospy
import smach, smach_ros
import subprocess
import baxter_demo.msg as bdm 

from bondpy import bondpy
from baxter_demo import Baxter
from baxter_demo import YamlExtractor
from baxter_demo import nxr 
from std_msgs.msg import String


def shutdown_handler():
	""" Allows the Baxter to move to home position and disable its motors once 
	shutdown is called. """
	#robot = Baxter()
	print "Exiting system..."
	# if not robot._rs.state().enabled:
	# 	robot.clean_shutdown()
	# robot._rs.disable()
	rospy.sleep(3)
	return True


class MoveToHome(smach.State):
	""" Inherited class from smach.State.

	A SMACH state that interacts with the user interface. Its execute method
	gets called when the user starts the system and sends an action goal to
	move the Baxter's both arms to a pre-defined home position.

	Outcomes:
		- 'in_home' : returns when the Baxter successfully moves to home position
		- 'different_request' : returns if another state is called
		- 'aborted' : returns if the Baxter fails to move to home position

	Input & Output keys:
		- 'goal_from_ui' : an action goal sent by the user
		- 'result_for_ui' : an action result to be sent to the user interface 
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes = ['in_home', 'different_request', 'aborted'],
									input_keys = ['goal_from_ui', 'result_for_ui'],
									output_keys = ['result_for_ui'])

	def execute(self, userdata):
		hp_goal = userdata.goal_from_ui
		hp_result = userdata.result_for_ui
		#robot = Baxter()    #Uncomment this when working with Baxter

		if hp_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_HOME_POSITION:

			# Uncomment below block when working with Baxter

			# hp_start_time = rospy.Time.now().secs
			# while rospy.Time.now().secs - hp_start_time < 10:
			# 	if robot.set_neutral():
			# 		hp_result.sys_result.result = bdm.GetResult.IS_IN_HOME_POSE
			# 		return 'in_home'

			# hp_result.sys_result.result = bdm.GetResult.IS_ABORTED
			# return 'aborted'

			rospy.sleep(4)
			hp_result.sys_result.result = bdm.GetResult.IS_IN_HOME_POSE
			return 'in_home'

		else:
			return 'different_request'


class MoveToIdle(smach.State):
	""" Inherited class from smach.State.

	A SMACH state that interacts with the user interface. Its execute method
	gets called if there is no activity from the user within a certain amount of 
	time.

	Outcomes:
		- 'in_idle' : returns when the Baxter successfully disables its motors and
		moves to idle mode
		- 'different_request' : returns if another state is called
		- 'aborted' : returns if the Baxter fails to move to idle mode

	Input & Output keys:
		- 'goal_from_ui' : an action goal sent by the user
		- 'result_for_ui' : an action result to be sent to the user interface 
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes = ['in_idle', 'different_request', 'aborted'],
									input_keys = ['goal_from_ui', 'result_for_ui'],
									output_keys = ['result_for_ui'])

	def execute(self, userdata):
		idle_goal = userdata.goal_from_ui
		idle_result = userdata.result_for_ui
		#robot = Baxter()

		if idle_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_IDLE_MODE:

			# idle_start_time = rospy.Time.now().secs 
			# while rospy.Time.now().secs - idle_start_time < 10:
			# 	if robot.disable_robot():
			# 		rospy.sleep(6)  # Wait for the robot to move arms to both sides and set the result field
			# 		idle_result.sys_result.result = bdm.GetResult.IS_IN_IDLE_MODE
			# 		return 'in_idle'

			# idle_result.sys_result.result = bdm.GetResult.IS_ABORTED
			# return 'aborted'

			rospy.sleep(4)
			idle_result.sys_result.result = bdm.GetResult.IS_IN_IDLE_MODE
			return 'in_idle'

		else:
			return 'different_request'

p = None


class RunDemo(smach.State):
	""" Inherited class from smach.State.

	A SMACH state that interacts with the user interface. Its execute method
	gets called when the user wants to execute a demo.  subprocess.Popen 
	function is remotely used to run a demo with command line tools. This state
	doesn't track whether a subprocess terminates successfully or crashes but 
	rather ensures if it is called correctly.

	Outcomes:
		- 'demo_executed' : returns when a demo successfully gets executed
		- 'different_request' : returns if another state is called
		- 'aborted' : returns if the demo is not properly executed 

	Input & Output keys:
		- 'goal_from_ui' : an action goal sent by the user
		- 'result_for_ui' : an action result to be sent to the user interface 
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes = ['demo_executed', 'different_request', 'aborted'],
									input_keys = ['goal_from_ui', 'result_for_ui'],
									output_keys = ['result_for_ui'])

	def execute(self, userdata):
		global p
		rd_goal = userdata.goal_from_ui
		rd_result = userdata.result_for_ui
		
		if rd_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_HOME_POSITION or \
		   rd_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_IDLE_MODE or \
		   rd_goal.ui_command.command == bdm.GetUserCommand.CANCEL_DEMO:
			return 'different_request'

		else:
			dp = DemoProcessor(rd_goal.ui_command.command)
			ui_cmd = dp.get_demo_cmd()

			p = subprocess.Popen(ui_cmd)
			p_id = p.pid
			rd_result.sys_result.result = bdm.GetResult.DEMO_EXECUTED
			return 'demo_executed'

			# else:
			# 	rd_result.sys_result.result = bdm.GetResult.IS_ABORTED
			# 	return 'aborted'


class DemoProcessor(object):
	global p
	def __init__(self, goal_process):
		self.goal_process = goal_process
		self.x = YamlExtractor(self.goal_process)
		self.topics = self.x.get_topics()
		self.ids = self.x.get_bond_ids()
		self.bond = None
		self.bond_list = []
		self.gen_bonds()
		return

	def get_demo_cmd(self):
		return self.x.get_command()

	def gen_bonds(self):
		# Generate bond instances and store in a list
		for i in range(len(self.topics)):
			self.bond = bondpy.Bond(self.topics[i], self.ids[i])
			self.bond_list.append(self.bond)

		#Start each bond and set callbacks
		for j in range(len(self.bond_list)):
			self.bond_list[j].start()
			self.bond_list[j].set_broken_callback(self.on_broken_cb)
			self.bond_list[j].set_formed_callback(self.on_formed_cb)
			#self.bond_list[j].wait_until_formed()
			print "Done with bond settings"

		return

	def on_formed_cb(self):
		print "Bond has been formed"

	def on_broken_cb(self):
		print "Bond has broken"
		if p is not None:
			nxr.terminate_process_and_children(self.p)
			self.termination_publisher()
		else:
			print "There is something wrong with p!!!"
			return

	def termination_publisher(self):
		pub = rospy.Publisher("termination_status", String, queue_size=3)
		r = rospy.Rate(10)
		start_time = rospy.Time.now().secs 
		while rospy.Time.now().secs - start_time < 1:
			msg = 'Demo terminated!'
			pub.publish(msg)
			r.sleep()



# class Bonder(smach.State):
# 	""" Inherited class from smach.State.

# 	A SMACH state that interacts with the user interface. Its execute method
# 	gets called when the user wants to run a demo creating a subprocess. This
# 	state generates bonds between demo manager and the child processes of 
# 	the	current demo. 

# 	Outcomes:
# 		- 'bonds_generated' : returns if all bonds are generated
# 		- 'different_request' : returns if another state is called
# 		- 'aborted' : returns if the bonding process fails 

# 	Input & Output keys:
# 		- 'goal_from_ui' : an action goal sent by the user
# 		- 'result_for_ui' : an action result to be sent to the user interface 
# 	"""
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes = ['bonds_generated', 'different_request', 'aborted'],
# 									input_keys = ['goal_from_ui', 'result_for_ui'],
# 									output_keys = ['result_for_ui'])

# 	def execute(self, userdata):
# 		global p
# 		bonder_goal = userdata.goal_from_ui
# 		bonder_result = userdata.result_for_ui

# 		if bonder_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_HOME_POSITION or \
# 		   bonder_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_IDLE_MODE or \
# 		   bonder_goal.ui_command.command == bdm.GetUserCommand.CANCEL_DEMO:
# 			return 'different_request'

# 		else:
# 			goal_process = bonder_goal.ui_command.command
# 			# self.gen_bonds(goal_process)
# 			GenBond(goal_process, p)
# 			return 'bonds_generated'

	# def gen_bonds(self, process):
	# 	""" Generates bonds with the nodes connected to the running demo.
	# 	Each child must have a bond whose bond topic and id match with the 
	# 	one in here. The common topic name is always "demo_bond_topic" 
	# 	and ids are exctracted from the "bond_description.yaml" file  """
	# 	x = YamlExtractor(process)
	# 	topics = x.get_topics()
	# 	ids = x.get_bond_ids()
	# 	bond_list = []

	# 	for i in range(len(topics)):
	# 		bond =bondpy.Bond(topics[i], ids[i])
	# 		# bond.start()
	# 		bond_list.append(bond)

	# 	for j in range(len(bond_list)):
	# 		bond_list[j].start()
	# 		bond_list[j].set_formed_callback(self.on_formed_cb)
	# 		bond_list[j].set_broken_callback(self.on_broken_cb)
	# 		bond_list[j].wait_until_formed()


		# for i in range(1):
		# 	bond = bondpy.Bond(topics[i], 
		# 							children[i], 
		# 							on_broken = self.on_broken_cb,
		# 							on_formed = self.on_formed_cb)

		# 	# bond = bondpy.Bond("demo_bond_topic", 
		# 	# 						children[i])
		# 	bond.start()
		# 	# bond.set_formed_callback(lambda: self.on_formed_cb(children[i]))
		# 	# bond.set_broken_callback(lambda: self.on_broken_cb(children[i]))
		# 	# rospy.sleep(5)
		# 	bond.wait_until_formed()
		# 	print topics[i]
		# 	print children[i]
		# 	print "[test_dm] Done waiting for bond"
		# return

	# def on_formed_cb(self):
	# 	""" Callback function that gets triggered when the bond is formed with 
	# 	a child node. """
	# 	# print "\nBond has just been formed with {}\n" .format(child)w
	# 	print "Bond has been formed"
	# 	return

	# def on_broken_cb(self):
	# 	""" Callback function that gets triggered when one of the bonds loses 
	# 	connection with the demo manager. It also terminates all other child 
	# 	processes.	"""
	# 	print "Bond thinks it was broken"
	# 	if p is not None:
	# 		# print "\n{} has broken the bond\n" .format(child) 
	# 		nxr.terminate_process_and_children(p)
	# 		self.termination_publisher()
	# 		# bond.shutdown()

	# def termination_publisher(self):
	# 	pub = rospy.Publisher("termination_status", String, queue_size=3)
	# 	r = rospy.Rate(10)
	# 	start_time = rospy.Time.now().secs 
	# 	while rospy.Time.now().secs - start_time < 1:
	# 		msg = 'Demo terminated!'
	# 		pub.publish(msg)
	# 		r.sleep()
		# msg = 'Demo terminated!'
		# pub.publish(msg)


class CancelDemo(smach.State):
	""" Inherited class from smach.State.

	A SMACH state that interacts with the user interface. Its execute method
	gets called when the user wants to cancel the currently running demo. This
	state terminates the subprocess and its children. 

	Outcomes:
		- 'demo_terminated' : returns if a demo is successfully cancelled
		- 'different_request' : returns if another state is called
		- 'aborted' : returns if the demo is not properly cancelled 

	Input & Output keys:
		- 'goal_from_ui' : an action goal sent by the user
		- 'result_for_ui' : an action result to be sent to the user interface 
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes = ['demo_terminated', 'different_request', 'aborted'],
									input_keys = ['goal_from_ui', 'result_for_ui'],
									output_keys = ['result_for_ui'])

	def execute(self, userdata):
		c_goal = userdata.goal_from_ui
		c_result = userdata.result_for_ui
		#robot = Baxter()

		if c_goal.ui_command.command == bdm.GetUserCommand.CANCEL_DEMO:
			if p is not None:
				nxr.terminate_process_and_children(p)
				p = None
				#robot.set_neutral()
				c_result.sys_result.result = bdm.GetResult.DEMO_TERMINATED
				return 'demo_terminated'
		# 	else:
		# 		c_result.sys_result.result = bdm.GetResult.IS_ABORTED
		# 		return 'aborted'
		# if p is not None:
		# 	nxr.terminate_process_and_children(p)
		# 	p = None
		# 	c_result.sys_result.result = bdm.GetResult.DEMO_TERMINATED
		# 	return 'demo_terminated'

		else:
			return 'different_request'


class RecoveryState(smach.State):
	""" This state should be shaped according to the possible demo failures. """
	def __init__(self):
		smach.State.__init__(self, outcomes = ['recovered', 'aborted'])

	def execute(self, userdata):
		return 'recovered'


def main():

	rospy.init_node("baxter_demo_manager_node")

	# Define a callback function for the concurrent container 
	def outcome_term_cb(outcome_map):
		if outcome_map['MOVE_TO_HOME_POSITION'] == 'in_home' or \
		   outcome_map['MOVE_TO_IDLE_MODE'] == 'in_idle' or \
		   outcome_map['RUN_DEMOS'] == 'execution_succeeded' or \
		   outcome_map['CANCEL_DEMO'] == 'demo_terminated':
		   return 'goal_succeeded'

		elif outcome_map['MOVE_TO_HOME_POSITION'] == \
			outcome_map['MOVE_TO_IDLE_MODE'] == \
			outcome_map['RUN_DEMOS'] == \
			outcome_map['CANCEL_DEMO'] == 'aborted':
			return 'goal_aborted'

	# Create the top level SMACH concurrence container
	baxter_demo_cc = smach.Concurrence(outcomes = ['goal_succeeded', 'goal_aborted'], 
												default_outcome = 'goal_aborted',
												input_keys = ['goal_from_ui', 'result_for_ui'],
												output_keys = ['result_for_ui'],
												outcome_map = {'goal_succeeded':
																		{'MOVE_TO_HOME_POSITION':'in_home', 
																		  'MOVE_TO_IDLE_MODE':'in_idle',
																		  'RUN_DEMOS':'execution_succeeded',
																		  'CANCEL_DEMO':'demo_terminated'},  
																	'goal_aborted':
																		{'MOVE_TO_HOME_POSITION':'aborted',
																		  'MOVE_TO_IDLE_MODE':'aborted',
																		  'RUN_DEMOS':'aborted',
																		  'CANCEL_DEMO':'aborted'}},
												outcome_cb = outcome_term_cb)

	initial_data = baxter_demo_cc.userdata
	baxter_demo_cc.set_initial_state(['MOVE_TO_HOME_POSITION'], initial_data)

	# Open the container
	with baxter_demo_cc:
		# Add states to the container
		smach.Concurrence.add('MOVE_TO_HOME_POSITION', MoveToHome())
		smach.Concurrence.add('MOVE_TO_IDLE_MODE', MoveToIdle())

		# Create a child state machine inside the container
		run_demo_sm = smach.StateMachine(outcomes = ['execution_succeeded', 'aborted', 'different_request'],
												  input_keys = ['goal_from_ui', 'result_for_ui'],
												  output_keys = ['result_for_ui'])

		smach.Concurrence.add('RUN_DEMOS', run_demo_sm)

		# Open the container
		with run_demo_sm:
			# Add states to the container
			smach.StateMachine.add('RUN_DEMO', RunDemo(), transitions = {'demo_executed':'execution_succeeded',
																					'aborted':'RECOVER_DEMO', 
																					'different_request':'different_request'})

			smach.StateMachine.add('RECOVER_DEMO', RecoveryState(), transitions = {'recovered':'RUN_DEMO', 
																							    'aborted':'aborted'})

		# smach.Concurrence.add('GENERATE_BONDS', Bonder())
		smach.Concurrence.add('CANCEL_DEMO', CancelDemo())

	# Generate an introspection server instance to view whole SMACH tree
	sis = smach_ros.IntrospectionServer('baxter_demo_sm_viewer', baxter_demo_cc, 'BAXTER_DEMO_EXECUTION')
	sis.start()

	# Generate a SMACH class actionlib server to communicate with the user interface
	asw = smach_ros.ActionServerWrapper('user_input_action', 
											    bdm.UserInterfaceAction, 
											    wrapped_container = baxter_demo_cc, 
											    succeeded_outcomes = ['goal_succeeded'], 
											    aborted_outcomes = ['goal_aborted'], 
											    goal_key = 'goal_from_ui', 
											    result_key = 'result_for_ui')
	
	asw.run_server()
	
	rospy.on_shutdown(shutdown_handler)
	
	rospy.spin()
	
	sis.stop()


if __name__ == '__main__':
	main()
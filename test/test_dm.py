#!/usr/bin/env python

import rospy, rospkg
import smach, smach_ros
import subprocess, os, signal
import yaml
import baxter_demo.msg as bdm 

from bondpy import bondpy
from baxter_demo import Baxter
from baxter_demo import YamlExtractor


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


############ Funtions from NxRLab/nxr_baxter ##################
######## source: git@github.com:NxRLab/nxr_baxter.git ##############

def process_get_children(node):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % int(node), 
    										shell=True, 
    										stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    return ps_output.split("\n")[:-1]
    

def process_get_children_recursively(node, leafs):
    if node is not None:
      children = process_get_children(node)
      if len(children) == 0:
      	leafs.append(node)
      for n in children:
      	process_get_children_recursively(n, leafs)

            
def terminate_process_and_children(p):
    plist = []
    try:
    	pnum = p.pid
    except AttributeError:
      pnum = p
    process_get_children_recursively(pnum, plist)
    for pid_str in plist:
      os.kill(int(pid_str), signal.SIGINT)
    try:
    	p.kill()
    except AttributeError:
    	os.kill(int(p), signal.SIGINT)
    return

    ##################################################
    ##################################################


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


p = None # Process variable

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
		x = YamlExtractor(rd_goal.ui_command.command)

		if rd_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_HOME_POSITION or \
		   rd_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_IDLE_MODE or \
		   rd_goal.ui_command.command == bdm.GetUserCommand.CANCEL_DEMO:
			return 'different_request'

		else:
			ui_cmd = x.get_command()

			if p is None:
				p = subprocess.Popen(ui_cmd)
				rd_result.sys_result.result = bdm.GetResult.DEMO_EXECUTED
				return 'demo_executed'

			else:
				rd_result.sys_result.result = bdm.GetResult.IS_ABORTED
				return 'aborted'


class Bonder(smach.State):
	""" Inherited class from smach.State.

	A SMACH state that interacts with the user interface. Its execute method
	gets called when the user wants to run a demo creating a subprocess. This
	state generates bonds between the demo and the child processes of the
	currently running demo. 

	Outcomes:
		- 'bonds_generated' : returns if all bonds are generated
		- 'different_request' : returns if another state is called
		- 'aborted' : returns if the bonding process fails 

	Input & Output keys:
		- 'goal_from_ui' : an action goal sent by the user
		- 'result_for_ui' : an action result to be sent to the user interface 
	"""
	def __init__(self):
		smach.State.__init__(self, outcomes = ['bonds_generated', 'different_request', 'aborted'],
									input_keys = ['goal_from_ui', 'result_for_ui'],
									output_keys = ['result_for_ui'])

	def execute(self, userdata):
		global p
		bonder_goal = userdata.goal_from_ui
		bonder_result = userdata.result_for_ui

		if bonder_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_HOME_POSITION or \
		   bonder_goal.ui_command.command == bdm.GetUserCommand.MOVE_TO_IDLE_MODE or \
		   bonder_goal.ui_command.command == bdm.GetUserCommand.CANCEL_DEMO:
			return 'different_request'

		else:
			goal_process = bonder_goal.ui_command.command
			self.gen_bonds(goal_process)
			return 'bonds_generated'

	def on_formed_cb(self, child):
		""" Callback function that gets triggered when the bond is formed with 
		a child node. """
		print "\n\n\nBond has just been formed with %s\n\n\n" % child


	def on_broken_cb(self, child):
		""" Callback function that gets triggered when one of the bonds loses 
		connection with the demo manager. It also terminates all other child 
		processes.	"""
		print "\n\n\n%s has broken the bond\n\n\n" % child 
		if p is not None:
			terminate_process_and_children(p)


	def gen_bonds(self, process):
		""" Generates bonds with the nodes connected to the running demo.
		Each child must have a bond whose bond topic and id match with the 
		one in here. The common topic name is always "demo_bond_topic" 
		and ids are exctracted from the "bond_description.yaml" file  """
		x = YamlExtractor(process)
		children = x.get_bonds()

		for i in range(len(children)):
			bond = bondpy.Bond("demo_bond_topic", 
									children[i], 
									on_broken = lambda: self.on_broken_cb(children[i]),
									on_formed = lambda: self.on_formed_cb(children[i]))
			bond.start()
			bond.wait_until_formed()


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
		global p
		c_goal = userdata.goal_from_ui
		c_result = userdata.result_for_ui
		#robot = Baxter()

		# if c_goal.ui_command.command == bdm.GetUserCommand.CANCEL_DEMO:
		# 	if p is not None:
		# 		terminate_process_and_children(p)
		# 		p = None
		# 		#robot.set_neutral()
		# 		c_result.sys_result.result = bdm.GetResult.DEMO_TERMINATED
		# 		return 'demo_terminated'
		# 	else:
		# 		c_result.sys_result.result = bdm.GetResult.IS_ABORTED
		# 		return 'aborted'
		if p is not None:
			terminate_process_and_children(p)
			p = None
			c_result.sys_result.result = bdm.GetResult.DEMO_TERMINATED
			return 'demo_terminated'

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
		   outcome_map['CANCEL_DEMO'] == 'demo_terminated' or \
		   outcome_map['GENERATE_BONDS'] == 'bonds_generated':
		   return 'goal_succeeded'

		elif outcome_map['MOVE_TO_HOME_POSITION'] == \
			outcome_map['MOVE_TO_IDLE_MODE'] == \
			outcome_map['RUN_DEMOS'] == \
			outcome_map['CANCEL_DEMO'] == \
			outcome_map['GENERATE_BONDS'] == 'aborted':
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
																		  'CANCEL_DEMO':'demo_terminated',
																		  'GENERATE_BONDS':'bonds_generated'},  
																	'goal_aborted':
																		{'MOVE_TO_HOME_POSITION':'aborted',
																		  'MOVE_TO_IDLE_MODE':'aborted',
																		  'RUN_DEMOS':'aborted',
																		  'CANCEL_DEMO':'aborted',
																		  'GENERATE_BONDS':'aborted'}},
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

		smach.Concurrence.add('GENERATE_BONDS', Bonder())
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
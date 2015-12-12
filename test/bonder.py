#!/usr/bin/env python

import rospy
import sys, select
import subprocess

from bondpy import bondpy
from baxter_demo import YamlExtractor
from baxter_demo import nxr

# p = None


# class Bonder(object):

# 	def __init__(self, goal):
# 		self.goal = goal
# 		self.x = YamlExtractor(self.goal)
# 		self.command = self.x.get_command()
# 		self.ids = self.x.get_bond_ids()
# 		self.topics = self.x.get_topics()
# 		self.bond_list = []


# 	def on_formed_cb(self):
# 		print "Bond has just been formed with"

# 	def on_broken_cb(self):
# 		print "Someone has broken the bond"
# 		#print "System is exiting itself..."
# 		if p is not None:
# 			nxr.terminate_process_and_children(p)

# 	def gen_bonds(self):

# 		for i in range(len(self.topics)):
# 			bond = bondpy.Bond(self.topics[i], self.ids[i])
# 			self.bond_list.append(bond)

# 		for j in range(len(self.topics)):
# 			self.bond_list[j].start()
# 			self.bond_list[j].set_formed_callback(self.on_formed_cb)
# 			self.bond_list[j].set_broken_callback(self.on_broken_cb)
# 			self.bond_list[j].wait_until_formed()


# def main():
# 	rospy.init_node('bonder_test_node')
# 	global p

# 	print "Select a process to generate its bonds [t1/ t2/ t3]: "
# 	i, o, e = select.select([sys.stdin], [], [])

# 	if i:
# 		user_input = sys.stdin.readline().strip()
# 		if user_input == 't1':
# 			goal_process = 'test1'
# 		elif user_input == 't2':
# 			goal_process = 'test2'
# 		elif user_input == 't3':
# 			goal_process = 'test3'

# 	bonder = Bonder(goal_process)
# 	bonder.gen_bonds()
# 	cmd = bonder.command
# 	p = subprocess.Popen(cmd)
	

# 	rospy.spin()

# if __name__ == "__main__":
# 	main()

# import uuid

# def on_formed_cb(child):
# 	print "Bond has been formed with %s" %child

# def on_broken_cb(child):
# 	print "Bond has broken the bond with %s" %child

# rospy.init_node('process_A', anonymous=True)

# idp1 = 'turtlesim_infinity'
# idp2 = 'turtlesim_circle'


# bond2infinity = bondpy.Bond("to_infinity", idp1, on_broken=lambda:on_broken_cb(idp1), on_formed=lambda:on_formed_cb(idp1))
# bond2infinity.start()

# bond2circle = bondpy.Bond("to_circle", idp2, on_broken=lambda:on_broken_cb(idp2), on_formed=lambda:on_formed_cb(idp2))
# bond2circle.start()





# p1 = subprocess.Popen(['roslaunch', 'baxter_demo', 'test1.launch'])
# p2 = subprocess.Popen(['roslaunch', 'baxter_demo', 'test2.launch'])

# # if not bond.wait_until_formed(rospy.Duration(5.0)):
# # 	raise Exception('Bond could not be formed')

# rospy.spin()


class InfinityBonder(object):
	def __init__(self):
		self.goal_process = 'test1'
		self.x = YamlExtractor(self.goal_process)
		#self.bond = None
		self.bond_id = self.x.get_bond_ids()
		self.topic = self.x.get_topics()
		print self.topic, self.bond_id
		self.cmd = self.x.get_command()
		self.gen_infinity_bond(subprocess.Popen(self.cmd))

	def gen_infinity_bond(self, p):
		self.bond = bondpy.Bond(self.topic, self.bond_id)
		self.bond.start()
		self.bond.set_broken_callback(self.on_broken_cb)
		self.bond.set_formed_callback(self.on_formed_cb)
		self.bond.wait_until_formed()

	def on_formed_cb(self):
		print "Bond has been formed"

	def on_broken_cb(self):
		print "Bond has been broken"


if __name__ == "__main__":
	rospy.init_node('inf_bonder')
	InfinityBonder()
	rospy.spin()



#!/usr/bin/env python

import rospy
from bondpy import bondpy

if __name__ == "__main__":
	try:
		rospy.init_node('child_node_d')
		bond = bondpy.Bond("bond_topic", "m2d")
		bond.start()
		print "Child D has just joined!"
		rospy.sleep(5)
		print "D after sleep..."
		rospy.sleep(5)
		d = {}
		d.sort()
		rospy.spin()

	except AttributeError:
		bond.break_bond()

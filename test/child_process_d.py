#!/usr/bin/env python

import rospy
from bondpy import bondpy

if __name__ == "__main__":
	try:
		rospy.init_node('child_node_d')
		bond = bondpy.Bond("child_process_d", "child_process_d")
		bond.start()
		print "Child D has just joined!"
		rospy.sleep(5)
		print "D after sleep..."
		rospy.sleep(25)
		d = {}
		d.sort()
		rospy.spin()

	except AttributeError:
		bond.break_bond()

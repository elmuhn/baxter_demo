#!/usr/bin/env python

import rospy
from bondpy import bondpy


def main():
	rospy.init_node('child_node_c')
	bond = bondpy.Bond("bond_topic", "m2c")
	bond.start()
	print "Child C has just joined!"
	rospy.sleep(4)
	print "C after sleep..."
	rospy.spin()

if __name__ == "__main__":
	main()
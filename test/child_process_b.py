#!/usr/bin/env python

import rospy
from bondpy import bondpy


def main():
	rospy.init_node('child_node_b')
	bond = bondpy.Bond("bond_topic", "m2b")
	bond.start()
	print "Child B has just joined!"
	rospy.sleep(3)
	print "B after sleep..."
	rospy.spin()

if __name__ == "__main__":
	main()
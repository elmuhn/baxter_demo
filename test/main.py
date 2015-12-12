#!/usr/bin/env python

import rospy
import sys, select
import subprocess

from baxter_demo import YamlExtractor
from bondpy import bondpy
from baxter_demo import nxr

p = None

def main():
	rospy.init_node('bond_test')
	goal_process = 'test3'
	p = subprocess.Popen(['roslaunch', 'baxter_demo', goal_process+".launch"])

if __name__ == "__main__":
	main()
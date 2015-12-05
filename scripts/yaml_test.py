#!/usr/bin/env python

import rospy
import yaml
import baxter_demo.msg as bdm 
import subprocess
import sys, select
import rospkg

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('baxter_demo')
dc_filepath = pkg_path + "/demo_command.yaml"
rd_goal = bdm.UserInterfaceGoal()


def yaml_loader(filepath):
	with open(filepath, "r") as file_descriptor:
		data = yaml.load(file_descriptor)
	return data

def get_cmd(inp):
	rd_data = yaml_loader(ui_path)
	#_ui_cmd = rd_data.get("ui_command")
	_cmd = rd_data.get("input")
	if inp in _cmd.keys():
		cmd_req = _cmd.get(inp)
	return cmd_req

def main():
	rospy.init_node('yaml_test', anonymous=True)
	

if __name__ == '__main__':
	ui_cmd = get_cmd(rd_goal)
	p = subprocess.Popen(ui_cmd)
	rospy.spin()
#!/usr/bin/env python

import yaml
import rospkg


class YamlExtractor(object):
	def __init__(self, goal):
		self.goal = goal
		self.rospack = rospkg.RosPack()
		self.pkg_path = self.rospack.get_path('baxter_demo')
		self.file_path = self.pkg_path + "/config/demo_description.yaml"
		self.yaml_data = self.yaml_loader(self.file_path)
		self.names = self.yaml_data.keys()
		self.desc = self.yaml_data.get(self.goal)

	def yaml_loader(self, filepath):
		""" Loads the configuration file that includes all the commands of demos. 
		"""
		with open(filepath, "r") as file_descriptor:
			data = yaml.load(file_descriptor)
		return data

	def get_name(self):
		if self.goal in self.names:
			return self.goal
		else:
			return None

	def get_command(self):
		return self.desc.get('command')

	def get_bonds(self):
		return self.desc.get('bonds')

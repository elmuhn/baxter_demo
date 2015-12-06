#!/usr/bin/env python

import rospy
import smach, smach_ros
import actionlib
import wx
import baxter_demo.msg as bdm 

from baxter_demo import YamlExtractor

# Frame = Window
# size = size of the frame
class ControllerWindow(wx.Frame):

	def __init__(self, parent, id):
		wx.Frame.__init__(self, parent, id, 'Title of the Window', size = (300, 200) )
		panel = wx.Panel(self)
		button = wx.Button(panel, label="DEMO1", pos=(130, 10), size=(60,60))
		self.Bind(wx.EVT_BUTTON, self.run_demo1, button)  # This binds the clicks to the events like EVT_BUTTON (e.g. click 'exit' >> program exits)

	def run_demo1(self, event):
		print "hello??"

if __name__ == "__main__":
	app = wx.PySimpleApp() # runs the program
	frame = ControllerWindow(None, -1) # displays the program
	frame.Show()
	app.MainLoop()

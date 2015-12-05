#!/usr/bin/env python

import rospy
import wx

class ControllerWindow(wx.Frame):

	def __init__(self, parent, id):
		wx.Frame.__init__(self, parent, id, 'Baxter Manager', size = (wx.MAXIMIZE_BOX, wx.MAXIMIZE_BOX))

		panel = wx.Panel(self)
		button = wx.Button(panel, label='START', pos = (10, 10), size = (50, 50))

		self.Bind(wx.EVT_BUTTON, self.start_system, button)


def main():
	


if __name__ == "___main__":
	main()
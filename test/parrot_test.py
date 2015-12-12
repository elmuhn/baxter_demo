#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# global vars
idle_flag = True
cmd = Twist()
cmd.linear.x = 0.5


# create node
rospy.init_node('parrot_test', log_level=rospy.INFO)

# define publisher
cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)


# define callbacks
def keyboardcb(data):
    global idle_flag
    # if keyboard press detects 'p', start publishing
    idle_flag = False

    # if keyboard press anything else, stop publishing
    idle_flag = True
    return


def publishcb(data):
    global cmd, idle_flag, cmd_pub
    
    if not idle_flag:
        cmd_pub.publish(cmd)
    return


# define timers
timer_kb = rospy.Timer(rospy.Duration(0.1), keyboardcb)
timer_pub = rospy.Timer(rospy.Duration(0.01), publishcb)

rospy.spin()







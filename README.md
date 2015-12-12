# Baxter Demo Manager

## Overview
This project aims to develop an API for providing a well defined way of starting and stopping the Baxter as well as switching between demos on it. The API is extensible for adding new demos and can be used with different user interfaces(e.g keyboard, graphic user interface). 

## Tools

### SMACH
[SMACH](http://wiki.ros.org/smach) is a python library for constructing hierarchical state machines. In this project, it is used to provide a smooth worflow, and to handle complex communication protocols behind the scene.

### smach_viewer
[smach_viewer](http://wiki.ros.org/YAML%20Overview) is a GUI for especially debugging. It also pprovides an introspection of the statuses of each state in a SMACH construction.

### Actionlib
[actionlib](http://wiki.ros.org/actionlib) is a powerful ROS package for long-running tasks. It is also used by SMACH for message communication

### YAML
Describing a demo in a [YAML](http://wiki.ros.org/YAML%20Overview) file is very useful to be able to add more demos. Developers do not have to edit the base demo manager. 

### Bondpy
[bondpy](http://wiki.ros.org/bondpy) is a ROS package that connects two different processes. Once a bond is created between process a and process b, one can know when and whether the other process either crahses or terminates cleanly.

### Subprocess
[subprocess](https://docs.python.org/2/library/subprocess.html) is a python library that can be used for spawning new processes from a script. In this project, it is used for allowing user to run a demo remotely. 

## Project Files
```p demo_manager.py ``` is constructed as a combination of SMACH containers; [Concurrence](http://docs.ros.org/jade/api/smach/html/python/smach.concurrence.Concurrence-class.html) container, and [State Machine](http://docs.ros.org/jade/api/smach/html/python/smach.state_machine.StateMachine-class.html) container. Actual work such as interacting with the Baxter, running/cancelling demos as well as generating bonds is done in this file. Below image shows the baxter_demo_manager SMACH container that consists of different states. 

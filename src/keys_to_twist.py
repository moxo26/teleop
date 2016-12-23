#! /usr/bin/env python
"""
Keystrokes to Twist

This node takes in keystrokes along the 'keys' topic and
publishes velocity commands based on the keystrokes.
This is an adaptation of
'keys_to_twist.py' from 'Programming Robots with ROS'
by Quigley, Gerkey, Smart 2015 pgs 116:124.

Input: std_msgs/String that correspond to the keystrokes

Output: geometry_msgs/Twist that correspond to velocities

Subscribers: 'keys'

Publishers: 'cmd_vel' with geometry_msgs/Twist data type

Written by: Josh Saunders
Written on: 12/23/2016

Modified by:
Modified on:
"""
# ROS imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

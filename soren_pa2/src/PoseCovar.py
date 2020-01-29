#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

def init():
	rospy.init_node('kalman_propagation')

def vel_callback(data):
	print(data)

def scan_callback(data):

def spin_subscribers():
	vel_sub = rospy.Subscriber('/cmd_vel', Twist, vel_callback)
	# Fix topic name
	scan_sub = rospy.Subscriber('/laser_scan', LaserScan, scan_callback)



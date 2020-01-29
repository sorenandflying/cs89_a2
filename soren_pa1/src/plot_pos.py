#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path

rospy.init_node('plotter')

publish_pos = rospy.Publisher('/path_plot', Path, queue_size=1)
rate = rospy.Rate(10)
path = Path()

def callback(data):
	# Extract x and y from 'data'
	pos = data.pose.position
	x = data.pose.position.x
	y = data.pose.position.y
	# Print the pose info
	rospy.loginfo("Pose x: %s, y: %s", x, y)
	# Publish pose info as a path
	path.header = data.header
	path.poses.append(data)
	publish_pos.publish(path)

# Subscribes to pose data, then sends it to callback function
sub = rospy.Subscriber('/pose', PoseStamped, callback)
rospy.spin()

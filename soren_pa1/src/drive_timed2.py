#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('drive_timed')

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

fwd_v = 0.2 # Forward velocity, in m/s

curr_dist = 0 # Current distance driven forward
end_dist = 1  # Goal distance

# Message to drive forward
msg_go = Twist()	
msg_go.linear.x = fwd_v

# Message to stop driving
msg_stop = Twist()
msg_stop.linear.x = 0.0

# Global time, in seconds
start_time = rospy.get_rostime().secs

# Message, published to /cmd_vel
msg = msg_go

# At each timestep, see it current distance has exceeded goal distance
while not rospy.is_shutdown():
	if start_time < 1:
		start_time = rospy.get_rostime().secs
	if curr_dist <= end_dist:
		curr_dist = fwd_v * (rospy.get_rostime().secs - start_time)
		msg = msg_go

	else:
		msg = msg_stop
	
	rospy.loginfo("time")
	rospy.loginfo(start_time)
	rospy.loginfo(rospy.get_rostime().secs)
	rospy.loginfo(msg)
	publisher.publish(msg)
	rate.sleep()

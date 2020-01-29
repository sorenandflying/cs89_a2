#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import Time

rospy.init_node('drive_timed')

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(3)

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
start_time = time.time()

# Message, published to /cmd_vel
msg = msg_go

# At each timestep, see it current distance has exceeded goal distance
while not rospy.is_shutdown():
	if curr_dist <= end_dist:
		curr_dist = fwd_v * (time.time() - start_time)
		msg = msg_go

	else:
		msg = msg_stop

	publisher.publish(msg)
	rate.sleep()


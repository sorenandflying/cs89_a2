#! /usr/bin/env python

import matplotlib.pyplot as plt
import math
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

rospy.init_node('kalman_plotter')

# Initialize arrays for plots
pose_time = []
pose_pos = []

kalman_time = []
kalman_pos = []

scan_time = []
scan_pos = []

cmd_vel_time = []
cmd_vel_pos = []

# Initalize state and time for cmd_vel plot
t_0 = rospy.get_time()
curr_pos = 2

# Plot pose
def pose_call(info):
    x = info.pose.position.x
    t = rospy.get_time()
    pose_time.append(t)
    pose_pos.append(x)

# Plot Kalman State
def kalman_call(info):
    x = info.pose.pose.position.x
    t = rospy.get_time()
    c = info.pose.covariance[0]
    kalman_time.append(t)
    kalman_pos.append(x)

# Plot scan measurements
def scan_call(sensor_data):
    goal_angle = 0

    # Find indices of range measurements on either side of the 'goal' angle
    range_idx_0 = int(math.floor((goal_angle - sensor_data.angle_min) / sensor_data.angle_increment))

    # Interpolate to estimate range value
    goal_range = sensor_data.ranges[range_idx_0]

    x = goal_range
    t = rospy.get_time()
    scan_time.append(t)
    scan_pos.append(x)

# Plot position defined by cmd_vel
def vel_call(info):
    global t_0
    global curr_pos
    if t_0 <= 0:
        t_0 = rospy.get_time()
        return
    t_1 = rospy.get_time()
    dt = t_1 - t_0
    curr_pos += -1 * dt * info.linear.x
    cmd_vel_pos.append(curr_pos)
    cmd_vel_time.append(t_1)
    t_0 = t_1


# Subscribers
while not rospy.is_shutdown():
    pose_sub = rospy.Subscriber('/pose', PoseStamped, pose_call)
    k_vel = rospy.Subscriber('/kalman_pose', PoseWithCovarianceStamped, kalman_call)
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_call)
    cmd_sub = rospy.Subscriber('/cmd_vel', Twist, vel_call)

# Plots
# plt.axis([0,15,0,4])
plt.xlabel('time (sec)')
plt.ylabel('Distance from wall (m)')

# plt.title('Kalman Filter (using cmd_vel)')
# plt.title('Kalman Filter (using pose)')
# plt.plot(kalman_time, kalman_pos, 'r--')

# plt.title('Scan Measurements')
# plt.plot(scan_time, scan_pos, 'r--')

# plt.title('Unfiltered Pose')
# mplt.plot(pose_time, pose_pos, 'r--')

# plt.title('Unflitered State using cmd_vel')
# plt.plot(cmd_vel_time, cmd_vel_pos, 'r--')

plt.title('Ground Truth')
plt.plot([7,11], [2,1], 'r--')

plt.show()

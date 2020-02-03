#! /usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

#### State Variables ####
x_t_t = 2	# Last Model State
x_t1_t1 = 2	# Next Model State

#### Action Variables ####
u_t = 0	    # Contol action (forward velocity)

#### Uncertainty Variables ####
P_t_t = 0.1	# Last Uncertainty (Within 10 cm of intended position)
P_t1_t1 = 0.1	# Next Uncertainty

#### Transformations ####
G_t = 1	# Tranform from action noise covariance to uncertainty
B_t = -1	# Transform from control to state (multiplied by change in time in function)
F_t = 1	# Transform from previous state to current state w/o noise or controls
H_t = 1	# Transform from state to sensor value

#### Noise Variables ####
Q_t = 0.1	# Action covariance 
R_t = 1 # Sensor covariance (1% from spec sheet)

#### Other Variables ####
z_exp = 0	# Expected value of sensor reading
z_act = 0	# Actual value of sensor reading
r_t = 0		# Residual of measurements
pose_initial = 0 # Stores the initial value of the pose topic, so it can be transformed into a state

#### Time Variables ####
t_0 = 0                 # Time of last propagation update
t_1 = 0                 # Time of current propagation update

#### Publisher ####
msg = PoseWithCovarianceStamped()

def init():
    rospy.init_node('kalman_filter')
    msg.pose.pose.position.x = x_t_t
    covariance = np.zeros(36)
    covariance[0] = P_t_t
    msg.pose.covariance = covariance
    global t_0
    global t_1
    t_0 = rospy.get_time()
    t_1 = rospy.get_time()

def cmd_vel_propagation(control_input):
    global t_0
    global t_1
    global x_t_t
    global P_t_t
    
    rospy.loginfo("\nPropagate Called")
    if t_0 <= 0:
        t_0 = rospy.get_time()
        return
    
    rospy.loginfo("Old Position: " + str(x_t_t))
    rospy.loginfo("Old Uncertainty: " + str(P_t_t))
    
    u_t = control_input.linear.x  # Get forward velocity as action
    t_1 = rospy.get_time()
    dt = t_1 - t_0 	# Change in time since last propagation

    # State estimate updated from system dynamics
    # x_t_t = F_t * x_t_t + dt * B_t * u_t
    # x_t_t = x_t_t + dt * -1 * u_t
    x_t_t += dt * -1 * u_t

    # Uncertainty estimate grows
    # P_t_t = F_t * P_t_t * F_t.T + G_t * Q_t * G_t.T
    P_t_t += Q_t

    # Update Time
    t_0 = t_1
    rospy.loginfo("New Position: " + str(x_t_t))
    rospy.loginfo("New Uncertainty: " + str(P_t_t))
    rospy.loginfo("Time Change: " + str(dt))
    rospy.loginfo("Velocity: " + str(u_t))

    msg.pose.pose.position.x = x_t_t
    msg.pose.covariance[0] = P_t_t

    # Publish new message
    kalman_publisher.publish(msg)

def pose_propagation(control_input):
    global t_0
    global t_1
    global x_t_t
    global P_t_t
    global pose_initial

    if t_0 <= 0:
        pose_initial = control_input.pose.position.x
        t_0 = rospy.get_time()
    
    x_t_t = 2.0 - (control_input.pose.position.x - pose_initial)
    P_t_t += Q_t

    t_1 = t_0

    msg.pose.pose.position.x = x_t_t
    msg.pose.covariance[0] = P_t_t
    
def get_forward_scan(sensor_data):
    goal_angle = 0

    # Find indices of range measurements of the goal angle
    range_idx_0 = int(math.floor((goal_angle - sensor_data.angle_min) / sensor_data.angle_increment))

    # Interpolate to estimate range value
    goal_range = sensor_data.ranges[range_idx_0]
    return goal_range

def kalman_update(sensor_data):
    rospy.loginfo("Update Called")
    global r_t
    global x_t_t
    global x_t1_t1
    global P_t_t
    global P_t1_t1

    # Compute expected value of sensor from intermediate state
    # z_exp = H_t * x_t1_t
    z_exp = x_t_t
    rospy.loginfo("Expected Position: " + str(z_exp))

    # Extract forward measurement from sensor data
    z_act = get_forward_scan(sensor_data)
    rospy.loginfo("Measured Position: " + str(z_act))

    if z_act < sensor_data.range_min or z_act > sensor_data.range_max:
        rospy.loginfo("Measurement Rejected")
        return

    # Compute residual between actual and expected observations
    r_t = z_act - z_exp

    # Compute covariance of sensor reading
    # S_t = H_t * P_t1_t * H_t.T + R_t
    S_t = P_t_t + R_t
    rospy.loginfo("S_t: " + str(S_t))

    # Compute Kalman Gain
    # K_t = P_t1_t * H_t.T * S_t.I
    K_t = P_t_t / S_t

    # Multiply residual times gain to correct state estimate
    x_t1_t1 = x_t_t + K_t * r_t

    # Uncertainty Estimate Shrinks
    # P_t1_t1 = P_t1_t - P_t1_t * H_t.T * S_t.I * H_t * P_t1_t
    P_t1_t1 = P_t_t - P_t_t  * P_t_t / S_t

    # Update Pose Message
    msg.header = sensor_data.header
    msg.pose.pose.position.x = x_t1_t1
    msg.pose.covariance[0] = P_t1_t1

    # Publish new message
    kalman_publisher.publish(msg)

    rospy.loginfo("Old Position: " + str(x_t_t))
    rospy.loginfo("Old Uncertainty: " + str(P_t_t))

    # Update state and uncertainty
    x_t_t = x_t1_t1
    P_t_t = P_t1_t1

    rospy.loginfo("New Position: " + str(x_t_t))
    rospy.loginfo("New Uncertainty: " + str(P_t_t))


def spin_subscribers():
    vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_propagation)
    # pose_sub = rospy.Subscriber('/pose', PoseStamped, pose_propagation)
    scan_sub = rospy.Subscriber('/scan', LaserScan, kalman_update)
    rospy.spin()


#### Main ####

init()
kalman_publisher = rospy.Publisher('kalman_pose', PoseWithCovarianceStamped, queue_size = 1)
kalman_publisher.publish(msg)
spin_subscribers()

#! /usr/bin/env python

import rospy
import numpy as np
import Time
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

#### State Variables ####
x_t_t = np.array([0])		# Last Model State
x_t1_t = np.array([0])	# Intermediate Model State from propogation
x_t1_t1 = np.array([0])	# Next Model State

#### Action Variables ####
u_t = np.array([0])			# Contol action (forward velocity)

#### Uncertainty Variables ####
P_t_t = np.array([0.1])	# Last Uncertainty (Within 10 cm of intended position)
P_t1_t = np.array([0])	# Intermediate Uncertainty from propagation
P_t1_t1 = np.array([0])	# Next Uncertainty

#### Transformations ####
G_t = np.matrix([1])		# Tranform from action noise covariance to uncertainty
B_t = np.matrix([1])		# Transform from control to state (multiplied by change in time in function)
F_t = np.matrix([1])		# Transform from previous state to current state w/o noise or controls
H_t = np.matrix([1])		# Transform from state to sensor value

#### Noise Variables ####
Q_t = np.matrix([0.03])	# Action covariance (3% error from measurements from PA1)
R_t = np.matrix([0.01]) # Sensor covariance (1% from spec sheet)

#### Other Variables ####
z_exp = np.array([0])		# Expected value of sensor reading
z_act = np.array([0])		# Actual value of sensor reading
r_t = np.array([0])			# Residual of measurements

#### Time Variables ####
t_0 = time.time()	# Time of last propagation update
t_1 = time.time()	# Time of current propagation update

#### Publisher ####
kalman_publisher = rospy.Publisher('kalman_pose', PoseWithCovarianceStamped, queue_size = 1)
kalman_publisher = rospy.Rate(10)

def init():
    rospy.init_node('kalman_propagation')

def kalman_propagation(control_input):
    u_t[0] = control_input.linear.x  # Get forward velocity as action
    t_1 = time.time()
		dt = t_1 - t_0 	# Change in time since last propagation

		# State estimate updated from system dynamics
		x_t1_t = F_t * x_t_t + dt * B_t * u_t

		# Uncertainty estimate grows
		P_t1_t = F_t * P_t_t * F_t.T + G_t * Q_t * G_t.T

		# Update intermediate estimates
		x_t_t = x_t1_t
		P_t_t = P_t1_t

def get_forward_scan(sensor_data):
		# TODO: Figure out how to get scan data from laser scan measurements
		goal_angle = math.pi

		# Find indices of range measurements on either side of the 'goal' angle
    range_idx_0 = math.floor((goal_angle - sensor_data.angle_min) / angle_increment)
		range_idx_1 = range_idx_0 + 1
		# range_idx_1 = math.ceil((goal_angle - sensor_data.angle_min) / angle_increment)

		angle_0 = range_idx_0 * sensor_data.angle_increment + sensor_data.angle_min
		angle_1 = angle_0 + sensor_data.angle_increment

    # Interpolate to estimate range value
		goal_range = (sensor_data.range[range_idx_0] * (angle_1 - goal_angle) + sensor_data.range[range_idx_1] * (goal_angle - angle_0)) / angle_increment
		return goal_range

def kalman_update(sensor_data):
		# Compute expected value of sensor from intermediate state
    z_exp = H_t * x_t1_t

		# Extract forward measurement from sensor data
		z_act = get_forward_scan(sensor_data)

		# Compute residual between actual and expected observations
		r_t = z_act - z_exp

		# Compute covariance of sensor reading
		S_t = H_t * P_t1_t * H_t.T + R_t

		# Compute Kalman Gain
		K_t = P_t1_t * H_t.T * S_t.I

    # Multiply residual times gain to correct state estimate
    x_t1_t1 = x_t1_t + K_t * r_t

    # Uncertainty Estimate Shrinks
    P_t1_t1 = P_t1_t - P_t1_t * H_t.T * S_t.I * H_t * P_t1_t




def spin_subscribers():
    vel_sub = rospy.Subscriber('/cmd_vel', Twist, kalman_propagation)
    # Fix topic name
    scan_sub = rospy.Subscriber('/laser_scan', LaserScan, kalman_update)
		rospy.spin()


#### Main ####

init()
spin_subscribers()

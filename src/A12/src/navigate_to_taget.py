#!/usr/bin/env python


# --- imports ---
import roslib
import rospy
import sys
import cv2
import time
import numpy as np
from collections import deque
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from model_track import Track

target_speed =  550 # Ausgangswert 450 (video)
curve_angle = 25
slow_curve = 0.75
is_shutdown = False
gps_offset = [-0.03, -0.015]

print(" ##### navigate_to_target started ######")

desired_position = np.array([1.96, 2.155])

past_queue_size_angle = 2
past_queue_size_velo = 5
past_angle = deque(maxlen=past_queue_size_angle)
past_angle_velo = deque(maxlen=past_queue_size_velo)

lane_block_queue = []
block_timestamp = time.time()

def get_past_array(a):
	return np.array(list(a))


def get_mean_past(b):
	arr = get_past_array(b)
	return np.mean(arr)


def callback_lanes_blocked(data):
	global lane_block_queue
	global block_timestamp

	if data.data == 111:
		lane_block_queue.append(111)
	# mindestens X mal in 1.5 Sekunden lane blocked signal erhalten
	if len(lane_block_queue) > 3:
		rospy.signal_shutdown(" #### LANE BLOCKED #### ")
	# setze timestamp und lane_block_queue Liste zurueck
	if time.time() - block_timestamp > 1.5:
		block_timestamp = time.time()
		lane_block_queue = []


def callback_position(data):
	global desired_position

	x, y, w, z = data.pose.pose.position.x + gps_offset[0], data.pose.pose.position.y + gps_offset[1], data.pose.pose.orientation.w, data.pose.pose.orientation.z
	# position & angle
	current_position = np.array([x, y])
	orientation_angle = 2 * np.arccos(w) * np.sign(z)

	orientation_vector = np.array([np.cos(orientation_angle), np.sin(orientation_angle)])
	origin_vector = current_position + np.array([0.3 * np.cos(orientation_angle), 0.3 * np.sin(orientation_angle)])

	desired_direction = desired_position - origin_vector

	steering_angle_temp = np.arccos(np.dot(orientation_vector, desired_direction) / (
			np.linalg.norm(orientation_vector) * np.linalg.norm(desired_direction)))

	steering_angle = (steering_angle_temp) / (np.pi) * 180

	orientation_vector = np.array([orientation_vector[0], orientation_vector[1], 0])
	desired_direction = np.array([desired_direction[0], desired_direction[1], 0])

	orientation = np.cross(orientation_vector, desired_direction)[2]
	steering_angle_final = np.clip(steering_angle * np.sign(orientation) * (-2) + 90, 0, 180)

	past_angle.appendleft(steering_angle_final)
	past_angle_velo.appendleft(steering_angle_final)

	pub_steering.publish(get_mean_past(past_angle))

	if not is_shutdown:
		if get_mean_past(past_angle_velo) > 90 + curve_angle or get_mean_past(past_angle_velo) < 90 - curve_angle:
			pub_speed.publish(target_speed * slow_curve)
		else:
			pub_speed.publish(target_speed)


def when_shutdown():
	print("SHUTTING DOWN")
	global is_shutdown
	is_shutdown = True
	pub_speed.publish(0.0)


rospy.on_shutdown(when_shutdown)


def callback_update_destiny(data):
	global desired_position
	desired_position = np.array([data.x, data.y])


rospy.init_node("navigate_to_target", anonymous=True)

pub_steering = rospy.Publisher("steering", UInt8, queue_size=1)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)

# create subscribers and publishers
sub_pos = rospy.Subscriber("/localization/odom/12", Odometry, callback_position, queue_size=1)
sub_des = rospy.Subscriber("/target_point", Point, callback_update_destiny, queue_size=1)
sub_obstacle_on_lane = rospy.Subscriber("/obstacle_on_lane", Int8, callback_lanes_blocked, queue_size=10)


# pub_speed = rospy.Publisher("/speed", UInt8, queue_size=1)


rospy.spin()

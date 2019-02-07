#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from model_track import Track

gps_offset = [-0.03, -0.015]
laneID = 0
distance = 0.35
model = Track(laneID, False)

location = np.array([])
location_corr = np.array([])

obstacle_int = 100

def callback_lane_swap(data):
	global model
	global laneID
	obstacle_int = data.data
	print(obstacle_int)
	if obstacle_int == 100:
		pass
	elif obstacle_int == 110:
		model.set_lane(1)
	elif obstacle_int == 101:
		model.set_lane(0)
	elif obstacle_int == 111:
		print(" ### LANE BLOCKED ### ")


def callback_target(data):
	global location
	global location_corr
	x, y = data.pose.pose.position.x + gps_offset[0], data.pose.pose.position.y + gps_offset[1]

	p_ahead = model.look_ahead((x, y), distance)
	to_pub = Point(p_ahead[0], p_ahead[1], 0)
	# print("ahead:          ",to_pub.x,to_pub.y)
	# print("    ------------------")
	pub_target.publish(to_pub)


def callback_lane_set_to(data):
	model.set_lane(data.data)
	

def callback_lane_switch(data):
	model.switch_lane()


rospy.init_node("localize", anonymous=True)
print(" ##### localize started ######")

# create subscribers and publishers
pub_target = rospy.Publisher("/target_point", Point, queue_size=1)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)

sub_pos = rospy.Subscriber("/localization/odom/12", Odometry, callback_target, queue_size=1)
sub_obstacle_on_lane = rospy.Subscriber("/obstacle_on_lane", Int8, callback_lane_swap, queue_size=10)

rospy.spin()


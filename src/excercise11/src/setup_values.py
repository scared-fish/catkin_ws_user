#!/usr/bin/env python

# Musterloesung von
# Group: ROSINCHEN
# Authors:
#       Felix
#       Adrian
#       Sebastian


class Setup:

	def __init__(self):
		self.lookahead_distance = 0.45  # distance = 0.35
		self.carID = 5
		self.target_speed = 450
		self.curve_angle = 25
		self.slowdown_curve = 0.75
		self.laneID = 1
		self.logging = False
		self.gps_offset = [-0.03, -0.015]
		# rospy.loginfo("initial setup:\n")


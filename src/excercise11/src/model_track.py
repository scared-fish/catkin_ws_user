#!/usr/bin/env python

import numpy as np


class Track:

	def __init__(self, initial_lane, logging=False):
		# Model - values in cm
		self.center_upper = [196, 215.5]
		self.center_lower = [405, 215.5]
		self.lines_inner = [[[196, 79], [405, 79]], [[196, 352], [405, 352]]]
		self.lines_outer = [[[196, 47], [405, 47]], [[196, 384], [405, 384]]]
		# lines = [left line, right line] with left line = [upper point, lower point]
		self.radius_inner = 136.5
		self.radius_outer = 168.5

		self.twoLines = np.array((self.lines_inner, self.lines_outer)) / 100.  # convert to meter
		self.twoRadii = np.array((self.radius_inner, self.radius_outer)) / 100.
		self.twoCenters = np.array((self.center_upper, self.center_lower)) / 100.

		self.current_lane = initial_lane

		self.logging = logging

		self.Lanes = {0: "inner lane", 1: "outer lane"}

	def switch_lane(self):
		curr_lane = self.current_lane
		updated_lane = (curr_lane + 1) % 2
		self.current_lane = updated_lane
		if self.logging:
			print("switched to %s (%d)" % (self.Lanes[updated_lane], updated_lane))

	def set_lane(self, new_lane):
		assert (new_lane in [0, 1]), "choose inner lane with [0] or outer lane with [1]"
		self.current_lane = new_lane
		if self.logging:
			print("set to %s (%d)" % (self.Lanes[new_lane], new_lane))

	def nearest_point(self, given_point):
		curr_lane = self.current_lane
		lines = self.twoLines[curr_lane]
		radius = self.twoRadii[curr_lane]
		centers = self.twoCenters
		x, y = given_point

		if x < centers[0, 0]:  # upper semicircle
			vec_center_given = given_point - centers[0]
			vec_circle = vec_center_given * radius / np.linalg.norm(vec_center_given)
			return 0, np.array(centers[0] + vec_circle)

		elif x > centers[1, 0]:  # lower semicircle
			vec_center_given = given_point - centers[1]
			vec_circle = vec_center_given * radius / np.linalg.norm(vec_center_given)
			return 1, np.array(centers[1] + vec_circle)

		elif y <= centers[0, 1]:  # left line
			return 2, np.array([x, lines[0, 0, 1]])

		elif y > centers[0, 1]:  # right line
			return 3, np.array([x, lines[1, 0, 1]])

		else:
			print("ERROR in choice of track part!")

	def look_ahead(self, given_point, distance, clockwise=False):
		distance_corr = distance + 0.3
		case, near_point = self.nearest_point(given_point)
		centers = self.twoCenters
		if case == 0:  # upper semicircle
			vec_center_given = near_point - centers[0]
			vec_circle = vec_center_given / np.linalg.norm(vec_center_given)
			distance_rotated = np.array([-distance_corr * vec_circle[1], distance_corr * vec_circle[0]])
			# print("distance_corr, rotated",distance_corr,distance_rotated)
			raw_ahead = near_point + distance_rotated
			return self.nearest_point(raw_ahead)[1]
		elif case == 1:  # lower semicircle
			vec_center_given = near_point - centers[1]
			vec_circle = vec_center_given / np.linalg.norm(vec_center_given)
			distance_rotated = np.array([-distance_corr * vec_circle[1], distance_corr * vec_circle[0]])
			# print("distance_corr, rotated",distance_corr,distance_rotated)
			raw_ahead = near_point + distance_rotated
			return self.nearest_point(raw_ahead)[1]
		elif case == 2:  # left line, driving downward, x increasing
			raw_ahead = np.array([near_point[0] + distance_corr, near_point[1]])
			return self.nearest_point(raw_ahead)[1]
		elif case == 3:  # right line, driving upward, x decreasing
			raw_ahead = np.array([near_point[0] - distance_corr, near_point[1]])
			return self.nearest_point(raw_ahead)[1]

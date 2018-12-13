#!/usr/bin/env python2
# code similliar to last exercies and on top of fub_steering_calibration/sripts/angle_calibrator_online.py

# - Programm flow - #

#1.	use RANSAC to discover line in front of car
#2.	calculate slope of the line
#3.	if slope not vertical -> steer
#4.	if no slope detected stop car

# - general imports - #

import sys


import numpy as np
from collections import namedtuple

from scipy import stats

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8, UInt16

from time import localtime, strftime

# - init variables - #

speed_value =150 #speed value
speed = +speed_value # initial direction is backward
steering_angle = 0 

max_y = 2.0 # initial y limitation is 2 meter

inlier_dist = 0.05  # max distance for inliers (in meters) since walls are very flat this can be low
drive_duration_max = 20  # number of seconds to drive

manual_mode = False  # in manual mode we don't actually send commands to the motor

mask_angles = True  # whether to mask any angle that's not on the right side of the car

sample_count = 50  # number RANSAC samples to take

turn_radii = []  # store the detected turn radii in this list
radius_theta = [] # store the detected turn radii in this list vs theta
servo_feedback =[]
wall_angle = 0
target_angle = wall_angle #mask the lidar points
add_pi = np.pi
last_theta = 0
pub_stop_start = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=100, latch=True)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
steering_angle_feedback=0

invert_sign_gamma = False

# - 1. RANSAC - #
# TODO get slope vertical to car
def get_inliers(points, slope, intercept):
	""" return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
	return get_distance(points, slope, intercept) <= inlier_dist


def find_best_params(points):
	""" find the best params to describe a detected line using the RANSAC algorithm """
	best_count = 0
	best_params = (0, 0)

	xs, ys = points.T

	# randomly sample points to define a line and remember the one with the most inliers
	for _ in xrange(sample_count):
		if (len(xs)==0):
			print("warn: The wall couldn't be found!")
			continue
		ind = np.random.randint(0, len(xs), 2)
		x_a, x_b = xs[ind]
		if x_a == x_b:
			continue  # avoid division by 0

		y_a, y_b = ys[ind].astype(np.float64)

		slope = (y_b - y_a) / (x_b - x_a)
		intercept = y_a - x_a * slope

		inlier = get_inliers(points, slope, intercept)
		inl_count = np.sum(inlier)
		if inl_count > best_count:
			best_count = inl_count
			best_params = (slope, intercept)

	# the set of points within inlier distance of the best line we found
	inlier_points = points[np.where(get_inliers(points, *best_params))]

	# perform a linear regression on the selected inlier points
	# slope, intercept, _, _, _ = stats.linregress(*inlier_points.T)
	slope, intercept = best_params

	return slope, intercept


def angle_diff(angle_a, angle_b):
	""" computes the minimum angle distance between two angles in range [-pi, pi) """
	angle_diff=angle_b - angle_a
	return angle_diff


def steering_feedback_callback(steering_angle):
	global steering_angle_feedback
	steering_angle_feedback=int(steering_angle.data)

def get_distance(points, slope, intercept):
	""" return the distance for each point to the parametrised line """
	pos = np.array((0, intercept))  # origin
	dir = np.array((1, slope))  # line gradient
	# element-wise cross product of each points origin offset with the gradient
	c = np.cross(dir, pos - points, axisb=-1)
	return np.abs(c) / np.linalg.norm(dir)



initial_line = None

LineParams = namedtuple('LineParams', ['slope', 'intercept', 'wall_dist', 'wall_angle', 'stamp'])


# - 4. stop driving - #

def stop_driving():
	pub_speed.publish(0)
	rospy.sleep(1)
	rospy.signal_shutdown('stop')
	print('stop driving')
	print ('close the plot to stop the program!')


def scan_callback(scan_msg):
	global initial_line, wall_angle, add_pi, last_theta, speed, speed_value, max_y,target_angle,steering_angle_feedback,invert_sign_gamma

	radius = np.asarray(scan_msg.ranges)
	angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment / 2, scan_msg.angle_increment)

	mask_fin = np.isfinite(radius)  # only consider finite radii

	if mask_angles:
		if (abs(target_angle-wall_angle)<np.pi/4):
			target_angle = wall_angle # right side of the car
		angle_spread = np.pi / 4  # size of cone to consider
		#if Lidar installed inversed!
		#mask_angle = np.logical_or((-np.pi+target_angle + angle_spread) > angles, angles > (np.pi+target_angle - angle_spread))
		#if Lidar installed direct!
		mask_angle = np.logical_or((target_angle + angle_spread) > angles, angles > (target_angle - angle_spread))

		mask = np.logical_and(mask_fin, mask_angle)
	else:
		mask = mask_fin

	masked_angles = angles[mask]
	masked_radius = radius[mask]

	# calculate coordinates of our masked values
	x = np.cos(masked_angles) * masked_radius
	y = np.sin(masked_angles) * masked_radius

	X = np.ones((np.size(x),3))
	X[:,0]=x
	X[:,2]=y

	points = np.column_stack((x, y))

	slope, intercept = find_best_params(points)  # detect a line in these coordinates
	# slope, intercept = np.linalg.lstsq(X[:,0:2],X[:,2], rcond=-1)[0]


	# wall_dist = get_distance((0, 0), slope, intercept)  # shortest distance from current position to the wall
	wall_angle = np.arctan(slope)  # angle of the wall
	if ((wall_angle>0)and (wall_angle<np.pi/2)):
		wall_angle=wall_angle-np.pi/2
	else:
		wall_angle +=np.pi/2
	wall_dist = abs(intercept)/pow(pow(slope,2)+1.0,0.5)  # shortest distance from current position to the wall


	if initial_line is None or scan_msg.header.stamp < initial_line.stamp:
		initial_line = LineParams(slope, intercept, wall_dist, wall_angle, scan_msg.header.stamp)
		del turn_radii[:]

		print 'starting to drive: wall_dist: %.3f, wall_angle: %.1f ' % (wall_dist, np.rad2deg(wall_angle))
		if not manual_mode:
			pub_steering.publish(UInt8(steering_angle))
			rospy.sleep(.2)
			speed=-speed_value
			pub_speed.publish(speed)
	else:
		theta = angle_diff(initial_line.wall_angle, wall_angle)
		dist_diff = initial_line.wall_dist - wall_dist
		abs_theta=abs(theta)
		t=abs(dist_diff)/np.cos((initial_line.wall_angle+wall_angle)/2)
		phi=(np.pi-abs_theta)/2


		time_diff = scan_msg.header.stamp - initial_line.stamp


		if (abs_theta >0.06) and (abs_theta<1.2) and (speed<0):
			if (theta<0):
				invert_sign_gamma=True
			else:
				invert_sign_gamma = False

			last_theta = theta
			turn_radius = t*np.sin(phi)/np.sin(abs_theta)
			time_offset = time_diff.secs + time_diff.nsecs * 1e-9


			# reset turn_radii plot if rosbag loops
			if len(turn_radii) and time_offset < turn_radii[-1][0]:
				del turn_radii[:]

			turn_radii.append([theta, turn_radius])
			servo_feedback.append(steering_angle_feedback)

			print('current wall dist: %.3f, dist diff: %.3f, angle diff: %.1f, current angle: %.1f, start angle: %.1f, turn radius: %.3f' %
			  (wall_dist, dist_diff,np.rad2deg(theta), np.rad2deg(wall_angle), np.rad2deg(initial_line.wall_angle), turn_radius))
		else:
			turn_radius = np.NAN  # dividing by such a small value leads very unexpected results


		# check if we have been driving long enough:
		if not manual_mode and time_diff.secs >= drive_duration_max:
			# pub_speed.publish(0)

			# np.savez_compressed('angle-%03d-%s.txt' % (steering_angle, strftime('%H-%M-%S', localtime())), turn_radii)
			#
			speed=speed_value
			pub_speed.publish(speed)

			if (125>steering_angle and steering_angle>61):
				if (wall_dist<1.2):
					if (len(turn_radii)>0):
						average_r=0
						for r in turn_radii:
							average_r+=r[1]
						average_r=average_r/float(len(turn_radii))
						feedback=np.average(servo_feedback)
						print ('average turn radius: %.3f' % average_r)
						l = 0.26        # 26cm
						gamma = np.arcsin(l/average_r)
						if (invert_sign_gamma==True):
							gamma=-gamma
					else:
						print "turn radius is nan!!"
					stop_driving()
			elif (abs(wall_angle)<0.1):

				if (len(turn_radii)>0):
					average_r=0
					for r in turn_radii:
						average_r+=r[1]
					average_r=average_r/float(len(turn_radii))
					feedback=np.average(servo_feedback)
					print ('average turn radius: %.3f' % average_r)
					l = 0.26        # 26cm
					gamma = np.arcsin(l/average_r)
					if (invert_sign_gamma==True):
						gamma=-gamma
				else:
					print "turn radius is nan!!"
				stop_driving()



def main(args):
	global steering_angle
	rospy.init_node("angle_calibration")
	if len(args) > 1:
		try:
			steering_angle = int(args[1])
			rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
			rospy.Subscriber("/steering_angle", UInt16, steering_feedback_callback, queue_size=1)  # small queue for only reading recent data

		except rospy.ROSInterruptException:
			pass
	else:
		print("please provide a steering setting from [0,180]") 

	#TODO - check?
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

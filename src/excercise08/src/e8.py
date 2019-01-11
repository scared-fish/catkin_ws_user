#!/usr/bin/env python2
import roslib
import sys
import rospy
import cv2
import time
import datetime
import os
import os.path
from std_msgs.msg import String,Int16, UInt8, UInt16
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# - init variables - #

inlier_dist = 0.05  # max distance for inliers (in meters)

sample_count = 50  # number RANSAC samples to take

# all the used publishers and subscribers
pub_stop_start = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=100, latch=True)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
pub_image = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)


bridge = CvBridge()

def get_distance(points, slope, intercept):
	""" return the distance for each point to the parametrised line """
	pos = np.array((0, intercept))  # origin
	dir = np.array((1, slope))  # line gradient
	# element-wise cross product of each points origin offset with the gradient
	c = np.cross(dir, pos - points, axisb=-1)
	return np.abs(c) / np.linalg.norm(dir)


def get_inliers(points, slope, intercept):
	""" return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
	return get_distance(points, slope, intercept) <= inlier_dist

# modifiyed to read in the pixels of the picture and not of the lidar
def find_best_params(points):
	""" find the best params to describe a detected line using the RANSAC algorithm """
	best_count = 0
	best_params = (0, 0)
	rel_points = (320,300,320,400)

	xs, ys = points.T

	# randomly sample points to define a line and remember the one with the most inliers
	for _ in xrange(sample_count):
		if (len(xs)==0):
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
			rel_points = (x_a, y_a, x_b, y_b)

	# the set of points within inlier distance of the best line we found
	inlier_points = points[np.where(get_inliers(points, *best_params))]

	# perform a linear regression on the selected inlier points
	# slope, intercept, _, _, _ = stats.linregress(*inlier_points.T)
	slope, intercept = best_params
	x_a, y_a, x_b, y_b = rel_points

	return x_a, y_a, x_b, y_b


def stop_driving():
	pub_speed.publish(0)
	rospy.sleep(1)
	rospy.signal_shutdown('stop')
	print ('stop driving')
	print ('close the plot to stop the program!')



"""
The PID-controller

We get the wheel turn rate from the tick publisher.
We masured that 180 ticks are equal to 1 meter.


call_pid_controller returns the current actual speed of the car.
This is achived by accumulating the ticks count over one second.
The function recives 'ticks' from the pulse senso. One tick is
emitted when the wheel turns a full round.


pid_velo_controller gets meters per seconds(mps) as input and sets
the speed of the car by publishing the speed as rpm.
At the same time it adjusts the speed so that the actual
mps are equal to the desired mps.
"""


#global ticks so that we can accumulate them over one second
ticks = 0
#global acutal speed of the carr
global_mps = 0

#timestamp renewed each second by call_pid_controller
timestamp_ticks = time.time()



def call_actual_mps(data):

	# collect ticks over one second	
	global timestamp_ticks
	global ticks
	global global_mps

	ticks += data.data

	#each second we update meters per second
	if time.time() - timestamp_ticks >= 1:
		#reset timestamp
		timestamp_ticks = time.time()
		global_mps = ticks

#set desired speed
def pid_velo_controller(mps):

	#global timestamp_velo_cont
	timestamp_velo_cont = time.time()
	delta_time = 1e-15	

	global global_mps
	actual_mps = global_mps

	# K_P, K_I, K_D have to be specified by trail and error
	K_P = 0.12
	K_I = 0.0001
	K_D = 2.5
	
	error_prev = mps - actual_mps
	error_sum = 0	
	while True:
		delta_time = time.time() - timestamp_velo_cont
		#proportional part
		error = mps - actual_mps
		#integral part
		error_sum = error_sum + error * delta_time
		#derivative part
		dedt = (error - error_prev) / delta_time
		# pid sum
		rpm = K_P * error + K_I * error_sum + K_D * dedt
		pub_speed.publish(rpm)
		error_prev = error
		print(rpm)
		print(delta_time)
		time.sleep(1)


def callback(data):
	pub_speed.publish(150)
	#pub_speed.publish(0)
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	# make it gray
	gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

	# bi_gray
	bi_gray_max = 255
	bi_gray_min = 200
	ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
	A= []

	for y in range(240, thresh1.shape[0]):
		for x in range(0, thresh1.shape[1]):
			if thresh1[y,x] > 230:
				A.append([x,y])
	points = np.array(A)

	x_a, y_a, x_b, y_b = find_best_params(points)

#Still the Controller from last exercise.
#It works perfect and we just needed to add the acceleration
#when there is no steering happening

	#check for steering
	if abs(x_a-x_b) > 25 or abs(320-x_a) > 40 or abs(320-x_b) > 40:
		if y_a<y_b: #ensure that y1<y2
			y1= y_a
			y2= y_b
			x1= x_a
			x2= x_b
		else:
			y1= y_b
			y2= y_a
			x1= x_b
			x2= x_a
		if x1-x2 > 20:
			#steer right a bit
			pub_steering.publish(150)
		elif x2-x1 > 20:
			#steer left a bit
			pub_steering.publish(30)
		elif 320-x1 > 20 and 320-x2 > 20:
			#steer left a bit
			pub_steering.publish(30)
		elif 320-x1 < -20 and 320-x2 < -20:
			#steer right a bit
			pub_steering.publish(150)
		rospy.sleep(0.5)
		pub_steering.publish(90)
		rospy.sleep(0.4)

	try:
		pub_image.publish(bridge.cv2_to_imgmsg(thresh1, "mono8"))
	except CvBridgeError as e:
		print(e)

sub_ticks = rospy.Subscriber("/ticks", UInt8, call_actual_mps, queue_size=20)

sub_image = rospy.Subscriber("/camera/color/image_raw",Image, callback, queue_size=1)

def main(args):
	rospy.init_node('advanced_line_drive', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)



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

	timestamp_velo_cont = time.time()
	delta_time = time.time() - timestamp_velo_cont + 1e-15

	global global_mps
	actual_mps = global_mps

	# K_P, K_I, K_D have to be specified by trail and error
	K_P = 0.12
	K_I = 0.0001
	K_D = 150
	
	error_prev = mps - actual_mps
	error_sum = 0	
	while True:
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
		time.sleep(1)

sub_ticks = rospy.Subscriber("/ticks", UInt8, call_actual_mps, queue_size=20)


def main(args):
	rospy.init_node('velocity_controller', anonymous=True)
	try:
		rospy.spin()
		pid_velo_controller(0.25)
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)



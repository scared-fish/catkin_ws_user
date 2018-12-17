#!/usr/bin/env python2
import roslib
import sys
import rospy
import cv2
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

pub_stop_start = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=100, latch=True)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)

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


  def get_inliers(points, slope, intercept):
    """ return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
    return get_distance(points, slope, intercept) <= inlier_dist

  def find_best_params(points, ficki):
	""" find the best params to describe a detected line using the RANSAC algorithm """
	best_count = 0
	best_params = (0, 0)

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
			best_params = (slope, intercept, x_a, y_a, x_b, y_b)

	# the set of points within inlier distance of the best line we found
	inlier_points = points[np.where(get_inliers(points, *best_params))]

	# perform a linear regression on the selected inlier points
	# slope, intercept, _, _, _ = stats.linregress(*inlier_points.T)
	slope, intercept, x_a, y_a, x_b, y_b = best_params

	return x_a, y_a, x_b, y_b

  def stop_driving():
    pub_speed.publish(0)
    rospy.sleep(1)
    rospy.signal_shutdown('stop')
    print ('stop driving')
    print ('close the plot to stop the program!')

  def callback(self,data):
    pub_speed.publish(150)
    pub_speed.publish(0)
    try:
     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
     print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 230
    ret,bnw=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
    A= []
	 
    for y in range(240, bnw.shape[0]):
	for x in range(120, bnw.shape[1]-120):
		if bnw[y,x] > 230:
			A.append([x,y])
    points = np.array(A)
    print(points)
    x_a, y_a, x_b, y_b = self.find_best_params(points)

    if abs(x_a-x_b) > 5 or abs(320-y_a) > 10 or abs(320-y_b) > 10:
	if y_a>y_b: #ensure that y1>y2
	 y1= y_a
	 y2= y_b
	 x1= x_a
	 x2= x_b
        else:
	 y1= y_b
	 y2= y_a
	 x1= x_b
	 x2= x_a
	
	if x1-x2 > 5:
	 #steer right a bit
	 pub_steering.publish(95)
	elif x2-x1 > 5:
	 #steer left a bit
	 pub_steering.publish(85)
	elif 320-y1 > 10 and 320-y2 > 10:
	 #steer left a bit
	 pub_steering.publish(85)
	elif 320-y1 < -10 and 320-y2 < -10:
	 #steer right a bit
	 pub_steering.publish(95)
	sleep(0.5)
	pub_steering.publish(90)
	sleep(0.5)
	pub_speed.publish(0)

    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
        print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)

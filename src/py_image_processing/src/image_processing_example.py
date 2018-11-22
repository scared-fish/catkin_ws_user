#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 230
    ret,bnw,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

	# A4
	# create (y,x) array of all white pixels
	# we use grayscaled image -> shape is (height, widht, 1)
	# the one stands for gray scale 0=black to 255=white
	# since the max grayscale threshold is set to 255 we only have
	# black or white pixels
	A= []
	 
	for y in range(0, bnw.shape[0]):
		for x in range(0, bnw.shape[1]):
			if bnw[y,x] == 255:
				A.append([y,x])
	print A
	# it makes sense to have a cut off at the horizont line.
	# in our test we started from y=240
	# there were spots on the white wall that where brighter than the floor marks



    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)

	# A5
    fx = 614.1699
    fy = 614.9002
    cx = 329.9491
    cy = 237.2788
    camera_mat = np.zeros((3,3,1))
    camera_mat[:,:,0] = np.array([[fx, 0, cx],[0, fy, cy],[0,  0,  1]])
    print camera_mat
    k1 =  0.1115
    k2 = -0.1089
    p1 =  0
    p2 =  0
    dist_coeffs = np.zeros((4,1))
    dist_coeffs[:,0] = np.array([[k1, k2, p1, p2]])
    print dist_coeffs

    # far to close, left to right (order of discovery) in cm
    obj_points = np.zeros((6,3,1))
    obj_points[:,:,0] = np.array([[00.0, 00.0, 0],[40.0, 00.0, 0],[00.0, 38.6, 0],[40.0, 38.6, 0],[00.0, 58, 0],[40.0, 58, 0]])
    image_points = np.zeros((6,3,1))
	# take pixels of left upper corner
    image_points = np.array([[281,239],[281,465],[338,204],[338,505],[456,104],[456,583]], dtype = np.float32)
    print obj_points
    retval, rvec, tvec = cv2.solvePnP(obj_points, image_points, camera_mat, dist_coeffs)
 
    print retval
    print rvec
    print tvec
    rmat = np.zeros((3,3))
    pp = cv2.Rodrigues(rvec, rmat, jacobian=0)
    print pp






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

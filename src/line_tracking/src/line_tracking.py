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
import numpy as np

# from __future__ import print_function

class line_tracker:

  def __init__(self):
    self.image_pub = rospy.Publisher("/line_tracker/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of white color in HSV
    lower_white = np.array([0,0,250])
    upper_white = np.array([50,200,255])

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

    img = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)

    b,g,r = cv2.split(img)

    max1 = (0,0)
    max2 = (img.shape[1],0)
    min1 = (img.shape[1],0)
    min2 = (0,0)

    for y in range(0, img.shape[0]):
     for x in range(0, img.shape[1]):
      if y > img.shape[0]/2:
       if not(r[y,x] == 0 and b[y,x] == 0 and g[y,x] == 0):
        r[y,x] = 255
        g[y,x] = 255
        b[y,x] = 255
        if x < min1[0] and y > img.shape[0]-10:
         min1 = (x,y)
        if x > min2[0] and y > img.shape[0]-10:
         min2 = (x,y)
        if x > max1[0] and x<img.shape[1]/2:
         max1 = (x,y)
        if x < max2[0] and x>img.shape[1]/2:
         max2 = (x,y)
      else:
       r[y,x] = 0
       g[y,x] = 0
       b[y,x] = 0
    img = cv2.merge((b,g,r))

    min1 = (min1[0]+6,min1[1])
    max1 = (max1[0]+1,max1[1])

    line = cv2.line(img, min1, max1, (0,0,255),5);

    min2 = (min2[0]-8,min2[1])
    max2 = (max2[0]+1,max2[1])

    line = cv2.line(img, min2, max2, (0,255,0),5);

    try:
     self.image_pub.publish(self.bridge.cv2_to_imgmsg(line, "bgr8"))
    except CvBridgeError as e:
     print(e)

def main(args):
  rospy.init_node('line_tracker', anonymous=True)
  lc = line_tracker()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

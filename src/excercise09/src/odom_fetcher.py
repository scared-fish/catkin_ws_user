#!/usr/bin/env python2
import roslib
import rospy
import time
import datetime
from nav_msgs.msg import Odometry

#global variables
timestamp = time.time()
position_list = []

def callback(data):

    global timestamp
    global position_list

    if time.time() - timestamp >= 1:
	 #directly transform to pixel values
   	 cur_x = int(round(data.pose.pose.position.x *100)
   	 cur_y = int(round(data.pose.pose.position.y *100)
   	 position_list.append([cur_x,cur_y])data.pose.pose.position.y
   	 timestamp = time.time()
   	 print position_list


sub_odom = rospy.Subscriber("/localization/odom/12",Odometry, callback)


if __name__ == '__main__':
    	rospy.init_node('get_position', anonymous=True)
    	try:
            	rospy.spin()
    	except KeyboardInterrupt:
            	print("Shutting down")


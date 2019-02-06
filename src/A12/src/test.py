#!/usr/bin/env python

import roslib
import rospy
import sys
import cv2
import numpy as np
from collections import deque
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from model_track import Track


def callback_lane_switch(data):
	print(data.data)



rospy.init_node('test', anonymous=True)

sub_obstacle_on_lane = rospy.Subscriber("/obstacle_on_lane", Int8, callback_lane_switch, queue_size=1)

rospy.spin()

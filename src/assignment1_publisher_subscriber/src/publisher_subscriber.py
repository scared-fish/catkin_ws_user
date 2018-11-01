#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32

def callback(request):
	pub=rospy.Publisher("assignment1_publisher_subscriber",String,queue_size=10)
	pub.publish("I heard: " + str(request.data))

rospy.init_node('easy_node')

rospy.Subscriber("yaw",Float32,callback)

rospy.spin()

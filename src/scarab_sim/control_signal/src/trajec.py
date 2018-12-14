#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import rospy
import math
import sys
import os
import cPickle as pickle
import numpy as np

rospy.init_node('register')
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker,queue_size=1)

data_file = open(str(sys.argv[1]),'rb')
# os.system("read -p 'Press Enter to continue...' var")
# print 'Argument List:', 

traject = pickle.load(data_file)
data_file.close()


markerArray = MarkerArray()
rate = rospy.Rate(1) # 1hz

marker = Marker()
marker.header.frame_id = "/desired_trajec"
#marker.type = marker.SPHERE
marker.type = marker.LINE_LIST
marker.action = marker.ADD
marker.scale.x = 0.2
marker.scale.y = 0.2
marker.scale.z = 0.2
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.pose.orientation.w = 1.0
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0

# We add the new marker to the MarkerArray, removing the oldest
# marker from it when necessary
for i in range(90):
    p1 = Point()
    p1.x = traject[i,0]
    p1.y = traject[i,1]
    p1.z = 0
    marker.points.append(p1)


while not rospy.is_shutdown():   
    # Publish the Marker
    publisher.publish(marker)

    rate.sleep()

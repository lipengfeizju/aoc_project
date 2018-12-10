#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np


rospy.init_node('register')
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker,queue_size=1)


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
N = 50
xt = np.zeros([N+1,5])
# xk = np.linspace(0, 10, N+1, endpoint=False)
for i in range(6):
    xt[i,0] = 0
    xt[i,1] = i
    xt[i,2] = 0.
    xt[i,3] = 0.
    xt[i,4] = 0.

# for i in range(6,20):
#     xt[i,0] = i-1
#     xt[i,1] = 6
#     xt[i,2] = 0.
#     xt[i,3] = 0.
#     xt[i,4] = 0.

# for i in range(20,N+1):
#     xt[i,0] = 20
#     xt[i,1] = i/10
#     xt[i,2] = 0.
#     xt[i,3] = 0.
#     xt[i,4] = 0.


for i in range(N+1):
    p1 = Point()
    p1.x = xt[i,0]
    p1.y = xt[i,1]
    p1.z = 0
    marker.points.append(p1)
marker.points.append(p1)
# for i in range(3):
#     p1 = Point()
#     p1.x = 6
#     p1.y = -(i+1.0)/2
#     p1.z = 0
#     marker.points.append(p1)
# for i in range(15):
#     p1 = Point()
#     p1.x = 7+i
#     p1.y = -1.5
#     p1.z = 0
#     marker.points.append(p1)
# for i in range(13):
#     p1 = Point()
#     p1.x = 21
#     p1.y = -i/2.0 - 2
#     p1.z = 0
#     marker.points.append(p1)
# for i in range(18):
#     p1 = Point()
#     p1.x = 22 + i
#     p1.y = -8
#     p1.z = 0
#     marker.points.append(p1)

while not rospy.is_shutdown():   
    # Publish the Marker
    publisher.publish(marker)

    rate.sleep()

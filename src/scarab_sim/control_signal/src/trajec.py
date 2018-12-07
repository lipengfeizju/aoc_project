#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import rospy
import math

rospy.init_node('register')
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker,queue_size=1)


markerArray = MarkerArray()
rate = rospy.Rate(3) # 40hz

marker = Marker()
marker.header.frame_id = "/desired_trajec"
#marker.type = marker.SPHERE
marker.type = marker.LINE_STRIP
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
for i in range(7):
    p1 = Point()
    p1.x = i
    p1.y = 0
    p1.z = 0
    marker.points.append(p1)
for i in range(2):
    p1 = Point()
    p1.x = 6
    p1.y = -(i+1)/2
    p1.z = 0
    marker.points.append(p1)
for i in range(14):
    p1 = Point()
    p1.x = 7+i
    p1.y = -1.5
    p1.z = 0
    marker.points.append(p1)

while not rospy.is_shutdown():   
    # Publish the Marker
    publisher.publish(marker)

    rate.sleep()

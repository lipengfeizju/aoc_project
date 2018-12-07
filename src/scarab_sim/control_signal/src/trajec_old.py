#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import rospy
import math

rospy.init_node('register')
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker,queue_size=10)
publisher1 = rospy.Publisher('visualization_p', Point,queue_size=10)

markerArray = MarkerArray()
rate = rospy.Rate(40) # 40hz
count = 0
MARKERS_MAX = 100
marker = Marker()
marker.header.frame_id = "/neck"

while not rospy.is_shutdown():

    
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
    #    marker.pose.position.x = math.cos(count / 50.0)
    #    marker.pose.position.y = math.cos(count / 40.0) 
    #    marker.pose.position.z = math.cos(count / 30.0) 
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    # We add the new marker to the MarkerArray, removing the oldest
    # marker from it when necessary
    p1 = Point()
    p1.x = math.cos(count / 50.0)
    p1.y = math.cos(count / 40.0) 
    p1.z = math.cos(count / 30.0) 
    marker.points.append(p1)
    if(count > MARKERS_MAX):
        marker.points.pop(0)

    # Publish the Marker
    publisher.publish(marker)
    publisher1.publish(p1)
    count += 1

    rate.sleep()

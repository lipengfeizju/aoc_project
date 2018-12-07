#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from control_signal.msg import Control

vel_msg = Twist()
velocity_publisher = rospy.Publisher('/scarab40/cmd_vel_mux/input/navi', Twist, queue_size=10)
# state vector
delta = 0
v = 0
vel_msg.linear.x = v # velocity
vel_msg.angular.z = v*math.tan(delta) # theta_dot
# constant values 
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
topic_hz = 10

def convert_con(con_in):
    global vel_msg,v,delta,velocity_publisher
    v = con_in.u_a/topic_hz + v
    delta = con_in.u_delta/topic_hz + delta
    vel_msg.linear.x = v
    vel_msg.angular.z = v*math.tan(delta)
    velocity_publisher.publish(vel_msg)
    

def control():
    # Starts a new node
    rospy.init_node('dymanic_con', anonymous=True)
    velocity_publisher.publish(vel_msg)
    rospy.Subscriber('/scarab40/con_in', Control, convert_con)
    # Initial movement.
    velocity_publisher.publish(vel_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass

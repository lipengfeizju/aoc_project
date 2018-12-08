#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from control_signal.msg import Control
from control_signal.msg import State

vel_msg = Twist()
velocity_publisher = rospy.Publisher('cmd_output', Twist, queue_size=10)
state_publisher = rospy.Publisher('State', State, queue_size=10)

# state vector
S1 = State()
S1.delta = 0
S1.v = 0
vel_msg.linear.x = S1.v # velocity
vel_msg.angular.z = S1.v*math.tan(S1.delta) # theta_dot
# constant values 
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
topic_hz = 20

def convert_con(con_in):
    global vel_msg,S1,velocity_publisher
    S1.v = con_in.u_a/topic_hz + S1.v
    S1.delta = con_in.u_delta/topic_hz + S1.delta
    vel_msg.linear.x = S1.v
    vel_msg.angular.z = S1.v*math.tan(S1.delta)
    velocity_publisher.publish(vel_msg)
    state_publisher.publish(S1)
    

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

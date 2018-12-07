#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf2_ros

def move():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    rate = rospy.Rate(5) # 30hz
    velocity_publisher = rospy.Publisher('/scarab40/cmd_vel_mux/input/navi', Twist, queue_size=10)
    vel_msg = Twist()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    vel_msg.linear.x = 0
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        current_distance = 0
        try:
            trans = tfBuffer.lookup_transform('map','scarab40/base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        else:
            #Loop to move the turtle in an specified distance
            while(t1-t0 < 1):                                    
                #Publish the velocity
                if trans.transform.translation.x -5 > 0:
                    vel_msg.linear.x = 0.5 - abs(t1-t0-0.5)
                else:
                    vel_msg.linear.x = abs(t1-t0-0.5) - 0.5
                if trans.transform.translation.y + 1 < 0:
                    vel_msg.angular.z = 0.5 - abs(t1-t0-0.5)
                else:
                    vel_msg.angular.z = abs(t1-t0-0.5) - 0.5
                velocity_publisher.publish(vel_msg)
               #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                rate.sleep()
            rospy.loginfo("x:%f\n",trans.transform.translation.x)
            rospy.loginfo("y:%f\n",trans.transform.translation.y)
            rospy.loginfo("z:%f\n",trans.transform.translation.z)
            #After the loop, stops the robot
            # vel_msg.linear.x = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

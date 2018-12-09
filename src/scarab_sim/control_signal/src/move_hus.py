#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf2_ros

def move():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    rate = rospy.Rate(10) # 30hz
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
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
        #Loop to move the turtle in an specified distance

        t0 = rospy.Time.now().to_sec()
        t1 = t0
        i = 0
        while (t1-t0 < 20): 
            #rospy.loginfo("t:%f\n",t1 - t0)
            if t1 - t0 < 3:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            else:
                if i < 30:
                    vel_msg.linear.x = i/20
                    vel_msg.angular.z = 0.00*(i/10)
                elif i < 60:   
                    vel_msg.linear.x = (60 - i)/20
                    vel_msg.angular.z = 0.00*(i)
                else:
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                i = i + 1
            t1=rospy.Time.now().to_sec()
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            rate.sleep()
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

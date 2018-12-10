#!/usr/bin/env python
import rospy
from control_signal.msg import Control
import cPickle as pickle
import numpy as np
import os
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Header

data_file = open('/home/lipengfei/workspace/aoc_project/src/scarab_sim/control_signal/data_debug/data_us.p','rb')
con_sig = pickle.load(data_file)
data_file.close()
data_file = open('/home/lipengfei/workspace/aoc_project/src/scarab_sim/control_signal/data_debug/data_xs.p','rb')
state_sig = pickle.load(data_file)
data_file.close()

def con_input():
    # Starts a new node
    rospy.init_node('con_in_node', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    con_in_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=10)
    #Setting the current time for distance calculus
    # let's build a 3x3 matrix:
    drive_in = AckermannDriveStamped()
    drive_in.header = Header()
    drive_in.header.frame_id = 'base_link'
    drive_in.header.stamp = rospy.Time.now()
    drive_in.header.seq = 0

    drive_in.drive = AckermannDrive()
    drive_in.drive.steering_angle = 0
    drive_in.drive.steering_angle_velocity = 0
    drive_in.drive.speed = 0
    drive_in.drive.acceleration = 0
    drive_in.drive.jerk = 0
    os.system("read -p 'Press Enter to continue...' var")
    #print('Any key to start')
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        i = 0
        while (t1-t0 < 20): 
            #rospy.loginfo("t:%f\n",t1 - t0)
            con_in_pub.publish(drive_in)
            if t1 - t0 < 3:
                drive_in.drive.steering_angle = 0
                drive_in.drive.steering_angle_velocity = 0
                drive_in.drive.speed = 0
                drive_in.drive.acceleration = 0
                drive_in.header.seq = i
            else:
                if i < 50:                   
                    drive_in.drive.speed = 2*state_sig[i+1,3]
                    drive_in.drive.acceleration = 2*con_sig[i,0]
                    drive_in.drive.steering_angle = state_sig[i+1,4]
                    drive_in.drive.steering_angle_velocity = con_sig[i,1]
                    i = i + 1             
                else:
                    drive_in.drive.steering_angle = 0
                    drive_in.drive.steering_angle_velocity = 0
                    drive_in.drive.speed = 0
                    drive_in.drive.acceleration = 0
                    drive_in.header.seq = i
            t1=rospy.Time.now().to_sec()
            rate.sleep()
        rate.sleep()
            
if __name__ == '__main__':
    try:
        con_input()
    except rospy.ROSInterruptException: pass

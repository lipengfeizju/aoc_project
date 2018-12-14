#!/usr/bin/env python
import rospy
from control_signal.msg import Control
import cPickle as pickle
import numpy as np
import os
import sys

#con_file = open('/home/lipengfei/workspace/aoc_project/src/scarab_sim/control_signal/data_debug/data.p')
con_file = open(str(sys.argv[1]),'rb')
con_sig = pickle.load(con_file)

def con_input():
    # Starts a new node
    rospy.init_node('con_in_node', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    con_in_pub = rospy.Publisher('con_in', Control, queue_size=10)
    #Setting the current time for distance calculus
    # let's build a 3x3 matrix:
    con_in = Control()
    con_in.u_a = 0
    con_in.u_delta = 0
    os.system("read -p 'Press Enter to continue...' var")
    #print('Any key to start')
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        i = 0
        while (t1-t0 < 20): 
            #rospy.loginfo("t:%f\n",t1 - t0)
            con_in_pub.publish(con_in)
            if t1 - t0 < 3:
                con_in.u_a = 0
                con_in.u_delta = 0 
            else:
                if i < 50:
                    con_in.u_a = con_sig[i][0]*5
                    con_in.u_delta = con_sig[i][1]*5
                    i = i + 1             
                else:
                    con_in.u_a = 0
                    con_in.u_delta = 0
            t1=rospy.Time.now().to_sec()
            rate.sleep()
        rate.sleep()
            
if __name__ == '__main__':
    try:
        con_input()
    except rospy.ROSInterruptException: pass

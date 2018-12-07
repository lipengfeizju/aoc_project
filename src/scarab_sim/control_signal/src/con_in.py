#!/usr/bin/env python
import rospy
from control_signal.msg import Control

def con_input():
    # Starts a new node
    rospy.init_node('con_in_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    con_in_pub = rospy.Publisher('/scarab40/con_in', Control, queue_size=10)
    #Setting the current time for distance calculus
    # let's build a 3x3 matrix:
    con_in = Control()
    con_in.u_a = 0
    con_in.u_delta = 0

   
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        while (t1-t0 < 20): 
            if  t1-t0 > 3:
                #rospy.loginfo("t:%f\n",t1 - t0)
                con_in_pub.publish(con_in)
                if t1 - t0 < 4:
                    con_in.u_a = 1
                    con_in.u_delta = -0.5                    
                elif t1 - t0 < 5:
                    con_in.u_a = -1
                    con_in.u_delta = 0.5 
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

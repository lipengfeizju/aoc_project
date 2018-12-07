#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

def con_input():
    # Starts a new node
    rospy.init_node('con_in_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    con_in_pub = rospy.Publisher('/scarab40/con_in', Float32MultiArray, queue_size=10)
    #Setting the current time for distance calculus
    # let's build a 3x3 matrix:
    control_mat = Float32MultiArray()
    control_mat.layout.dim.append(MultiArrayDimension())
    control_mat.layout.dim.append(MultiArrayDimension())
    control_mat.layout.dim[0].label = "height"
    control_mat.layout.dim[1].label = "width"
    control_mat.layout.dim[0].size = 2
    control_mat.layout.dim[1].size = 1
    control_mat.layout.dim[0].stride = 2
    control_mat.layout.dim[1].stride = 1
    control_mat.layout.data_offset = 0
    control_mat.data = [0.0]*2
   
    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        while (t1-t0 < 9):  
            #rospy.loginfo("t:%f\n",t1 - t0)
            con_in_pub.publish(control_mat)
            if t1 - t0 < 1:
                control_mat.data[0] = 1
                control_mat.data[1] = 0.1
                t1=rospy.Time.now().to_sec()
            elif t1 - t0 < 2:
                control_mat.data[0] = -1
                control_mat.data[1] = -0.1
                t1=rospy.Time.now().to_sec()
            else:
                control_mat.data[0] = 0
                t1=rospy.Time.now().to_sec()
            rate.sleep()
            
if __name__ == '__main__':
    try:
        con_input()
    except rospy.ROSInterruptException: pass

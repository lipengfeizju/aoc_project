#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include<iostream>  
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  //std::cout <<typeid(msg->transforms[1].transform.translation.x).name()<<"\n";
      int x = (msg->transforms).size();
      std::string str1 ("scarab03/base_footprint");

      
      if (str1.compare(msg->transforms[0].child_frame_id)){
        ROS_INFO("The result should be: [%d]",0);
      }
      else{
        ROS_INFO("The result should be: [%d]",1);
        std::cout <<"frame id should be "<<(msg->transforms[0].header.frame_id)<<"\n";
        std::cout <<"child frame should be"<<msg->transforms[0].child_frame_id<<"\n";
        ROS_INFO("The position is : [x :%f y :%f yaw :%f ]", msg->transforms[0].transform.translation.x,msg->transforms[0].transform.translation.y,msg->transforms[0].transform.rotation.z);
      }
      //std::cout <<typeid(msg->transforms[0].header.frame_id).name()<<"\n";
      //ROS_INFO("The frame should be: [%s]", msg->transforms[0].child_frame_id);
      }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("tf", 1000, chatterCallback);
  ros::spin();

  return 0;
}
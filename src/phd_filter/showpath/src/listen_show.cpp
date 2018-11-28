#include "ros/ros.h"
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "tf2_msgs/TFMessage.h"
#include<iostream>  

#include <sstream>
geometry_msgs::PoseStamped this_pose_stamped;




void chatterCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    std::string str1 ("scarab03/base_footprint");      
      if (str1.compare(msg->transforms[0].child_frame_id)){
        //ROS_INFO("The result should be: [%d]",0);
      }
      else{
        //ROS_INFO("The result should be: [%d]",1);
        //std::cout <<"frame id should be "<<(msg->transforms[0].header.frame_id)<<"\n";
        //std::cout <<"child frame should be"<<msg->transforms[0].child_frame_id<<"\n";
        //ROS_INFO("The position is : [x :%f y :%f yaw :%f ]", msg->transforms[0].transform.translation.x,msg->transforms[0].transform.translation.y,msg->transforms[0].transform.rotation.z);
        this_pose_stamped.pose.position.x = msg->transforms[0].transform.translation.x;
        this_pose_stamped.pose.position.y = msg->transforms[0].transform.translation.y;
        this_pose_stamped.pose.position.z = msg->transforms[0].transform.translation.z;

        this_pose_stamped.pose.orientation.x = msg->transforms[0].transform.rotation.x;
        this_pose_stamped.pose.orientation.y = msg->transforms[0].transform.rotation.x;
        this_pose_stamped.pose.orientation.z = msg->transforms[0].transform.rotation.z;
        this_pose_stamped.pose.orientation.w = msg->transforms[0].transform.rotation.w;
      }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");   
    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",10, true);
    ros::Subscriber sub = ph.subscribe("tf", 10, chatterCallback);
 

    ros::Time current_time;
    current_time = ros::Time::now();

    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="map";


   int count = 0;
   ros::Rate loop_rate(50); 
  while (ros::ok())
  {

    current_time = ros::Time::now();

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    
    path_pub.publish(path);
    ros::spinOnce();

    loop_rate.sleep();
  }


  ros::spin();

  return 0;
}
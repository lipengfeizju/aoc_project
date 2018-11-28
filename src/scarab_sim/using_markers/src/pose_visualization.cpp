// This code is used to generate axis of robot by using PoseStamped
// which can be visualized in RVIZ directly

// author: sikang liu

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nodelet/nodelet.h>

class PoseVisualization : public nodelet::Nodelet {
public:
  void onInit();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  ros::Publisher pose_pub;
  ros::Subscriber odom_sub;
};

void PoseVisualization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  geometry_msgs::PoseStamped p;
  p.header = msg->header;
  p.pose.position.x = msg->pose.pose.position.x;
  p.pose.position.y = msg->pose.pose.position.y;
  p.pose.position.z = msg->pose.pose.position.z;
  p.pose.orientation.x = msg->pose.pose.orientation.x;
  p.pose.orientation.y = msg->pose.pose.orientation.y;
  p.pose.orientation.z = msg->pose.pose.orientation.z;
  p.pose.orientation.w = msg->pose.pose.orientation.w;
  pose_pub.publish(p);
}

void PoseVisualization::onInit() {
  ros::NodeHandle n(getPrivateNodeHandle());
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10, true);
  odom_sub = n.subscribe("odom", 10, &PoseVisualization::odomCallback, this);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(using_markers, PoseVisualization, PoseVisualization,
                        nodelet::Nodelet);

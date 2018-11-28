// This code is used to generate axis of robot by using PoseStamped
// which can be visualized in RVIZ directly

// author: sikang liu

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <nodelet/nodelet.h>

class CmdVisualization : public nodelet::Nodelet {
public:
  void onInit();

private:
  void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  ros::Publisher pose_pub;
  ros::Subscriber cmd_sub;
};

void CmdVisualization::cmdCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
  geometry_msgs::PoseStamped p;
  p.header = msg->header;
  p.pose.position.x = msg->position.x;
  p.pose.position.y = msg->position.y;
  p.pose.position.z = msg->position.z;

  p.pose.orientation.w = cos(msg->yaw / 2);
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = sin(msg->yaw / 2);
  pose_pub.publish(p);
}

void CmdVisualization::onInit() {
  ros::NodeHandle n(getPrivateNodeHandle());
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10, true);
  cmd_sub =
      n.subscribe("position_cmd", 10, &CmdVisualization::cmdCallback, this);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(using_markers, CmdVisualization, CmdVisualization,
                        nodelet::Nodelet);

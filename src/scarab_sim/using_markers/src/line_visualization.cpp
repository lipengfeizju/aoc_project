// This code is used for creating lines with vetices prepresenting subscribed
// odometry of robot

// author: sikang liu
// last modify: 12/09/2013

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nodelet/nodelet.h>

class LineVisualization : public nodelet::Nodelet {
public:
  void onInit();

private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void
  pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void initParams();
  ros::Publisher line_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber pose_sub;
  visualization_msgs::Marker points, line;
  float point_color_r, point_color_g, point_color_b, point_color_a;
  float line_color_r, line_color_g, line_color_b, line_color_a;
  float point_scale_x, point_scale_y, line_scale;
};

void LineVisualization::odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  points.header = line.header = msg->header;
  points.header.frame_id = line.header.frame_id = std::string("map");
  geometry_msgs::Point p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  points.points.push_back(p);
  line.points.push_back(p);
  line_pub.publish(points);
  line_pub.publish(line);
}

void LineVisualization::pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  points.header = line.header = msg->header;
  points.header.frame_id = line.header.frame_id = std::string("map");
  geometry_msgs::Point p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  points.points.push_back(p);
  line.points.push_back(p);
  line_pub.publish(points);
  line_pub.publish(line);
}

void LineVisualization::initParams() {
  points.action = line.action = visualization_msgs::Marker::ADD;
  points.ns = line.ns = "line";

  points.pose.orientation.w = line.pose.orientation.w = 1.0;

  points.id = 0;
  line.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = point_scale_x;
  points.scale.y = point_scale_y;

  points.color.r = point_color_r;
  points.color.g = point_color_g;
  points.color.b = point_color_b;
  points.color.a = point_color_a;

  line.scale.x = line_scale;
  line.color.r = line_color_r;
  line.color.g = line_color_g;
  line.color.b = line_color_b;
  line.color.a = line_color_a;
}

void LineVisualization::onInit() {
  ros::NodeHandle n(getPrivateNodeHandle());
  line_pub = n.advertise<visualization_msgs::Marker>("lines", 10, true);
  odom_sub = n.subscribe("odom", 10, &LineVisualization::odom_callback, this);
  pose_sub = n.subscribe("pose", 10, &LineVisualization::pose_callback, this);
  // ros::Subscriber pose_sub = n.subscribe("pose", 10, pose_callback);
  n.param("point_color_r", point_color_r, 0.0f);
  n.param("point_color_g", point_color_g, 1.0f);
  n.param("point_color_b", point_color_b, 0.0f);
  n.param("point_color_a", point_color_a, 1.0f);
  n.param("point_scale_x", point_scale_x, 0.02f);
  n.param("point_scale_y", point_scale_y, 0.02f);
  n.param("line_color_r", line_color_r, 1.0f);
  n.param("line_color_g", line_color_g, 0.0f);
  n.param("line_color_b", line_color_b, 0.0f);
  n.param("line_color_a", line_color_a, 1.0f);
  n.param("line_scale", line_scale, 0.02f);
  initParams();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(using_markers, LineVisualization, LineVisualization,
                        nodelet::Nodelet);

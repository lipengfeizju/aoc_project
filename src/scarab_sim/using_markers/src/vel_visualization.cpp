#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <nodelet/nodelet.h>

class VelVisualization : public nodelet::Nodelet {
public:
  void onInit();

private:
  Eigen::Quaterniond VecToQuaternion(Eigen::Vector3d v);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  ros::Publisher v_pub;
  ros::Subscriber odom_sub;
  float color_r, color_g, color_b, color_a, scale_x, scale_y, scale_z;
};

Eigen::Quaterniond VelVisualization::VecToQuaternion(Eigen::Vector3d v) {
  Eigen::Quaterniond q;
  float pitch, yaw;
  pitch = atan2(-v[2], sqrt(v[0] * v[0] + v[1] * v[1]));
  yaw = atan2(v[1], v[0]);
  q.w() = cos(pitch / 2) * cos(yaw / 2);
  q.x() = -sin(pitch / 2) * sin(yaw / 2);
  q.y() = sin(pitch / 2) * cos(yaw / 2);
  q.z() = cos(pitch / 2) * sin(yaw / 2);
  return q;
}

void VelVisualization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  visualization_msgs::Marker m;
  Eigen::Vector3d v;
  v[0] = msg->twist.twist.linear.x;
  v[1] = msg->twist.twist.linear.y;
  v[2] = msg->twist.twist.linear.z;
  float d = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  Eigen::Quaterniond q = VecToQuaternion(v);
  m.header.frame_id = msg->header.frame_id;
  m.header.stamp = ros::Time(); // time 0 so that the marker will be displayed
                                // regardless of the current time
  m.ns = "arrow";               //"mesh_visualization";
  m.id = 0;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = msg->pose.pose.position.x;
  m.pose.position.y = msg->pose.pose.position.y;
  m.pose.position.z = msg->pose.pose.position.z;
  m.pose.orientation.x = q.x();
  m.pose.orientation.y = q.y();
  m.pose.orientation.z = q.z();
  m.pose.orientation.w = q.w();
  m.scale.x = d;
  m.scale.y = scale_y;
  m.scale.z = scale_z;
  m.color.a = color_a;
  m.color.r = color_r;
  m.color.g = color_g;
  m.color.b = color_b;
  v_pub.publish(m);
}

void VelVisualization::onInit() {
  ros::NodeHandle n(getPrivateNodeHandle());
  n.param("color/r", color_r, 1.0f);
  n.param("color/g", color_g, 0.0f);
  n.param("color/b", color_b, 0.0f);
  n.param("color/a", color_a, 1.0f);
  n.param("scale/x", scale_x, 1.0f);
  n.param("scale/y", scale_y, 1.0f);
  n.param("scale/z", scale_z, 1.0f);
  v_pub = n.advertise<visualization_msgs::Marker>("vel_arrow", 10, true);
  odom_sub = n.subscribe("odom", 10, &VelVisualization::odomCallback, this);
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(using_markers, VelVisualization, VelVisualization,
                        nodelet::Nodelet);

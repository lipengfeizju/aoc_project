#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

class CovarVisualization : public nodelet::Nodelet {
public:
  void onInit();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  ros::Publisher covar_pub;
  ros::Subscriber odom_sub;
  float color_r, color_g, color_b, color_a, scale_x, scale_y, scale_z;
};

void CovarVisualization::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  Eigen::Matrix3d noise_mat;
  for (unsigned int i = 0; i < 2; i++)
    for (unsigned int j = 0; j < 2; j++)
      noise_mat(i, j) = msg->pose.covariance[i + j * 6];
  noise_mat(2, 2) = msg->pose.covariance[3 + 3 * 6];
  noise_mat(1, 2) = msg->pose.covariance[1 + 3 * 6];
  noise_mat(2, 1) = msg->pose.covariance[3 + 1 * 6];
  noise_mat(2, 0) = msg->pose.covariance[3 + 0 * 6];
  noise_mat(0, 2) = msg->pose.covariance[0 + 3 * 6];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(noise_mat);

  if (es.info() == Eigen::Success) {
    Eigen::Quaterniond q(es.eigenvectors().determinant() * es.eigenvectors());
    visualization_msgs::Marker covar;
    covar.header = msg->header;
    covar.ns = "covar_visualization"; //"mesh_visualization";
    covar.id = 2;
    covar.type = visualization_msgs::Marker::SPHERE;
    covar.action = visualization_msgs::Marker::ADD;
    covar.pose.position.x = msg->pose.pose.position.x;
    covar.pose.position.y = msg->pose.pose.position.y;
    covar.pose.position.z = msg->pose.pose.position.z;
    covar.pose.orientation.w = q.w();
    covar.pose.orientation.x = q.x();
    covar.pose.orientation.y = q.y();
    covar.pose.orientation.z = q.z();
    covar.scale.x = es.eigenvalues()[0] * scale_x;
    covar.scale.y = es.eigenvalues()[1] * scale_y;
    covar.scale.z = es.eigenvalues()[2] * scale_z;
    covar.color.a = color_a;
    covar.color.r = color_r;
    covar.color.g = color_g;
    covar.color.b = color_b;
    covar_pub.publish(covar);
  }
}

void CovarVisualization::onInit() {
  ros::NodeHandle n(getPrivateNodeHandle());
  n.param("color/r", color_r, 1.0f);
  n.param("color/g", color_g, 0.0f);
  n.param("color/b", color_b, 0.0f);
  n.param("color/a", color_a, 0.5f);
  n.param("scale/x", scale_x, 10000.0f);
  n.param("scale/y", scale_y, 10000.0f);
  n.param("scale/z", scale_z, 10000.0f);
  covar_pub = n.advertise<visualization_msgs::Marker>("covariance", 10, true);
  odom_sub = n.subscribe("odom", 10, &CovarVisualization::odomCallback, this);
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(using_markers, CovarVisualization, CovarVisualization,
                        nodelet::Nodelet);

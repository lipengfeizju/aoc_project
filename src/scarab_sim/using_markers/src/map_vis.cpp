#include <ros/ros.h>
#include <data_ros_utils.h>
#include <map_utils/voxel_map_util.h>
#include "space_visualization.hpp"

ros::Publisher occ_map_pub;
ros::Publisher free_map_pub;
ros::Publisher space_pub;

std::unique_ptr<VoxelMapUtil> map_util_;
SpaceVisualization space_util_;

void visualizeMap(const mav_high_level_msgs::VoxelMap::ConstPtr &msg) {
  map_util_->setMap(*msg);

  sensor_msgs::PointCloud free_cloud = vec_to_cloud(map_util_->getFreeCloud());
  free_cloud.header = msg->header;
  free_map_pub.publish(free_cloud);

  sensor_msgs::PointCloud occ_cloud = vec_to_cloud(map_util_->getCloud());
  occ_cloud.header = msg->header;
  occ_map_pub.publish(occ_cloud);

  sensor_msgs::PointCloud corners = vec_to_cloud(map_util_->getSpace());
  corners.header = msg->header;
  space_util_.generate_space(corners);

  space_pub.publish(space_util_.get_space());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_vis");
  ros::NodeHandle nh("~");

  ros::Subscriber map_sub = nh.subscribe("voxel_map", 1, visualizeMap);

  free_map_pub = nh.advertise<sensor_msgs::PointCloud>("free_map", 1, true);
  occ_map_pub = nh.advertise<sensor_msgs::PointCloud>("occ_map", 1, true);
  space_pub = nh.advertise<visualization_msgs::Marker>("space", 1, true);

  map_util_.reset(new VoxelMapUtil());

  ros::spin();

  return 0;
}

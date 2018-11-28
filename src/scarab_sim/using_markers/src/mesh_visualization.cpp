#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mesh_visualization");

  ros::NodeHandle nh("~");

  std::string mesh_resource;
  nh.param("mesh_resource", mesh_resource,
           std::string("package://nx_models/models/fla_warehouse1/fla_warehouse1.dae"));
  std::string world_frame;
  nh.param("world_frame", world_frame, std::string("map"));  
  ros::Publisher pub_vis =
      nh.advertise<visualization_msgs::Marker>("env", 1, true);

  visualization_msgs::Marker marker;
  marker.mesh_use_embedded_materials = true;
  marker.header.frame_id = world_frame;
  marker.header.stamp = ros::Time(); // time 0 so that the marker will be
                                     // displayed regardless of the current time
  marker.ns = "mesh_visualization";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.mesh_resource = mesh_resource;
  pub_vis.publish(marker);

  ros::spin();

  return 0;
}

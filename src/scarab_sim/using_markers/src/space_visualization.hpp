// Class of Space Visualization
// merged into map_vis

// author: sikang liu
// last modify: 06/28/2016

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>

class SpaceVisualization {
public:
  SpaceVisualization() {
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.ns = "edges";
    line_list.pose.orientation.w = 1.0;
    line_list.id = 1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    float line_list_color[4], line_list_scale;
    line_list_color[0] = 1.0;
    line_list_color[1] = 1.0;
    line_list_color[2] = 0.0;
    line_list_color[3] = 1.0;
    line_list_scale = 0.02;

    line_list.color.r = line_list_color[0];
    line_list.color.g = line_list_color[1];
    line_list.color.b = line_list_color[2];
    line_list.color.a = line_list_color[3];
    line_list.scale.x = line_list_scale;
    line_list.scale.y = line_list_scale;
  }

  void generate_space(const sensor_msgs::PointCloud &cloud) {
    if (cloud.points.size() != 8) {
      ROS_ERROR("SpaceVisualization: input points number is not correct!");
      return;
    }
    line_list.points.clear();
    line_list.header = cloud.header;
    std::vector<geometry_msgs::Point32> p_list;
    for (size_t i = 0; i < cloud.points.size(); i++)
      p_list.push_back(cloud.points[i]);
    add_line(p_list[0], p_list[1]);
    add_line(p_list[1], p_list[2]);
    add_line(p_list[2], p_list[3]);
    add_line(p_list[3], p_list[4]);
    add_line(p_list[4], p_list[5]);
    add_line(p_list[5], p_list[6]);
    add_line(p_list[6], p_list[7]);
    add_line(p_list[0], p_list[7]);
    add_line(p_list[6], p_list[1]);
    add_line(p_list[0], p_list[3]);
    add_line(p_list[2], p_list[5]);
    add_line(p_list[4], p_list[7]);
  }

  void add_line(const geometry_msgs::Point32 &p1,
                const geometry_msgs::Point32 &p2) {
    geometry_msgs::Point p;
    p.x = p1.x;
    p.y = p1.y;
    p.z = p1.z;
    line_list.points.push_back(p);
    p.x = p2.x;
    p.y = p2.y;
    p.z = p2.z;
    line_list.points.push_back(p);
  }

  visualization_msgs::Marker get_space() { return line_list; }

private:
  visualization_msgs::Marker line_list;
};

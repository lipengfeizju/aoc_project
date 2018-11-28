#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_image_once_node");
  if (argc < 2)
  {
    ROS_INFO("usage: pub_image_once_node path_to_img [optional_topic]");
    return 1;
  }

  std::string topic;
  if (argc == 3) topic = std::string(argv[2]);
  else topic = "image";
  
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic, 10);

  ros::Duration(1).sleep();
  
  // Load the image
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::Image img_msg = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg()); //TODO: Is this efficient??

  ros::Rate rate(1);

  while (ros::ok()) {
  
    pub.publish(img_msg);

    ros::spinOnce();

    rate.sleep();

  }

  return 0;
}


#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include "ffld_ros/SetDpmParams.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "FFLDDetector.hpp"
#include "ffld_ros/ObjectDetection.h"
#include "dpm_detect_client.cpp"

dpm_detect_client::Params pr_;

void imgCallback(const sensor_msgs::ImageConstPtr msg,
                 dpm_detect_client::DpmDetectClient dpm_client,
                 image_transport::Publisher image_pub) {
  ffld_ros::DetectObject srv;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  ffld_ros::DetectObject detections =
      dpm_client.requestDetection(*(cv_ptr->toImageMsg()));

  cv::Mat detection_image = dpm_client.drawDetection(cv_ptr->image, detections);

  image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                       detection_image).toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "detect_object_client");

  ros::NodeHandle nh;

  nh.param("model_num", pr_.model_num, 0);
  nh.param("overlap", pr_.overlap, 0.5);
  nh.param("threshold", pr_.threshold, -0.5);
  nh.param("padding", pr_.padding, 6);
  nh.param("interval", pr_.interval, 5);
  nh.param("resize_val", pr_.resize_val, 0.25);
  nh.param("rate", pr_.rate, 1.0);

  dpm_detect_client::DpmDetectClient client(nh);

  client.setDpmParams(pr_);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/detections", 1);

  std::string cam_topic = "image";

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(
      cam_topic, 1, boost::bind(imgCallback, _1, client, image_pub));

  // Load the image
  /*cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(image, image, CV_BGR2GRAY);*/
  // srv.request.img = *(cv_bridge::CvImage(std_msgs::Header(), "mono8",
  // image).toImageMsg()); //TODO: Is this efficient??
  /*srv.request.model = 0;

  if (client.call(srv))
  {
    ROS_INFO("Received %ld detections.", srv.response.score.size());
  }
  else
  {
    ROS_ERROR("Failed to call service detect_object");
    return 1;
  }

  cv::Mat detection_image;
  cv::cvtColor(image, detection_image, CV_GRAY2BGR);

  cv::Scalar color(255, 0, 0);

  for (int i = 0; i < srv.response.score.size(); ++i) {
    cv::Point top_left(srv.response.left[i], srv.response.top[i]);
    cv::Point top_right(srv.response.right[i], srv.response.top[i]);
    cv::Point bottom_right(srv.response.right[i], srv.response.bottom[i]);
    cv::Point bottom_left(srv.response.left[i], srv.response.bottom[i]);

    cv::line(detection_image, top_left, top_right, color, 3);
    cv::line(detection_image, top_right, bottom_right, color, 3);
    cv::line(detection_image, bottom_right, bottom_left, color, 3);
    cv::line(detection_image, bottom_left, top_left, color, 3);
  }

  ros::Rate rate(10);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("/detections", 1);

  while (ros::ok())
    {
      cv::imshow("Window",detection_image);
      cv::waitKey(1);
      //image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8",
  image).toImageMsg());
      ros::spinOnce();
      rate.sleep();
    }
  */

  ros::Rate rate(pr_.rate);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

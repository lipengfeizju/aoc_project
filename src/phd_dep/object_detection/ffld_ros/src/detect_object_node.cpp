#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include "FFLDDetector.hpp"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <fstream>
#include <chrono>
#include "ffld_ros/ObjectDetection.h"
#include "ffld_ros/nms.h"

struct Params {
  std::string model_name;
  FFLD::Mixture mixture;
  FFLDDetector DPM;
  double overlap;   // Minimum overlap in non maxima suppression (default 0.5)
  double threshold; // Minimum detection threshold (default -1)
  int interval; // Number of levels per octave in the HOG pyramid (default 5)
  int padding;  // Amount of zero padding in HOG cells (default 6)
  double resize_val; // Value to scale image by. 0.5 means half in each dimension.
  double rate;    // Maximum rate that this node can run. -1 means no limit.
} pr_;

void imgCallback(const sensor_msgs::ImageConstPtr &msg,
                 const ros::Publisher &pub,
                 const image_transport::Publisher &pub_images) {
  cv::Mat cvimage;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(0, 0), pr_.resize_val, pr_.resize_val);

  cvimage = cv_ptr->image;

  sensor_msgs::ImagePtr resized = cv_ptr->toImageMsg();

  // Load image
  const FFLD::JPEGImage image(
      resized->width, resized->height,
      sensor_msgs::image_encodings::numChannels(resized->encoding),
      resized->data.data());

  using namespace std::chrono;

  // Detect
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<Detection> detections;
  if (pr_.DPM.detectImg(pr_.mixture, image, detections, pr_.overlap,
                        pr_.threshold, pr_.interval, pr_.padding) == -1) {
    ROS_WARN("Object detection failed!");
    return;
  }

  std::cout << pr_.overlap << " " << pr_.threshold << " " << pr_.interval << " "
            << pr_.padding << std::endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(t2 - t1).count();
  ROS_INFO_STREAM("[detect_object_node]: Detection took " << duration << " ms. " << detections.size()
                                    << " detections found.");

  ffld_ros::ObjectDetection msg_out;
  msg_out.header = msg->header;

  for (int i = 0; i < detections.size(); ++i) {
    msg_out.score.push_back(detections[i].score);
    msg_out.left.push_back(detections[i].left()); // make index 1-based instead of 0-based
    msg_out.top.push_back(detections[i].top());
    msg_out.right.push_back(detections[i].right());
    msg_out.bottom.push_back(detections[i].bottom());
  }

  // non-maxima suppression, i.e. remove duplicate detections
  msg_out = nms::nms(msg_out, 0.4);

  pub.publish(msg_out);

  // draw detections on image
  cv::Mat color_img;
  cv::cvtColor(cvimage, color_img, CV_GRAY2BGR);

  cv::Scalar color(255, 0, 0);

  for (int i = 0; i < detections.size(); ++i) {
    cv::Point top_left(msg_out.left[i], msg_out.top[i]);
    cv::Point top_right(msg_out.right[i], msg_out.top[i]);
    cv::Point bottom_right(msg_out.right[i], msg_out.bottom[i]);
    cv::Point bottom_left(msg_out.left[i], msg_out.bottom[i]);

    cv::line(color_img, top_left, top_right, color, 3);
    cv::line(color_img, top_right, bottom_right, color, 3);
    cv::line(color_img, bottom_right, bottom_left, color, 3);
    cv::line(color_img, bottom_left, top_left, color, 3);
    
    std::cout << msg_out.score[i] << ' ' << msg_out.left[i] << ' '
        << msg_out.top[i] << ' ' << msg_out.right[i] << ' '
        << msg_out.bottom[i] << std::endl;

  }

  cv_bridge::CvImage img_msg;
  img_msg.header = msg->header;
  img_msg.image = color_img;
  img_msg.encoding = sensor_msgs::image_encodings::BGR8;

  pub_images.publish(img_msg.toImageMsg());
}

int main(int argc, char **argv) {
  // ROS_INFO("argv[0] = %s",argv[0]);
  ros::init(argc, argv, "object_detector");

  ros::NodeHandle nh("object_detector");

  nh.param("overlap", pr_.overlap, 0.5);
  nh.param("threshold", pr_.threshold, -1.0);
  nh.param("padding", pr_.padding, 6);
  nh.param("interval", pr_.interval, 5);
  nh.param("resize_val", pr_.resize_val, 1.0);
  nh.param("rate", pr_.rate, -1.0);
  // nh.param<std::string>("model_name", pr_.model_name, "object");

  std::string cam_topic;

  if (!nh.getParam("cam_topic", cam_topic)) {
    ROS_WARN("Parameter cam_topic not set, using default \"/cam0/image_raw\"");
    cam_topic = "/cam0/image_raw";
  }

  if (!nh.getParam("model_name", pr_.model_name)) {
    ROS_WARN("Parameter model_name not set");
    return 1;
  }

  // Try to read the model
  std::ifstream in(pr_.model_name.c_str(), std::ios::binary);
  if (!in.is_open()) {
    ROS_ERROR("[detect_object_server]: Invalid model file: %s",
              pr_.model_name.c_str());
    return 1;
  }
  in >> pr_.mixture;
  if (pr_.mixture.empty()) {
    ROS_ERROR("[detect_object_server]: Invalid model file: %s",
              pr_.model_name.c_str());
    return 1;
  }

  ros::Publisher pub_detections =
      nh.advertise<ffld_ros::ObjectDetection>("object_detections", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_images = it.advertise("image", 1);
  // using namespace std::placeholders;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(
      cam_topic, 1, boost::bind(imgCallback, _1, pub_detections, pub_images));

  ROS_INFO("[detect_object_server]: Server started with model: %s\n Resolution "
           "is scaled by %f\n Max rate is %f",
           pr_.model_name.c_str(), pr_.resize_val, pr_.rate);
  // ROS_INFO("[detect_object_server]: Ready to detect objects.");

  if (pr_.rate == -1) {
    ros::spin();
  } else {
    ros::Rate rate(pr_.rate);
    while (ros::ok()) {
      rate.sleep();
      ros::spinOnce();
    }
  }

  return 0;
}

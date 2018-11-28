#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <fstream>
#include <chrono>
#include "ffld_ros/ObjectDetection.h"

class DetectionImageCoalescer {
public:
  DetectionImageCoalescer(const std::vector<std::string>& model_names, ros::NodeHandle& nh);

  void imgCallback(const sensor_msgs::ImageConstPtr& msg);
  void detCallback(const ffld_ros::ObjectDetection::ConstPtr& msg, int model_id);
private:

  void tryPublishImage();
  void initializeColor(size_t i);

  ros::NodeHandle& nh_;
  image_transport::ImageTransport it_;

  image_transport::Publisher img_pub_;

  ros::Subscriber img_sub_;
  std::vector<ros::Subscriber> det_subs_;

  std::vector<std::string> model_names_;

  std::vector<cv::Scalar> colors_;

  std::deque<sensor_msgs::ImageConstPtr> images_;
  std::vector< std::deque<ffld_ros::ObjectDetection::ConstPtr> > detections_;
};

DetectionImageCoalescer::DetectionImageCoalescer(const std::vector<std::string>& model_names, ros::NodeHandle& nh) 
  : nh_(nh),
    it_(nh) {
  img_pub_ = it_.advertise("image", 10);

  img_sub_ = nh.subscribe<sensor_msgs::Image>("/klt/image", 10, &DetectionImageCoalescer::imgCallback, this);

  model_names_ = model_names;

  detections_.resize(model_names_.size());
  colors_.resize(model_names_.size());

  for (size_t i = 0; i < model_names.size(); ++i) {
    std::string topic = std::string("/object_detector/") + model_names[i] + "/detections";
    det_subs_.push_back( nh.subscribe<ffld_ros::ObjectDetection>(topic, 10, boost::bind(&DetectionImageCoalescer::detCallback, this, _1, i)) );
    initializeColor(i);
  }
}

void DetectionImageCoalescer::imgCallback(const sensor_msgs::ImageConstPtr& msg) {
  images_.push_back(msg);
}

void DetectionImageCoalescer::detCallback(const ffld_ros::ObjectDetection::ConstPtr& msg, int model_id) {
  detections_[model_id].push_back(msg);
  tryPublishImage();
}

void DetectionImageCoalescer::initializeColor(size_t i) {
  // assign a color to object i
  switch (i) {
    case 0:
      colors_[i] = cv::Scalar(0, 255, 0); // green
      break;

    case 1: 
      colors_[i] = cv::Scalar(0, 255, 255); // yellow
      break;

    case 2:
      colors_[i] = cv::Scalar(0, 0, 255); // red (BGR image)
      break;

    case 3:
      colors_[i] = cv::Scalar(255, 0, 0);  // blue

    default:
      colors_[i] = cv::Scalar(rand() % 256, rand() % 256, rand() % 256); // random
      break;
  }
}

void DetectionImageCoalescer::tryPublishImage() {
  // wait until we have all detections and the image for the next time step
  for (size_t i = 0; i < model_names_.size(); ++i) {
    if (detections_[i].empty()) return;
  }

  if (images_.empty()) return;

  // ROS_INFO_STREAM("drawing");

  // don't explicitly check time synchronization but assume it's ok

  sensor_msgs::ImageConstPtr img_msg = images_.front();
  images_.pop_front();

  cv::Mat cvimage;
  try {
      cvimage = cv_bridge::toCvShare(img_msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // loop through all detection messages and draw boxes
  cv::Mat color_img;
  cv::cvtColor(cvimage, color_img, CV_GRAY2BGR);

  for (size_t i = 0; i < model_names_.size(); ++i) {

    cv::Scalar color = colors_[i];

    ffld_ros::ObjectDetection::ConstPtr msg = detections_[i].front();
    detections_[i].pop_front();

    for (int j = 0; j < msg->left.size(); ++j) {
      cv::Point top_left(msg->left[j], msg->top[j]);
      cv::Point top_right(msg->right[j], msg->top[j]);
      cv::Point bottom_right(msg->right[j], msg->bottom[j]);
      cv::Point bottom_left(msg->left[j], msg->bottom[j]);

      cv::line(color_img, top_left, top_right, color, 3);
      cv::line(color_img, top_right, bottom_right, color, 3);
      cv::line(color_img, bottom_right, bottom_left, color, 3);
      cv::line(color_img, bottom_left, top_left, color, 3);

      cv::putText(color_img, model_names_[i].c_str(), top_left, cv::FONT_HERSHEY_COMPLEX, 1.0, color, 2.0, 8);
    }
  }

  cv_bridge::CvImage img_msg_out;
  img_msg_out.header = img_msg->header;
  img_msg_out.image = color_img;
  img_msg_out.encoding = sensor_msgs::image_encodings::BGR8;
  img_pub_.publish(img_msg_out.toImageMsg());
}

int main(int argc, char **argv)
{
  // ROS_INFO("argv[0] = %s",argv[0]);
  ros::init(argc, argv, "object_detection_visualizer");
  
  ros::NodeHandle nh("object_detection_visualizer");

  std::vector<std::string> model_names;
  nh.getParam("model_names", model_names);

  DetectionImageCoalescer dic(model_names, nh);
  
  // ROS_INFO("[detect_object_server]: Ready to detect objects.");
  ros::spin();

  return 0;
}



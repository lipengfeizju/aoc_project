#ifndef DPM_DETECT_CLIENT_CPP_
#define DPM_DETECT_CLIENT_CPP_

#include "ffld_ros/dpm_detect_client.h"

namespace dpm_detect_client {

DpmDetectClient::DpmDetectClient(ros::NodeHandle nh) {
  params_client = nh.serviceClient<ffld_ros::SetDpmParams>("set_dpm_params");
  detection_client = nh.serviceClient<ffld_ros::DetectObject>("detect_object");
  params = new Params;
}

// detection_image
ffld_ros::DetectObject
DpmDetectClient::requestDetection(sensor_msgs::Image image) {
  ffld_ros::DetectObject srv;
  srv.request.img = image;
  if (detection_client.call(srv)) {
    //ROS_INFO("[dpm_detect_client]: Received %ld detections.", srv.response.score.size());
  } else {
    ROS_ERROR("[dpm_detect_client]: Failed to call service detect_object");
  }

  return srv;
}

void DpmDetectClient::setDpmParams(dpm_detect_client::Params params_in) {
  ros::service::waitForService("set_dpm_params");
  params->model_num = params_in.model_num;
  params->overlap = params_in.overlap;
  params->threshold = params_in.threshold;
  params->padding = params_in.padding;
  params->interval = params_in.interval;
  params->resize_val = params_in.resize_val;

  ffld_ros::SetDpmParams srv;
  srv.request.model_num = params->model_num;
  srv.request.set_vals = true;
  srv.request.overlap = params->overlap;
  srv.request.threshold = params->threshold;
  srv.request.padding = params->padding;
  srv.request.interval = params->interval;
  srv.request.resize_val = params->resize_val;

  if (params_client.call(srv)) {
    //ROS_INFO("[dpm_detect_client]: Params set. Response is: %d.", srv.response.success);
  } else {
    ROS_ERROR("[dpm_detect_client]: Failed to call service set_dpm_params");
  }

  return;
}

DpmDetectClient::~DpmDetectClient() {}

cv::Mat DpmDetectClient::drawDetection(const cv::Mat &image,
                                       ffld_ros::DetectObject srv) {
  cv::Mat detection_image;

  cv::resize(image, detection_image, cv::Size(0, 0), params->resize_val,
             params->resize_val);

  cv::cvtColor(detection_image, detection_image, CV_GRAY2BGR);

  if (srv.response.left.size() > 0) {
    cv::Scalar color(255, 0, 0);

    cv::Point top_left(srv.response.left[0], srv.response.top[0]);
    cv::Point top_right(srv.response.right[0], srv.response.top[0]);
    cv::Point bottom_right(srv.response.right[0], srv.response.bottom[0]);
    cv::Point bottom_left(srv.response.left[0], srv.response.bottom[0]);
    
    cv::line(detection_image, top_left, top_right, color, 3);
    cv::line(detection_image, top_right, bottom_right, color, 3);
    cv::line(detection_image, bottom_right, bottom_left, color, 3);
    cv::line(detection_image, bottom_left, top_left, color, 3);
  }
  return detection_image;
}
}

#endif

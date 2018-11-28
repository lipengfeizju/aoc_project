#ifndef FLA_DPM_CLIENT_H_
#define FLA_DPM_CLIENT_H_

#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include "ffld_ros/SetDpmParams.h"
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include "ffld_ros/ObjectDetection.h"
#include "ffld_ros/dpm_object_detector.h"

namespace fla_dpm_client {

struct Params;

class FlaDpmClient {
 public:
  FlaDpmClient(ros::NodeHandle nh);
  ~FlaDpmClient();
  
  ffld_ros::DetectObject requestDetection(sensor_msgs::Image image);
  void setDpmParams(dpm_detect_client::Params params);
  cv::Mat drawDetection(const cv::Mat &image,
                        ffld_ros::DetectObject srv_response);

 protected:
  ros::ServiceServer service;
  ros::Subscriber goal_sub;
  ros::Subscriber odom_sub;
  double eps;
  double rate;
  bool started, debug;

  // Variables
  ros::ServiceClient params_client;
  ros::ServiceClient detection_client;
  Params* params;

  
};
}

#endif

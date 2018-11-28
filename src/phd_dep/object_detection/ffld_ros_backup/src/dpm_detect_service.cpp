#ifndef DPM_DETECT_SERVICE_CPP
#define DPM_DETECT_SERVICE_CPP

#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include "ffld_ros/SetDpmParams.h"
#include "FFLDDetector.hpp"
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <fstream>
#include "dpm_object_detector.cpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

dpm_object_detector::DpmObjectDetector detector;

bool detect_object(ffld_ros::DetectObject::Request &req,
                   ffld_ros::DetectObject::Response &res) {
  cv::Mat cvimage;
  try {
    cvimage = cv_bridge::toCvCopy(req.img, "mono8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("[dpm_detect_service]: cv_bridge exception: %s", e.what());
  }

  int err = detector.runDetection(cvimage);

  if (err != 0) {
    ROS_ERROR("[dpm_detect_service]: error, could not run detection");
    return false;
  }

  ffld_ros::ObjectDetection detection = detector.getDetection();

  for (int i = 0; i < detection.score.size(); ++i) {
    res.score.push_back(detection.score[i]);
    res.left.push_back(detection.left[i]);
    res.top.push_back(detection.top[i]);
    res.right.push_back(detection.right[i]);
    res.bottom.push_back(detection.bottom[i]);
  }

  return true;
}

bool set_dpm_params(ffld_ros::SetDpmParams::Request &req,
                    ffld_ros::SetDpmParams::Response &res) {
  if (req.set_vals) {
    return detector.setParams(req.model_num, req.overlap, req.threshold,
                              req.padding, req.interval, req.resize_val);
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "detect_object_service");

  double overlap, threshold, resize_val, rate;
  int interval, padding;
  std::string model_name;

  ros::NodeHandle pnh("~");
  overlap = 0.5;
  threshold = -0.5;
  interval = 5;
  padding = 6;
  resize_val = 1.0;
  rate = -1.0;

  std::string path;
  std::vector<std::string> models;
  pnh.getParam("model_path", path);
  pnh.getParam("models", models);

  std::vector<int> valid = detector.loadModels(path, models);

  if (valid.size() != models.size()) {
    ROS_ERROR("[dpm_detect_service]: Invalid path to model: %s",
              path.c_str());
  }

  ROS_INFO("[dpm_detect_service]: Service started");
  ros::NodeHandle nh;
  ros::ServiceServer detection_service =
      nh.advertiseService("detect_object", detect_object);
  ros::ServiceServer param_service =
      nh.advertiseService("set_dpm_params", set_dpm_params);

  ROS_INFO("[dpm_detect_service]: Ready to detect objects.");
  ros::spin();

  return 0;
}

#endif

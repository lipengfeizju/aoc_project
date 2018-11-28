#ifndef DPM_DETECT_CLIENT_H_
#define DPM_DETECT_CLIENT_H_

#include "ros/ros.h"
#include "ffld_ros/DetectObject.h"
#include "ffld_ros/SetDpmParams.h"
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include "ffld_ros/ObjectDetection.h"
#include "ffld_ros/dpm_object_detector.h"

namespace dpm_detect_client {
struct Params {
  double overlap;   // Minimum overlap in non maxima suppression (default 0.5)
  double threshold; // Minimum detection threshold (default -1)
  int interval;  // Number of levels per octave in the HOG pyramid (default 5)
  int padding;   // Amount of zero padding in HOG cells (default 6)
  int model_num; // Index for model to be used
  double
      resize_val; // Value to scale image by. 0.5 means half in each dimension.
  double rate;    // Maximum rate that this node can run. -1 means no limit.
};

class DpmDetectClient {
public:
  DpmDetectClient(ros::NodeHandle nh);
  ~DpmDetectClient();
  ffld_ros::DetectObject requestDetection(sensor_msgs::Image image);
  void setDpmParams(dpm_detect_client::Params params);
  cv::Mat drawDetection(const cv::Mat &image,
                        ffld_ros::DetectObject srv_response);

private:
  // Variables
  ros::ServiceClient params_client;
  ros::ServiceClient detection_client;
  Params *params;
};
}

#endif

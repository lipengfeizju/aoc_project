#ifndef DPM_OBJECT_DETECTOR_H_
#define DPM_OBJECT_DETECTOR_H_

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
//#include "ffld_ros/nms.h"

namespace dpm_object_detector {

class DpmObjectDetector {
public:
  DpmObjectDetector();
  ~DpmObjectDetector();
  int runDetection(const cv::Mat &image);
  cv::Mat drawDetection(const cv::Mat &image);
  std::vector<int> loadModels(std::string model_path, std::vector<std::string> models_in);
  ffld_ros::ObjectDetection getDetection();
  bool setParams(int model_num_in, double overlap_in, double threshold_in,
                 int padding_in, int interval_in, double resize_val_in);

private:
  // Variables
  ffld_ros::ObjectDetection detection; // Stores last detection
  int num_models;
  // Params
  std::vector<FFLD::Mixture> mixtures;
  FFLDDetector DPM;
  double overlap;   // Minimum overlap in non maxima suppression (default 0.5)
  double threshold; // Minimum detection threshold (default -1)
  int interval; // Number of levels per octave in the HOG pyramid (default 5)
  int padding;  // Amount of zero padding in HOG cells (default 6)
  int model_num;
  double
      resize_val; // Value to scale image by. 0.5 means half in each dimension.
  double rate;    // Maximum rate that this node can run. -1 means no limit.
};
}

#endif

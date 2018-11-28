#ifndef DPM_OBJECT_DETECTOR_CPP_
#define DPM_OBJECT_DETECTOR_CPP_

#include "ffld_ros/dpm_object_detector.h"
#include "nms.cpp"

namespace dpm_object_detector {
DpmObjectDetector::DpmObjectDetector() {
  overlap = 0.5;
  threshold = -0.5;
  padding = 6;
  interval = 5;
  resize_val = 1.0;
  model_num = 0;
  num_models = 0;
}

std::vector<int>
DpmObjectDetector::loadModels(std::string model_path, std::vector<std::string> models_in) {
  std::vector<int> valid_models;

  for (int i = 0; i < models_in.size(); i++) {
    FFLD::Mixture mixture;
    std::ifstream in((model_path+models_in[i]).c_str(), std::ios::binary);
    if (!in.is_open()) {
      continue;
    }
    in >> mixture;
    if (mixture.empty()) {
      continue;
    }
    mixtures.push_back(mixture);

    valid_models.push_back(i);
  }

  return valid_models;
}

DpmObjectDetector::~DpmObjectDetector() {}

ffld_ros::ObjectDetection DpmObjectDetector::getDetection() {
  return detection;
}

bool DpmObjectDetector::setParams(int model_num_in, double overlap_in,
                                  double threshold_in, int padding_in,
                                  int interval_in, double resize_val_in) {
  overlap = overlap_in;
  threshold = threshold_in;
  padding = padding_in;
  interval = interval_in;
  resize_val = resize_val_in;
  if (model_num_in <= mixtures.size()) {
    model_num = model_num_in;
    return true;
  } else {
    return false;
  }
}

int DpmObjectDetector::runDetection(const cv::Mat &image) {
  cv::Mat det_image;
  cv::resize(image, det_image, cv::Size(0, 0), resize_val, resize_val);

  // Load image
  const FFLD::JPEGImage jpegimage(det_image.cols, det_image.rows,
                                  det_image.channels(), det_image.data);

  using namespace std::chrono;

  // Detect
  //high_resolution_clock::time_point t1 = high_resolution_clock::now();

  std::vector<Detection> detections;

  if (DPM.detectImg(mixtures[model_num], jpegimage, detections, overlap,
                    threshold, interval, padding) == -1) {
    ROS_WARN("Object detection failed!");
    return -1;
  }

  //high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //auto duration = duration_cast<milliseconds>(t2 - t1).count();
  //  ROS_INFO_STREAM("[dpm_object_detector]: Detection took " << duration << " ms." << detections.size() << " detections found.");

  detection.score.clear();
  detection.left.clear();
  detection.top.clear();
  detection.right.clear();
  detection.bottom.clear();

  for (int i = 0; i < detections.size(); i++) {
    detection.score.push_back(detections[i].score);
    detection.left.push_back(
        detections[i].left()); // make index 1-based instead of 0-based
    detection.top.push_back(detections[i].top());
    detection.right.push_back(detections[i].right());
    detection.bottom.push_back(detections[i].bottom());
  }

  // non-maxima suppression, i.e. remove duplicate detections
  detection = nms::nms(detection, 0.4);

  return 0;
}

cv::Mat DpmObjectDetector::drawDetection(const cv::Mat &image) {
  cv::Mat detection_image;
  cv::cvtColor(image, detection_image, CV_GRAY2BGR);

  cv::Scalar color(255, 0, 0);

  for (int i = 0; i < detection.score.size(); ++i) {
    cv::Point top_left(detection.left[i], detection.top[i]);
    cv::Point top_right(detection.right[i], detection.top[i]);
    cv::Point bottom_right(detection.right[i], detection.bottom[i]);
    cv::Point bottom_left(detection.left[i], detection.bottom[i]);

    cv::line(detection_image, top_left, top_right, color, 3);
    cv::line(detection_image, top_right, bottom_right, color, 3);
    cv::line(detection_image, bottom_right, bottom_left, color, 3);
    cv::line(detection_image, bottom_left, top_left, color, 3);
  }
  return detection_image;
}
}

/*int main(int argc, char **argv)
  {
  dpm_object_detector::DpmObjectDetector test;
  return 0;
  }*/

#endif

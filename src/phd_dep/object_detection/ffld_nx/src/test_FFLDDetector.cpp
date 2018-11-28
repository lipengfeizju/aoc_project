

#include "FFLDDetector.hpp"

#include <string>
#include <fstream>
#include <iostream>

int main(int argc, char ** argv)
{
  const std::string model_name(argv[1]);
  const std::string jpg_file(argv[2]);
  
  // Try to open the mixture
  std::ifstream in(model_name.c_str(), std::ios::binary);
  if (!in.is_open()) {
	  std::cerr << "\nInvalid model file " << model_name << std::endl;
	  return -1;
  }
  
  // Read model
  FFLD::Mixture mixture;
  in >> mixture;
  if (mixture.empty()) {
	  std::cerr << "\nInvalid model file " << model_name << std::endl;
	  return -1;
  }
  
  // Load the image
  const FFLD::JPEGImage image(jpg_file);
  
  // Detect
  std::vector<Detection> detections;
  FFLDDetector DPM;
  if( DPM.detectImg( mixture, image, detections, 0.5, -0.5 ) == - 1 )
  {
    std::cerr << "\nDetection unsuccessful" << std::endl;
    return -1;
  }

  for (int i = 0; i < detections.size(); ++i) {
    std::cout << detections[i].score << ' ' << (detections[i].left() + 1) << ' '
				<< (detections[i].top() + 1) << ' ' << (detections[i].right() + 1) << ' '
				<< (detections[i].bottom() + 1) << std::endl;
  }
  return 0;
}



//--------------------------------------------------------------------------------------------------
// Implementation of the papers "Exact Acceleration of Linear Object Detectors", 12th European
// Conference on Computer Vision, 2012 and "Deformable Part Models with Individual Part Scaling",
// 24th British Machine Vision Conference, 2013.
//
// Copyright (c) 2013 Idiap Research Institute, <http://www.idiap.ch/>
// Written by Charles Dubout <charles.dubout@idiap.ch>
//
// This file is part of FFLDv2 (the Fast Fourier Linear Detector version 2)
//
// FFLDv2 is free software: you can redistribute it and/or modify it under the terms of the GNU
// Affero General Public License version 3 as published by the Free Software Foundation.
//
// FFLDv2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero
// General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License along with FFLDv2. If
// not, see <http://www.gnu.org/licenses/>.
//--------------------------------------------------------------------------------------------------

#ifndef FFLDDetector_HPP_
#define FFLDDetector_HPP_

#include "SimpleOpt.h"
#include "Intersector.h"
#include "Mixture.h"
//#include "Scene.h"
#include <vector>
#include <iostream>

struct Detection : public FFLD::Rectangle
{
	FFLD::HOGPyramid::Scalar score;
	int x;
	int y;
	int z;
	
	Detection() : score(0), x(0), y(0), z(0)
	{
	}
	
	Detection(FFLD::HOGPyramid::Scalar score, int x, int y, int z, FFLD::Rectangle bndbox) : Rectangle(bndbox),
	score(score), x(x), y(y), z(z)
	{
	}
	
	bool operator<(const Detection & detection) const
	{
		return score > detection.score;
	}
};


class FFLDDetector
{  
public:	
  // @INPUTS:
  //  overlap = Minimum overlap in non maxima suppression (default 0.5)  
  //  threshold = Minimum detection threshold (default -1)
  //  interval = Number of levels per octave in the HOG pyramid (default 5)
  //  padding = Amount of zero padding in HOG cells (default 6)
  // 
  // @OUTPUT:
  //  detections
  int detectImg( const FFLD::Mixture & mixture, const FFLD::JPEGImage &image, std::vector<Detection> & detections,
              double overlap = 0.5, double threshold = -1.0, int interval = 5, int padding = 6 )
  {
    // Compute the HOG features
    FFLD::HOGPyramid pyramid(image, padding, padding, interval);
    if (pyramid.empty()) {
			std::cerr << "\nInvalid image " << std::endl;
			return -1;
		}
		
	  // Initialize the Patchwork class
		if (!FFLD::Patchwork::InitFFTW((pyramid.levels()[0].rows() - padding + 15) & ~15,
								 (pyramid.levels()[0].cols() - padding + 15) & ~15))
		{
			std::cerr << "\nCould not initialize the Patchwork class" << std::endl;
			return -1;
		}

		// Transform the filters (Only after Patchwork initialization)
		mixture.cacheFilters();
		
		// Compute the detections
		detect(mixture, image.width(), image.height(), pyramid, threshold, overlap, detections);
		return 0;
  }

private:
  void detect(const FFLD::Mixture & mixture, int width, int height, const FFLD::HOGPyramid & pyramid,
			double threshold, double overlap, std::vector<Detection> & detections)
  {
	  // Compute the scores
	  std::vector<FFLD::HOGPyramid::Matrix> scores;
	  std::vector<FFLD::Mixture::Indices> argmaxes;
	  std::vector<std::vector<std::vector<FFLD::Model::Positions> > > positions;
	  
	  mixture.convolve(pyramid, scores, argmaxes, &positions);
	  
	  // Cache the size of the models
	  std::vector<std::pair<int, int> > sizes(mixture.models().size());
	  
	  for (int i = 0; i < sizes.size(); ++i)
		  sizes[i] = mixture.models()[i].rootSize();
		  
	  // For each scale
	  for (int z = 0; z < scores.size(); ++z)
	  {
		  const double scale = pow(2.0, static_cast<double>(z) / pyramid.interval() + 2);
		
		  const int rows = static_cast<int>(scores[z].rows());
		  const int cols = static_cast<int>(scores[z].cols());
		
		  for (int y = 0; y < rows; ++y) {
			  for (int x = 0; x < cols; ++x)
			  {
				  const double score = scores[z](y, x);
				
				  if (score > threshold)
				  {
					  // Non-maxima suppresion in a 3x3 neighborhood
					  if (((y == 0) || (x == 0) || (score >= scores[z](y - 1, x - 1))) &&
						  ((y == 0) || (score >= scores[z](y - 1, x))) &&
						  ((y == 0) || (x == cols - 1) || (score >= scores[z](y - 1, x + 1))) &&
						  ((x == 0) || (score >= scores[z](y, x - 1))) &&
						  ((x == cols - 1) || (score >= scores[z](y, x + 1))) &&
						  ((y == rows - 1) || (x == 0) || (score >= scores[z](y + 1, x - 1))) &&
						  ((y == rows - 1) || (score >= scores[z](y + 1, x))) &&
						  ((y == rows - 1) || (x == cols - 1) ||
						   (score >= scores[z](y + 1, x + 1)))) 
						{
						  FFLD::Rectangle bndbox((x - pyramid.padx()) * scale + 0.5,
										   (y - pyramid.pady()) * scale + 0.5,
										   sizes[argmaxes[z](y, x)].second * scale + 0.5,
										   sizes[argmaxes[z](y, x)].first * scale + 0.5);
						
						  // Truncate the object
						  bndbox.setX(std::max(bndbox.x(), 0));
						  bndbox.setY(std::max(bndbox.y(), 0));
						  bndbox.setWidth(std::min(bndbox.width(), width - bndbox.x()));
						  bndbox.setHeight(std::min(bndbox.height(), height - bndbox.y()));
						
						  if (!bndbox.empty())
							  detections.push_back(Detection(score, x, y, z, bndbox));
					  }
				  }
			  }
		  }
	  }
	  
	  // Non maxima suppression
	  std::sort(detections.begin(), detections.end());
	
	  for (int i = 1; i < detections.size(); ++i)
		  detections.resize(std::remove_if(detections.begin() + i, detections.end(),
									  FFLD::Intersector(detections[i - 1], overlap, true)) - detections.begin());		
  }
};

#endif


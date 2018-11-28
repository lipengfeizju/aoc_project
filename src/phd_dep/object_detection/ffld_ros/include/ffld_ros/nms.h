#ifndef NMS_H_
#define NMS_H_

#include <vector>
#include <algorithm>

#include <ffld_ros/ObjectDetection.h>

namespace nms {
template <typename T>
std::vector<size_t> sort_indices(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  for (size_t i = 0; i < idx.size(); ++i)
    idx[i] = i;

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {
    return v[i1] > v[i2];
  }); // sort in *descending* order

  return idx;
}

ffld_ros::ObjectDetection nms(const ffld_ros::ObjectDetection &msg,
                              double overlap) {
  if (msg.left.size() == 0) {
    return msg;
  }

  auto x1 = msg.left;
  auto x2 = msg.right;

  auto y1 = msg.top;
  auto y2 = msg.bottom;

  auto s = msg.score;

  // compute box areas
  std::vector<double> area(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    area[i] = (x2[i] - x1[i] + 1) * (y2[i] - y1[i] + 1);
  }

  ffld_ros::ObjectDetection out_msg;

  std::vector<size_t> sorted_indices = sort_indices(s);
  std::vector<bool> skip_indices(s.size(), false);

  for (size_t ipos = 0; ipos < sorted_indices.size(); ++ipos) {
    if (skip_indices[ipos])
      continue;

    size_t i = sorted_indices[ipos];

    out_msg.left.push_back(x1[i]);
    out_msg.right.push_back(x2[i]);
    out_msg.top.push_back(y1[i]);
    out_msg.bottom.push_back(y2[i]);
    out_msg.score.push_back(s[i]);

    for (size_t jpos = ipos + 1; jpos < s.size(); ++jpos) {
      if (skip_indices[jpos])
        continue;

      size_t j = sorted_indices[jpos];

      int xx1 = std::max(x1[i], x1[j]);
      int yy1 = std::max(y1[i], y1[j]);
      int xx2 = std::min(x2[i], x2[j]);
      int yy2 = std::min(y2[i], y2[j]);

      int w = xx2 - xx1 + 1;
      int h = yy2 - yy1 + 1;

      // if (x2[j] < x2[i] && x1[j] > x1[i] && y2[j] < y2[i] && y1[j] > y1[i])
      // 	skip_indices[jpos] = true;

      // if (x2[j] > x2[i] && x1[j] < x1[i] && y2[j] > y2[i] && y1[j] < y1[i])
      // 	skip_indices[jpos] = true;

      if (w > 0 && h > 0) {
        double o = w * h / area[j];
        double o2 = w * h / area[i];

        if (o > overlap || o2 > overlap) {
          skip_indices[jpos] = true;
        }
      }
    }
  }

  return out_msg;
}
}
#endif

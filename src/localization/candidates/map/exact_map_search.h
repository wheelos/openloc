// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <vector>

#include "flann/flann.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace openloc {
namespace localization {
namespace unified {

// Same XYZ layout, distance, tree and exact-search parameters as KdTreeFLANN.
// Batch queries amortize FLANN's result-set allocation and parallel-region setup.
class ExactMapSearch {
 public:
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    index_.reset();
    points_.resize(cloud->size() * 3);
    for (size_t i = 0; i < cloud->size(); ++i) {
      points_[i * 3] = (*cloud)[i].x;
      points_[i * 3 + 1] = (*cloud)[i].y;
      points_[i * 3 + 2] = (*cloud)[i].z;
    }
    if (points_.empty()) return;
    index_ = std::make_unique<Index>(
        flann::Matrix<float>(points_.data(), cloud->size(), 3),
        flann::KDTreeSingleIndexParams(15));
    index_->buildIndex();
  }

  int nearestKSearch(const pcl::PointXYZ& point, unsigned int k,
                     std::vector<int>& indices,
                     std::vector<float>& distances) const {
    k = std::min<size_t>(k, points_.size() / 3);
    indices.resize(k);
    distances.resize(k);
    if (k == 0) return 0;
    std::array<float, 3> query{{point.x, point.y, point.z}};
    std::vector<size_t> neighbors(k);
    flann::Matrix<size_t> neighbor_matrix(neighbors.data(), 1, k);
    flann::Matrix<float> distance_matrix(distances.data(), 1, k);
    index_->knnSearch(flann::Matrix<float>(query.data(), 1, 3),
                      neighbor_matrix, distance_matrix, k, Parameters());
    std::copy(neighbors.begin(), neighbors.end(), indices.begin());
    return k;
  }

  void NearestBatch(const std::vector<float>& queries,
                    std::vector<size_t>* indices,
                    std::vector<float>* distances) const {
    const size_t count = queries.size() / 3;
    indices->resize(count);
    distances->resize(count);
    if (count == 0) return;
    flann::Matrix<size_t> neighbor_matrix(indices->data(), count, 1);
    flann::Matrix<float> distance_matrix(distances->data(), count, 1);
    index_->knnSearch(
        flann::Matrix<float>(const_cast<float*>(queries.data()), count, 3),
        neighbor_matrix, distance_matrix, 1, Parameters());
  }

 private:
  static flann::SearchParams Parameters() {
    flann::SearchParams parameters(-1, 0.0f);
    parameters.cores = 1;
    return parameters;
  }

  using Index = flann::Index<flann::L2_Simple<float>>;
  std::vector<float> points_;
  std::unique_ptr<Index> index_;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc

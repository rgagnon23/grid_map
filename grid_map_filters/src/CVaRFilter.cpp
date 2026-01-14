/*
 * CVaRFilter.cpp
 *
 *  Created on: Nov 28, 2025
 *      Author: Riana Gagnon (based on GridMap filters by ANYbotics)
 */

#include "grid_map_filters/CVaRFilter.hpp"

#include <algorithm>
#include <numeric>

#include <ros/ros.h>

namespace grid_map {

CVaRFilter::CVaRFilter()
    : alpha_(0.2),
      windowSize_(3),
      useWindowLength_(false),
      windowLength_(0.0),
      isComputeEmptyCells_(true),
      edgeHandling_(SlidingWindowIterator::EdgeHandling::INSIDE) {}

CVaRFilter::~CVaRFilter() = default;

bool CVaRFilter::configure() {
  // ---- Required params ----
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("CVaRFilter missing required parameter 'input_layer'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("CVaRFilter missing required parameter 'output_layer'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("alpha"), alpha_)) {
    ROS_ERROR("CVaRFilter missing required parameter 'alpha'.");
    return false;
  }

  // ---- Window size OR window length ----
  if (!FilterBase::getParam(std::string("window_size"), windowSize_)) {
    if (FilterBase::getParam(std::string("window_length"), windowLength_)) {
      useWindowLength_ = true;
    } else {
      ROS_ERROR("CVaRFilter: must specify either 'window_size' or 'window_length'.");
      return false;
    }
  }

  // ---- Optional flags ----
  if (!FilterBase::getParam(std::string("compute_empty_cells"), isComputeEmptyCells_)) {
    ROS_ERROR("CVaRFilter missing parameter 'compute_empty_cells'.");
    return false;
  }

  // ---- Edge handling ----
  std::string edgeHandlingMethod;
  if (!FilterBase::getParam(std::string("edge_handling"), edgeHandlingMethod)) {
    ROS_ERROR("CVaRFilter missing required parameter 'edge_handling'.");
    return false;
  }

  if (edgeHandlingMethod == "inside") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::INSIDE;
  } else if (edgeHandlingMethod == "crop") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::CROP;
  } else if (edgeHandlingMethod == "empty") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::EMPTY;
  } else if (edgeHandlingMethod == "mean") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::MEAN;
  } else {
    ROS_ERROR("CVaRFilter: unknown edge handling method '%s'.",
              edgeHandlingMethod.c_str());
    return false;
  }

  return true;
}

bool CVaRFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  // Copy the entire map, then add the CVaR layer
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  Matrix& outputData = mapOut[outputLayer_];

  // Setup sliding window
  grid_map::SlidingWindowIterator iterator(mapIn, inputLayer_, edgeHandling_, windowSize_);
  if (useWindowLength_) {
    iterator.setWindowLength(mapIn, windowLength_);
  }

  for (; !iterator.isPastEnd(); ++iterator) {

    const Eigen::MatrixXf& window = iterator.getData();
    int linearIndex = iterator.getLinearIndex();

    // Extract finite values only
    std::vector<float> values;
    values.reserve(window.size());

    for (int r = 0; r < window.rows(); ++r) {
      for (int c = 0; c < window.cols(); ++c) {
        float v = window(r, c);
        if (std::isfinite(v)) {
          values.push_back(v);
        }
      }
    }

    // Handle empty window
    if (values.empty()) {
      if (isComputeEmptyCells_) {
        outputData(linearIndex) = NAN;
      }
      continue;
    }

    // Sort ascending to get worst (highest) tail or worst (lowest) tail depending on definition.
    // Risk normally means "high = bad", so CVaR is upper tail.
    std::sort(values.begin(), values.end());

    // Determine tail cutoff
    size_t N = values.size();
    size_t tailCount = std::max<size_t>(1, static_cast<size_t>(std::ceil(alpha_ * N)));

    // Compute CVaR: mean of tail (highest values)
    float sumTail = 0.0f;
    for (size_t i = N - tailCount; i < N; ++i) {
      sumTail += values[i];
    }
    float cvar = sumTail / static_cast<float>(tailCount);

    outputData(linearIndex) = cvar;
  }

  return true;
}

}  // namespace grid_map

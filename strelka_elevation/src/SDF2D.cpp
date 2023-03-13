#include "strelka_elevation/SDF2D.hpp"

#include <cmath>

#include <tbb/task_scheduler_init.h>
#include <tbb/tbb.h>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace filters;

namespace strelka_elevation {

SDF2D::SDF2D() {}

SDF2D::~SDF2D() = default;

bool SDF2D::configure() {

  if (!FilterBase::getParam(std::string("threshold"), threshold)) {
    ROS_ERROR("SDF2D filter did not find parameter threshold.");
    return false;
  }

  if (!FilterBase::getParam(std::string("safe_zone"), safe_zone)) {
    ROS_ERROR("SDF2D filter did not find parameter safe_zone.");
    return false;
  }

  if (!FilterBase::getParam(std::string("radius"), radius)) {
    ROS_ERROR("SDF2D filter did not find parameter radius.");
    return false;
  }

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("SDF2D filter did not find parameter `input_layer`.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("SDF2D filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("SDF2D output layer = %s.", outputLayer_.c_str());

  return true;
}

bool SDF2D::update(const GridMap &mapIn, GridMap &mapOut) {

  mapOut = mapIn;
  if (!mapOut.exists(outputLayer_)) {
    mapOut.add(outputLayer_);
  }

  Matrix &inputMap{mapOut[inputLayer_]}; // copy by value to do filtering first.
  Matrix &outputMap{mapOut[outputLayer_]};

  grid_map::Size gridMapSize = mapOut.getSize();

  // Set number of thread to use for parallel programming.
  std::unique_ptr<tbb::task_scheduler_init> TBBInitPtr;
  TBBInitPtr.reset(new tbb::task_scheduler_init(THREAD_MAX_MUM));
  // Parallelized iteration through the map.
  tbb::parallel_for(0, gridMapSize(0) * gridMapSize(1), [&](int range) {
    // Recover Cell index from range iterator.
    const Index index(range / gridMapSize(1), range % gridMapSize(1));
    compute(inputMap, outputMap, mapOut, index);
  });

  mapOut.setBasicLayers({});
  return true;
}

void SDF2D::compute(Matrix &inputMap, Matrix &outputMap, GridMap &mapOut,
                    const grid_map::Index &index) {
  Position center;

  mapOut.getPosition(index, center);

  double min_distance = 2.0 * radius; // Default large number

  for (grid_map::CircleIterator submapIterator(mapOut, center, radius);
       !submapIterator.isPastEnd(); ++submapIterator) {
    const Index index_sub(*submapIterator);
    const auto &inputValue_sub{inputMap(index_sub(0), index_sub(1))};
    Position pos_sub;
    mapOut.getPosition(index_sub, pos_sub);

    if (inputValue_sub < threshold ||
        !std::isfinite(
            inputValue_sub)) { // Consider unknown values are also dangerous
      double distance = std::sqrt(std::pow(pos_sub.x() - center.x(), 2) +
                                  std::pow(pos_sub.y() - center.y(), 2));
      if (min_distance > distance) {
        min_distance = distance;
      }
    }
  }

  if (min_distance < safe_zone)
    mapOut.at(outputLayer_, index) = 0;
  else
    mapOut.at(outputLayer_, index) = min_distance;
}
} // namespace strelka_elevation
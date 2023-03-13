#include "strelka_elevation/ConstInpaint.hpp"

#include <cmath>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

#include <tbb/task_scheduler_init.h>
#include <tbb/tbb.h>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace filters;

namespace strelka_elevation {

ConstInpaint::ConstInpaint() {}

ConstInpaint::~ConstInpaint() = default;

bool ConstInpaint::configure() {
  if (!FilterBase::getParam(std::string("fill_value"), fill_value)) {
    ROS_ERROR("ConstInpaint filter did not find parameter fill_value.");
    return false;
  }

  if (!FilterBase::getParam(std::string("radius"), radius)) {
    ROS_ERROR("ConstInpaint filter did not find parameter radius.");
    return false;
  }

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("ConstInpaint filter did not find parameter `input_layer`.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("ConstInpaint filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("ConstInpaint output layer = %s.", outputLayer_.c_str());

  return true;
}

bool ConstInpaint::update(const GridMap &mapIn, GridMap &mapOut) {
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

void ConstInpaint::compute(Matrix &inputMap, Matrix &outputMap, GridMap &mapOut,
                           const grid_map::Index &index) {
  const auto &inputValue{inputMap(index(0), index(1))};
  auto &outputValue{outputMap(index(0), index(1))};

  if (std::isfinite(inputValue)) {
    outputValue = inputValue;
  } else {
    double valueMax = fill_value;
    Eigen::Vector2d center;
    mapOut.getPosition(index, center);
    for (grid_map::CircleIterator submapIterator(mapOut, center, radius);
         !submapIterator.isPastEnd(); ++submapIterator) {
      const Index index_sub(*submapIterator);
      const auto &inputValue_sub{inputMap(index_sub(0), index_sub(1))};

      if (std::isfinite(inputValue_sub)) {
        if (inputValue_sub > valueMax)
          valueMax = inputValue_sub;
      }
    }
    outputValue = valueMax;
  }
}

} // namespace strelka_elevation
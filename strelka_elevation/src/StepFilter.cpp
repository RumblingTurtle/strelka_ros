#include "strelka_elevation/StepFilter.hpp"

#include <cmath>

// Grid Map
#include <grid_map_core/grid_map_core.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace filters;

namespace strelka_elevation {

StepFilter::StepFilter() {}

StepFilter::~StepFilter() = default;

bool StepFilter::configure() {
  if (!FilterBase::getParam(std::string("step_heights"), step_heights_)) {
    ROS_ERROR("Step filter did not find parameter fill_hole_radius.");
    return false;
  }

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("Step filter did not find parameter `input_layer`.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("Step filter did not find parameter `output_layer`.");
    return false;
  }

  ROS_DEBUG("Step output layer = %s.", outputLayer_.c_str());

  return true;
}

bool StepFilter::update(const GridMap &mapIn, GridMap &mapOut) {

  mapOut = mapIn;
  if (!mapOut.exists(outputLayer_)) {
    mapOut.add(outputLayer_);
  }
  int step_count = step_heights_.size();
  mapOut.convertToDefaultStartIndex();

  Matrix inputMap{mapOut[inputLayer_]}; // copy by value to do filtering first.
  Matrix &outputMap{mapOut[outputLayer_]};

  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    const auto &inputValue{inputMap(index(0), index(1))};
    auto &outputValue{outputMap(index(0), index(1))};

    if (inputValue > step_heights_[step_count - 1]) {
      outputValue = step_heights_[step_count - 1];
      continue;
    }

    if (inputValue <= step_heights_[0]) {
      outputValue = step_heights_[0];
      continue;
    }

    for (int i = 0; i < step_count - 1; i++) {

      if (inputValue > step_heights_[i] && inputValue <= step_heights_[i + 1]) {

        if (abs(inputValue - step_heights_[i]) <
            abs(inputValue - step_heights_[i + 1])) {
          outputValue = step_heights_[i];
        } else {
          outputValue = step_heights_[i + 1];
        }
        break;
      }
    }
  }
  mapOut.setBasicLayers({});
  return true;
}

} // namespace strelka_elevation
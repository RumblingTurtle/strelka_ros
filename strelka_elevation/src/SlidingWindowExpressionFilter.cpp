#include "strelka_elevation/SlidingWindowExpressionFilter.hpp"

using namespace filters;
using namespace grid_map;
namespace strelka_elevation {

SlidingWindowExpressionFilter::SlidingWindowExpressionFilter()
    : windowSize_(3), useWindowLength_(false), windowLength_(0.0),
      isComputeEmptyCells_(true),
      edgeHandling_(SlidingWindowIterator::EdgeHandling::INSIDE), clamp_(false),
      normalize_(false) {}

SlidingWindowExpressionFilter::~SlidingWindowExpressionFilter() = default;

bool SlidingWindowExpressionFilter::configure() {
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("SlidingWindowExpressionFilter did not find parameter "
              "'input_layer'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("SlidingWindowExpressionFilter did not find parameter "
              "'output_layer'.");
    return false;
  }

  double buf;
  clamp_ = FilterBase::getParam(std::string("clamp_value"), buf);
  normalize_ =
      FilterBase::getParam(std::string("normalize"), normalize_) && clamp_;

  if (clamp_)
    clamp_value_ = buf;

  if (!FilterBase::getParam(std::string("expression"), expression_)) {
    ROS_ERROR("SlidingWindowExpressionFilter did not find parameter "
              "'expression'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("window_size"), windowSize_)) {
    if (FilterBase::getParam(std::string("window_length"), windowLength_)) {
      useWindowLength_ = true;
    }
  }

  if (!FilterBase::getParam(std::string("compute_empty_cells"),
                            isComputeEmptyCells_)) {
    ROS_ERROR("SlidingWindowExpressionFilter did not find parameter "
              "'compute_empty_cells'.");
    return false;
  }

  std::string edgeHandlingMethod;
  if (!FilterBase::getParam(std::string("edge_handling"), edgeHandlingMethod)) {
    ROS_ERROR("SlidingWindowExpressionFilter did not find parameter "
              "'edge_handling'.");
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
    ROS_ERROR("SlidingWindowExpressionFilter did not find method '%s' for "
              "edge handling.",
              edgeHandlingMethod.c_str());
    return false;
  }

  // TODO(magnus): Can we make caching work with changing shared variable?
  //  parser_.setCacheExpressions(true);
  return true;
}

bool SlidingWindowExpressionFilter::update(const GridMap &mapIn,
                                           GridMap &mapOut) {

  grid_map::GridMap mapInCopy = mapIn; // copy map since given mapIn is const so
                                       // startIndex can't be changed
  mapInCopy.convertToDefaultStartIndex();
  mapOut = mapInCopy;

  if (!mapOut.exists(outputLayer_)) {
    mapOut.add(outputLayer_);
  }

  Matrix &outputData = mapOut[outputLayer_];
  grid_map::SlidingWindowIterator iterator(mapInCopy, inputLayer_,
                                           edgeHandling_, windowSize_);
  if (useWindowLength_)
    iterator.setWindowLength(mapInCopy, windowLength_);
  iterator.setWindowLength(mapInCopy, windowLength_);

  for (; !iterator.isPastEnd(); ++iterator) {
    parser_.var(inputLayer_).setLocal(iterator.getData());
    EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
    if (result.matrix().cols() == 1 && result.matrix().rows() == 1) {
      Eigen::Map<Eigen::MatrixXf> &mat = result.matrix();
      float res = mat(0);
      if (clamp_) {
        res = std::min(res, clamp_value_);
        if (normalize_) {
          res /= clamp_value_;
        }
      }

      outputData(iterator.getLinearIndex()) = res;
    } else {
      ROS_ERROR("SlidingWindowMathExpressionFilter could not apply filter "
                "because expression has to result in a scalar!");
    }
  }
  return true;
}

} // namespace strelka_elevation

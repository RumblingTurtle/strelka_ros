#include "strelka_elevation/MathExpressionFilter.hpp"

using namespace filters;
using namespace grid_map;

namespace strelka_elevation {

MathExpressionFilter::MathExpressionFilter()
    : clamp_(false), normalize_(false){};

MathExpressionFilter::~MathExpressionFilter() = default;

bool MathExpressionFilter::configure() {
  if (!FilterBase::getParam(std::string("expression"), expression_)) {
    ROS_ERROR("MathExpressionFilter did not find parameter 'expression'.");
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("MathExpressionFilter did not find parameter 'output_layer'.");
    return false;
  }

  double buf;
  clamp_ = FilterBase::getParam(std::string("clamp_value"), buf);
  normalize_ =
      FilterBase::getParam(std::string("normalize"), normalize_) && clamp_;

  if (clamp_)
    clamp_value_ = buf;

  // TODO Can we make caching work with changing shared variable?
  //  parser_.setCacheExpressions(true);
  return true;
}

bool MathExpressionFilter::update(const GridMap &mapIn, GridMap &mapOut) {
  mapOut = mapIn;
  if (!mapOut.exists(outputLayer_)) {
    mapOut.add(outputLayer_);
  }

  for (const auto &layer : mapOut.getLayers()) {
    parser_.var(layer).setShared(mapOut[layer]);
  }

  Matrix &outputData = mapOut[outputLayer_];
  EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
  for (int i = 0; i < result.matrix().rows(); i++) {
    for (int j = 0; j < result.matrix().cols(); j++) {
      Index idx{i, j};
      float res = result.matrix().row(i)(j);
      if (clamp_) {
        res = std::min(res, clamp_value_);
        if (normalize_) {
          res /= clamp_value_;
        }
      }
      mapOut.at(outputLayer_, idx) = res;
    }
  }
  return true;
}

} // namespace strelka_elevation
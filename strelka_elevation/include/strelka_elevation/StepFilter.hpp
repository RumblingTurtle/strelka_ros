#pragma once

#include <Eigen/Core>
#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <opencv2/core.hpp>

using namespace grid_map;
namespace strelka_elevation {

class StepFilter : public filters::FilterBase<GridMap> {
public:
  StepFilter();

  ~StepFilter() override;

  bool configure() override;

  bool update(const GridMap &mapIn, GridMap &mapOut) override;

private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;
  std::vector<double> step_heights_;
};

} // namespace strelka_elevation

#pragma once

#include <Eigen/Core>
#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <opencv2/core.hpp>

using namespace grid_map;
namespace strelka_elevation {

class SDF2D : public filters::FilterBase<GridMap> {
public:
  SDF2D();

  ~SDF2D() override;

  bool configure() override;

  bool update(const GridMap &mapIn, GridMap &mapOut) override;
  void compute(Matrix &inputMap, Matrix &outputMap, GridMap &mapOut,
               const grid_map::Index &index);

private:
  //! Input layer name.
  std::string inputLayer_;

  //! Output layer name.
  std::string outputLayer_;

  double threshold;
  double safe_zone;
  double radius;
};

} // namespace strelka_elevation

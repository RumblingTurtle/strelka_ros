#include <pluginlib/class_list_macros.h>

#include "strelka_elevation/ConstInpaint.hpp"
#include "strelka_elevation/MathExpressionFilter.hpp"
#include "strelka_elevation/SDF2D.hpp"
#include "strelka_elevation/SlidingWindowExpressionFilter.hpp"
#include "strelka_elevation/StepFilter.hpp"

PLUGINLIB_EXPORT_CLASS(strelka_elevation::StepFilter,
                       filters::FilterBase<grid_map::GridMap>)

PLUGINLIB_EXPORT_CLASS(strelka_elevation::SlidingWindowExpressionFilter,
                       filters::FilterBase<grid_map::GridMap>)

PLUGINLIB_EXPORT_CLASS(strelka_elevation::ConstInpaint,
                       filters::FilterBase<grid_map::GridMap>)

PLUGINLIB_EXPORT_CLASS(strelka_elevation::SDF2D,
                       filters::FilterBase<grid_map::GridMap>)

PLUGINLIB_EXPORT_CLASS(strelka_elevation::MathExpressionFilter,
                       filters::FilterBase<grid_map::GridMap>)
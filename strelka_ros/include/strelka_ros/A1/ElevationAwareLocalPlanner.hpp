
#include <chrono>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <ros/ros.h>
#include <strelka/common/macros.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka_robots/A1/control/A1LocalPlanner.hpp>

#include <strelka_ros/ElevationAwareFootPlanner.hpp>

#include <strelka_messages/HighLevelCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/WbicCommand.hpp>

#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/constants.hpp>
using namespace strelka;
using namespace strelka::control;

class ElevationAwareLocalPlanner : public A1LocalPlanner {
  std::shared_ptr<ElevationAwareFootPlanner> _footPlanner;

public:
  ElevationAwareLocalPlanner(
      std::shared_ptr<ElevationAwareFootPlanner> footPlanner,
      ros::NodeHandle nh);
  void elevationMapCallback(const grid_map_msgs::GridMap::ConstPtr &map);
};
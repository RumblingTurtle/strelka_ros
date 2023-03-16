#include <chrono>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>

#include <strelka_ros/ElevationAwareFootPlanner.hpp>

#include <strelka_messages/HighLevelCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/WbicCommand.hpp>

#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/constants.hpp>

#include <strelka/common/macros.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka_robots/A1/control/A1LocalPlanner.hpp>

using namespace strelka;
using namespace strelka::control;

int main(int argc, char **argv) {
  using namespace strelka::control;
  ros::init(argc, argv, "a1_local_planner");

  ros::NodeHandle nh;

  std::shared_ptr<GaitScheduler> gaitScheduler =
      std::make_shared<GaitScheduler>(strelka::GAITS::TROT);

  std::shared_ptr<ElevationAwareFootPlanner> footPlanner =
      std::make_shared<ElevationAwareFootPlanner>(gaitScheduler, 0.15, nh);

  A1LocalPlanner planner{footPlanner};

  while (ros::ok()) {
    ros::spinOnce();
    planner.handle();
  };

  return 0;
}
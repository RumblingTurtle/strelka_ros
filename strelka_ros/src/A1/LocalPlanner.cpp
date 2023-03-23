#include <chrono>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>

#include <strelka_ros/A1/RvizVisualizer.hpp>
#include <strelka_ros/ElevationAwareFootPlanner.hpp>

#include <strelka_lcm_headers/WbicCommand.hpp>
#include <strelka_messages/HighLevelCommand.hpp>

#include <strelka/robots/A1/UnitreeA1.hpp>
#include <strelka/robots/A1/constants.hpp>

#include <strelka/common/macros.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/nodes/LocalPlannerNode.hpp>

using namespace strelka;
using namespace strelka::control;

int main(int argc, char **argv) {
  using namespace strelka::control;
  using namespace strelka::robots;

  ros::init(argc, argv, "a1_local_planner");

  ros::NodeHandle nh;

  std::shared_ptr<LocalPlannerNode<UnitreeA1>> planner;
  std::string gaitName;
  float searchRadius, stepDt, heightFreq, pitchFreq;
  int horizonSteps;
  bool blind, contFootUpdate;

  nh.param<std::string>("gait", gaitName, "trot");
  nh.param<bool>("blind", blind, true);
  nh.param<bool>("upd_footholds_continuously", contFootUpdate, false);
  nh.param<float>("foothold_search_radius", searchRadius, 0.1);

  nh.param<float>("mpc_step_dt", stepDt, 0.02);
  nh.param<int>("mpc_horizon_steps", horizonSteps, 15);

  nh.param<float>("height_lpf_freq", heightFreq, 30.0);
  nh.param<float>("pitch_lpf_freq", pitchFreq, 10.0);

  if (strelka::GAITS_MAP.count(gaitName) == 0) {
    ROS_ERROR_STREAM("LocalPlannerNode: Can't find gait named " << gaitName);
    return 1;
  }

  if (blind) {
    planner = std::make_shared<LocalPlannerNode<UnitreeA1>>(
        strelka::GAITS_MAP.at(gaitName), stepDt, horizonSteps, heightFreq,
        pitchFreq, contFootUpdate);
  } else {
    std::shared_ptr<GaitScheduler> gaitScheduler =
        std::make_shared<GaitScheduler>(strelka::GAITS_MAP.at(gaitName));

    std::shared_ptr<ElevationAwareFootPlanner> footPlanner =
        std::make_shared<ElevationAwareFootPlanner>(gaitScheduler, searchRadius,
                                                    contFootUpdate, nh);

    planner = std::make_shared<LocalPlannerNode<UnitreeA1>>(
        footPlanner, stepDt, horizonSteps, heightFreq, pitchFreq);
  }

  RvizVisualizer visualizer{nh};

  while (ros::ok()) {
    ros::spinOnce();
    planner->handle();
    visualizer.publishLocalPlannerInfo(planner);
  };

  return 0;
}
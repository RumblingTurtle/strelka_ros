#include <ros/ros.h>
#include <strelka_robots/A1/control/A1LocalPlanner.hpp>

int main(int argc, char **argv) {
  using namespace strelka::control;
  ros::init(argc, argv, "a1_local_planner");
  ros::NodeHandle nh;

  std::string gaitName;
  nh.param<std::string>("gait", gaitName, "trot");
  if (strelka::GAITS_MAP.count(gaitName) == 0) {
    ROS_INFO_STREAM("A1LocalPlanner: Can't find gait named " << gaitName);
    return 1;
  }
  A1LocalPlanner planner{strelka::GAITS_MAP.at(gaitName)};
  while (planner.handle() && ros::ok()) {
  }
  return 0;
}
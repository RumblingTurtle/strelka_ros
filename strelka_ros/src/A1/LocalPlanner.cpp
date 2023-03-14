#include <ros/ros.h>
#include <strelka_robots/A1/control/A1LocalPlanner.hpp>

int main(int argc, char **argv) {
  using namespace strelka::control;
  ros::init(argc, argv, "a1_local_planner");

  A1LocalPlanner planner{strelka::GAITS::TROT};
  while (planner.handle() && ros::ok()) {
  }
  return 0;
}
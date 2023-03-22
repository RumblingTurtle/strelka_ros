#include <strelka/nodes/WBICNode.hpp>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_wbic");
  using namespace strelka;
  using namespace strelka::control;

  WBICNode<robots::UnitreeA1> wbicController{
      robots::UnitreeA1::createDummyA1RobotWithRawState(), DEFAULT_WBIC_PARAMS};
  while (wbicController.handle() && ros::ok()) {
  }
  return 0;
}
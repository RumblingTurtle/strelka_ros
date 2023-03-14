#include <strelka_robots/A1/control/A1WBIC.hpp>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_wbic");
  using namespace strelka;
  using namespace strelka::control;

  A1WBIC wbicController{DEFAULT_WBIC_PARAMS};
  while (wbicController.handle() && ros::ok()) {
  }
  return 0;
}
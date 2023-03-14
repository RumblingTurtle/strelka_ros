#include <strelka_robots/A1/interfaces/A1GazeboInterface.hpp>
#include <strelka_robots/A1/state_estimation/A1StateEstimator.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_state_estimator");
  ros::NodeHandle nh;

  using namespace strelka::interfaces;
  using namespace strelka::state_estimation;

  A1GazeboInterface interface;
  interface.moveToInit();

  std_srvs::Empty resetObj;
  ros::service::call("/gazebo/reset_world", resetObj);

  ros::Duration(1.0).sleep();

  interface.moveToStand();

  A1StateEstimator estimator{};

  while (estimator.handle() && ros::ok()) {
  }

  interface.moveToInit();
  return 0;
}
#include <strelka/nodes/MoveToInterface.hpp>
#include <strelka/nodes/StateEstimatorNode.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_state_estimator");
  ros::NodeHandle nh;

  using namespace strelka::interfaces;
  using namespace strelka::state_estimation;
  using namespace strelka::robots;

  UnitreeA1 dummyRobot = UnitreeA1::createDummyA1RobotWithRawState();

  MoveToInterface<UnitreeA1> interface {
    dummyRobot
  };

  interface.moveToInit();

  std_srvs::Empty resetObj;
  ros::service::call("/gazebo/reset_world", resetObj);

  ros::Duration(1.0).sleep();

  interface.moveToStand();

  StateEstimatorNode<UnitreeA1> estimator{};

  while (estimator.handle() && ros::ok()) {
  }

  interface.moveToInit();
  return 0;
}
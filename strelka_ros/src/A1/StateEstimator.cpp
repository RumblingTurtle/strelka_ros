#include <strelka/nodes/MoveToInterface.hpp>
#include <strelka/nodes/StateEstimatorNode.hpp>
#include <strelka_ros/A1/CheaterEstimator.hpp>
#include <strelka_ros/SimSlowdown.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_state_estimator");
  ros::NodeHandle nh;
  using namespace strelka::interfaces;
  using namespace strelka::state_estimation;
  using namespace strelka::robots;

  slowdownGazebo(1.0);

  MoveToInterface<UnitreeA1> interface {};
  interface.moveToInit();

  std_srvs::Empty resetObj;
  ros::service::call("/gazebo/reset_world", resetObj);
  ros::Duration(1.0).sleep();

  interface.moveToStand();

  float sim_slowdown;
  bool perfect_odometry;

  nh.param<bool>("/common/perfect_odometry", perfect_odometry, false);
  nh.param<float>("/common/sim_slowdown", sim_slowdown, 1.0f);

  slowdownGazebo(sim_slowdown);

  if (perfect_odometry) {
    A1CheaterEstimator estimator{};
    while (estimator.handle() && ros::ok()) {
    }

  } else {
    StateEstimatorNode<UnitreeA1> estimator{};
    while (estimator.handle() && ros::ok()) {
    }
  }

  slowdownGazebo(1.0f);

  interface.moveToInit();
  return 0;
}
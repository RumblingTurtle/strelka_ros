#include <strelka_ros/A1/A1CheaterEstimator.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <strelka_robots/A1/interfaces/A1GazeboInterface.hpp>

A1CheaterEstimator::A1CheaterEstimator() : gazeboStateRecieved(false) {
  robotStateMsg = new a1_lcm_msgs::RobotState();
  robotGazeboMsg = new a1_lcm_msgs::RobotGazeboState();

  subGazebo = lcm.subscribe(strelka::A1::constants::GAZEBO_STATE_TOPIC_NAME,
                            &A1CheaterEstimator::updateGazebo, this);
  subRawState = lcm.subscribe(strelka::A1::constants::RAW_STATE_TOPIC_NAME,
                              &A1CheaterEstimator::update, this);
  subGazebo->setQueueCapacity(1);
  subRawState->setQueueCapacity(1);
}

void A1CheaterEstimator::processLoop() {
  while (handle()) {
  }
}

bool A1CheaterEstimator::handle() {
  if (0 != lcm.handle()) {
    std::cout << "A1CheaterEstimator: lcm.handle() nonzero return value"
              << std::endl;
    return false;
  }

  return true;
}

A1CheaterEstimator::~A1CheaterEstimator() {
  delete robotStateMsg;
  delete robotGazeboMsg;
  if (subRawState) {
    lcm.unsubscribe(subRawState);
  }

  if (subGazebo) {
    lcm.unsubscribe(subGazebo);
  }
}

void A1CheaterEstimator::propagateRobotRawState(
    const a1_lcm_msgs::RobotRawState *messageIn,
    a1_lcm_msgs::RobotState *messageOut) {
  memcpy(messageOut->quaternion, messageIn->quaternion, sizeof(float) * 4);
  memcpy(messageOut->gyro, messageIn->gyro, sizeof(float) * 3);
  memcpy(messageOut->accel, messageIn->accel, sizeof(float) * 3);
  memcpy(messageOut->footForces, messageIn->footForces, sizeof(float) * 4);
  memcpy(messageOut->q, messageIn->q, sizeof(float) * 12);
  memcpy(messageOut->dq, messageIn->dq, sizeof(float) * 12);
  messageOut->tick = messageIn->tick;
}

void A1CheaterEstimator::updateGazebo(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::RobotGazeboState *messageIn) {
  memcpy(robotGazeboMsg, messageIn, sizeof(a1_lcm_msgs::RobotGazeboState));
  if (!gazeboStateRecieved) {
    gazeboStateRecieved = true;
  }
}

void A1CheaterEstimator::update(const lcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const a1_lcm_msgs::RobotRawState *messageIn) {
  if (!gazeboStateRecieved) {
    return;
  }

  strelka::robots::UnitreeA1 robot(messageIn);
  propagateRobotRawState(messageIn, robotStateMsg);

  memcpy(robotStateMsg->position, robotGazeboMsg->position_world,
         sizeof(float) * 3);

  Vec3<float> velocityBody = robot.rotateWorldToBodyFrame(
      Eigen::Map<Vec3<float>>(robotGazeboMsg->velocity_world, 3));

  memcpy(robotStateMsg->velocityBody, velocityBody.data(), sizeof(float) * 3);

  Vec12<float> footPosititons;
  FOR_EACH_LEG {
    footPosititons.block<3, 1>(LEG_ID * 3, 0) =
        robot.footPositionTrunkFrame(LEG_ID);
  }

  memcpy(robotStateMsg->footPositions, footPosititons.data(),
         sizeof(float) * 12);
  memcpy(robotStateMsg->jacobians, robot.footJacobians().data(),
         sizeof(float) * 36);

  lcm.publish(strelka::A1::constants::ROBOT_STATE_TOPIC_NAME, robotStateMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_state_estimator");
  ros::NodeHandle nh;

  strelka::interfaces::A1GazeboInterface interface;
  interface.moveToInit();

  std_srvs::Empty resetObj;
  ros::service::call("/gazebo/reset_world", resetObj);

  ros::Duration(1.0).sleep();

  interface.moveToStand();

  A1CheaterEstimator estimator{};

  while (estimator.handle() && ros::ok()) {
  }

  interface.moveToInit();
  return 0;
}
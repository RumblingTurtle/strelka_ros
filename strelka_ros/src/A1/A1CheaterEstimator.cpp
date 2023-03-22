#include <strelka_ros/A1/A1CheaterEstimator.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <strelka/nodes/MoveToInterface.hpp>
#include <tf/transform_listener.h>

A1CheaterEstimator::A1CheaterEstimator()
    : gazeboStateRecieved(false), trunkToFootZOffset(0) {
  robotStateMsg = new strelka_lcm_headers::RobotState();
  robotGazeboMsg = new strelka_lcm_headers::RobotGazeboState();
  zeroOffset = Vec3<float>::Zero();
  subGazebo = lcm.subscribe(strelka::constants::GAZEBO_STATE_TOPIC_NAME,
                            &A1CheaterEstimator::updateGazebo, this);
  subRawState = lcm.subscribe(strelka::constants::RAW_STATE_TOPIC_NAME,
                              &A1CheaterEstimator::update, this);
  subGazebo->setQueueCapacity(1);
  subRawState->setQueueCapacity(1);
  setupZOffset();
  handle();
}

void A1CheaterEstimator::setupZOffset() {
  bool transformRecieved = false;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  while (ros::ok() && !transformRecieved) {
    try {
      listener.lookupTransform("/trunk", "/FR_foot", ros::Time(0), transform);
      transformRecieved = true;
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }
  }

  trunkToFootZOffset =
      -transform.getOrigin().z() + strelka::A1::constants::FOOT_RADIUS;
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
    const strelka_lcm_headers::RobotRawState *messageIn,
    strelka_lcm_headers::RobotState *messageOut) {
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
    const strelka_lcm_headers::RobotGazeboState *messageIn) {
  memcpy(robotGazeboMsg, messageIn,
         sizeof(strelka_lcm_headers::RobotGazeboState));
  if (!gazeboStateRecieved) {
    gazeboStateRecieved = true;
    zeroOffset = Eigen::Map<const Vec3<float>>(messageIn->position_world, 3);
    // Approximate height from trunk to feet
    zeroOffset(2) -= trunkToFootZOffset;
  }
}

void A1CheaterEstimator::update(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotRawState *messageIn) {
  if (!gazeboStateRecieved) {
    return;
  }

  strelka::robots::UnitreeA1 robot(messageIn);
  propagateRobotRawState(messageIn, robotStateMsg);

  Vec3<float> positionWorld =
      Eigen::Map<const Vec3<float>>(robotGazeboMsg->position_world, 3) -
      zeroOffset;

  memcpy(robotStateMsg->position, positionWorld.data(), sizeof(float) * 3);

  Vec3<float> velocityBody = robot.rotateWorldToBodyFrame(
      Eigen::Map<const Vec3<float>>(robotGazeboMsg->velocity_world, 3));

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

  lcm.publish(strelka::constants::ROBOT_STATE_TOPIC_NAME, robotStateMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_state_estimator");
  ros::NodeHandle nh;
  using namespace strelka::interfaces;
  using namespace strelka::robots;

  MoveToInterface<UnitreeA1> interface {};
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
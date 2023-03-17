#ifndef A1_CHEATER_STATE_ESITMATOR_H
#define A1_CHEATER_STATE_ESITMATOR_H

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <strelka/common/constants.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotGazeboState.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotRawState.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotState.hpp>
#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/constants.hpp>
#include <strelka_robots/A1/kinematics.hpp>

class A1CheaterEstimator {
  lcm::LCM lcm;
  a1_lcm_msgs::RobotState *robotStateMsg;
  a1_lcm_msgs::RobotGazeboState *robotGazeboMsg;

  lcm::Subscription *subRawState;
  lcm::Subscription *subGazebo;
  float trunkToFootZOffset;
  Vec3<float> zeroOffset;
  bool gazeboStateRecieved;

  void update(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const a1_lcm_msgs::RobotRawState *messageIn);
  void updateGazebo(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const a1_lcm_msgs::RobotGazeboState *messageIn);

  void propagateRobotRawState(const a1_lcm_msgs::RobotRawState *messageIn,
                              a1_lcm_msgs::RobotState *messageOut);

public:
  A1CheaterEstimator();
  void setupZOffset();
  void processLoop();
  bool handle();
  ~A1CheaterEstimator();
};

#endif // A1_CHEATER_STATE_ESITMATOR_H
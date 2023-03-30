#ifndef A1_CHEATER_STATE_ESITMATOR_H
#define A1_CHEATER_STATE_ESITMATOR_H

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <strelka/common/constants.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/robots/A1/UnitreeA1.hpp>
#include <strelka/robots/A1/constants.hpp>
#include <strelka/robots/A1/kinematics.hpp>
#include <strelka_lcm_headers/RobotGazeboState.hpp>
#include <strelka_lcm_headers/RobotRawState.hpp>
#include <strelka_lcm_headers/RobotState.hpp>

class A1CheaterEstimator {
  lcm::LCM lcm;
  const char *const GAZEBO_STATE_TOPIC_NAME = "gazebo_state";
  strelka_lcm_headers::RobotState *robotStateMsg;
  strelka_lcm_headers::RobotGazeboState *robotGazeboMsg;

  lcm::Subscription *subRawState;
  lcm::Subscription *subGazebo;
  float trunkToFootZOffset;
  Vec3<float> zeroOffset;
  bool gazeboStateRecieved;

  void update(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const strelka_lcm_headers::RobotRawState *messageIn);
  void updateGazebo(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                    const strelka_lcm_headers::RobotGazeboState *messageIn);

  void
  propagateRobotRawState(const strelka_lcm_headers::RobotRawState *messageIn,
                         strelka_lcm_headers::RobotState *messageOut);

public:
  A1CheaterEstimator();
  void setupZOffset();
  void processLoop();
  bool handle();
  ~A1CheaterEstimator();
};

#endif // A1_CHEATER_STATE_ESITMATOR_H
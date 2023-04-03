#include <iostream>
#include <signal.h>
#include <unistd.h>

#include <lcm/lcm-cpp.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <strelka/common/constants.hpp>
#include <strelka_lcm_headers/HighLevelCommand.hpp>
#include <strelka_messages/HighLevelCommand.hpp>

void sigHandler(int s) {
  std::cout << "Stopped high level command publisher" << std::endl;
  exit(1);
}

void setupKeyboardInterrupt() {
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = sigHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

class TwistHandler {
public:
  float velocityX;
  float velocityY;
  float velocityYaw;
  void handle(const geometry_msgs::Twist &twist) {
    velocityX = twist.linear.x;
    velocityY = twist.linear.y;
    velocityYaw = twist.angular.z;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_high_level_command_publisher");
  ros::NodeHandle nh;

  using namespace strelka::messages;

  lcm::LCM lcm;

  if (!lcm.good()) {
    return 1;
  }

  bool useMoveBase;
  float footHeight, bodyHeight, velocityX, velocityY, velocityYaw;
  nh.param("/high_command/velocity_x", velocityX, 0.0f);
  nh.param("/high_command/velocity_y", velocityY, 0.0f);
  nh.param("/high_command/velocity_yaw", velocityYaw, 0.0f);
  nh.param("/high_command/foot_height", footHeight, 0.08f);
  nh.param("/high_command/body_height", bodyHeight, 0.26f);
  nh.param("/high_command/use_move_base", useMoveBase, false);

  strelka_lcm_headers::HighLevelCommand highCommandMsg{
      .linearSpeed = {velocityX, velocityY, 0.0},
      .angularVelocity = {0, 0, velocityYaw},
      .footHeight = footHeight,
      .footClearance = 0.002,
      .hipOffsets = {0, 0},
      .rpy = {0, 0, 0},
      .comOffset = {0, 0},
      .bodyHeight = bodyHeight,
      .stop = false};

  setupKeyboardInterrupt();

  ros::Subscriber sub;
  TwistHandler handler;
  if (useMoveBase) {
    velocityX = 0.0f;
    velocityY = 0.0f;
    velocityYaw = 0.0f;
    sub = nh.subscribe("/cmd_vel", 100, &TwistHandler::handle, &handler);
  }

  while (ros::ok()) {
    if (useMoveBase) {
      ros::spinOnce();
      highCommandMsg.linearSpeed[0] = handler.velocityX;
      highCommandMsg.linearSpeed[1] = handler.velocityY;
      highCommandMsg.angularVelocity[2] = handler.velocityYaw;
    }
    lcm.publish(strelka::constants::HIGH_LEVEL_COMMAND_TOPIC_NAME,
                &highCommandMsg);
    usleep(10000);
  }
  return 0;
}
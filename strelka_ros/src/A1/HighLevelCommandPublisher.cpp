#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <strelka_messages/HighLevelCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/HighLevelCommand.hpp>
#include <strelka_robots/A1/constants.hpp>
#include <unistd.h>

#include <ros/ros.h>

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_high_level_command_publisher");
  ros::NodeHandle nh;

  using namespace strelka::messages;

  lcm::LCM lcm;

  if (!lcm.good()) {
    return 1;
  }

  float velocityX, velocityY, velocityYaw, footHeight, bodyHeight;

  nh.param("velocityX", velocityX, 0.0f);
  nh.param("velocityY", velocityY, 0.0f);
  nh.param("velocityYaw", velocityYaw, 0.0f);
  nh.param("footHeight", footHeight, 0.08f);
  nh.param("bodyHeight", bodyHeight, 0.26f);

  a1_lcm_msgs::HighLevelCommand highCommandMsg{
      .linearSpeed = {velocityX, velocityY, 0.0},
      .angularVelocity = {0, 0, velocityYaw},
      .footHeight = 0.08,
      .footClearance = 0.002,
      .hipOffsets = {0, 0},
      .rpy = {0, 0, 0},
      .comOffset = {0, 0},
      .bodyHeight = 0.26,
      .stop = false};

  setupKeyboardInterrupt();
  while (ros::ok()) {
    lcm.publish(strelka::A1::constants::HIGH_LEVEL_COMMAND_TOPIC_NAME,
                &highCommandMsg);
    usleep(10000);
  }
  return 0;
}
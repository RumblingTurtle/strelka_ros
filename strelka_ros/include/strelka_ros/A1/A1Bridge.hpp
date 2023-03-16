#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <lcm/lcm-cpp.hpp>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <strelka_messages/a1_lcm_msgs/RobotGazeboState.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotLowCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/RobotRawState.hpp>
#include <strelka_robots/A1/constants.hpp>
#include <string>
#include <thread>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <vector>

static const std::vector<std::string> legInitials = {"FR", "FL", "RR", "RL"};

class LcmHandler {

  ros::NodeHandle nm;
  ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, gazebo_sub;
  ros::Publisher servo_pub[12];
  std::vector<float> foot_forces;
  std::string robot_name;
  std::thread thrd;
  ros::Time init_time;

public:
  a1_lcm_msgs::RobotRawState state;
  a1_lcm_msgs::RobotGazeboState gaz_state;
  bool runThread = true;
  lcm::LCM lcm;
  LcmHandler(const char *rname) : robot_name(rname) {
    foot_forces.resize(12);

    gazebo_sub = nm.subscribe("/gazebo/model_states", 1,
                              &LcmHandler::gazeboModelCallback, this);
    imu_sub = nm.subscribe("/trunk_imu", 1000, &LcmHandler::imuCallback, this);

    defineSubscriptions<0>();
    defineSubscriptions<1>();
    defineSubscriptions<2>();
    defineSubscriptions<3>();

    thrd = std::thread([this] { this->commandThread(); });
    init_time = ros::Time::now();
  }

  template <int legId> void defineSubscriptions() {
    footForce_sub[legId] = nm.subscribe(
        "/visual/" + legInitials[legId] + "_foot_contact/the_force", 1,
        &LcmHandler::footCallback<legId>, this);

    servo_sub[3 * legId] =
        nm.subscribe("/" + robot_name + "_gazebo/" + legInitials[legId] +
                         "_hip_controller/state",
                     1, &LcmHandler::motorCallback<3 * legId>, this);

    servo_sub[3 * legId + 1] =
        nm.subscribe("/" + robot_name + "_gazebo/" + legInitials[legId] +
                         "_thigh_controller/state",
                     1, &LcmHandler::motorCallback<3 * legId + 1>, this);

    servo_sub[3 * legId + 2] =
        nm.subscribe("/" + robot_name + "_gazebo/" + legInitials[legId] +
                         "_calf_controller/state",
                     1, &LcmHandler::motorCallback<3 * legId + 2>, this);

    servo_pub[3 * legId] = nm.advertise<unitree_legged_msgs::MotorCmd>(
        "/" + robot_name + "_gazebo/" + legInitials[legId] +
            "_hip_controller/command",
        1);
    servo_pub[3 * legId + 1] = nm.advertise<unitree_legged_msgs::MotorCmd>(
        "/" + robot_name + "_gazebo/" + legInitials[legId] +
            "_thigh_controller/command",
        1);
    servo_pub[3 * legId + 2] = nm.advertise<unitree_legged_msgs::MotorCmd>(
        "/" + robot_name + "_gazebo/" + legInitials[legId] +
            "_calf_controller/command",
        1);
  }

  ~LcmHandler() {
    runThread = false;
    thrd.join();
  }

  void gazeboModelCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    int robot_idx = -1;
    for (int i = 0; i < msg->name.size(); i++) {
      if (msg->name[i] == "a1_gazebo") {
        robot_idx = i;
        break;
      }
    }

    if (robot_idx != -1) {
      state.position[0] = msg->pose[robot_idx].position.x;
      state.position[1] = msg->pose[robot_idx].position.y;
      state.position[2] = msg->pose[robot_idx].position.z;

      state.velocity[0] = msg->twist[robot_idx].linear.x;
      state.velocity[1] = msg->twist[robot_idx].linear.y;
      state.velocity[2] = msg->twist[robot_idx].linear.z;

      gaz_state.position_world[0] = msg->pose[robot_idx].position.x;
      gaz_state.position_world[1] = msg->pose[robot_idx].position.y;
      gaz_state.position_world[2] = msg->pose[robot_idx].position.z;

      gaz_state.quaternion[1] = msg->pose[robot_idx].orientation.x;
      gaz_state.quaternion[2] = msg->pose[robot_idx].orientation.y;
      gaz_state.quaternion[3] = msg->pose[robot_idx].orientation.z;
      gaz_state.quaternion[0] = msg->pose[robot_idx].orientation.w;

      gaz_state.velocity_world[0] = msg->twist[robot_idx].linear.x;
      gaz_state.velocity_world[1] = msg->twist[robot_idx].linear.y;
      gaz_state.velocity_world[2] = msg->twist[robot_idx].linear.z;

      for (int i; i < 12; i++) {
        gaz_state.foot_forces[i] = foot_forces[i];
      }

      lcm.publish("gazebo_state", &gaz_state);
      ros::Duration(0.002).sleep();
    } else {
      ROS_INFO("could not find robot a1_gazebo in gazebo/model_states");
    }
  }

  void imuCallback(const sensor_msgs::Imu &msg) {
    state.quaternion[0] = msg.orientation.w;
    state.quaternion[1] = msg.orientation.x;
    state.quaternion[2] = msg.orientation.y;
    state.quaternion[3] = msg.orientation.z;

    state.gyro[0] = msg.angular_velocity.x;
    state.gyro[1] = msg.angular_velocity.y;
    state.gyro[2] = msg.angular_velocity.z;

    state.accel[0] = msg.linear_acceleration.x;
    state.accel[1] = msg.linear_acceleration.y;
    state.accel[2] = msg.linear_acceleration.z;
    ros::Time current_time = ros::Time::now();
    ros::Duration tick = current_time - init_time;
    state.tick = tick.toSec();
  }

  template <size_t motor_num>
  void motorCallback(const unitree_legged_msgs::MotorState &msg) {
    state.q[motor_num] = msg.q;
    state.dq[motor_num] = msg.dq;
  }

  template <size_t foot_num>
  void footCallback(const geometry_msgs::WrenchStamped &msg) {
    float norm = sqrt(pow(msg.wrench.force.x, 2) + pow(msg.wrench.force.y, 2) +
                      pow(msg.wrench.force.z, 2));
    state.footForces[foot_num] = norm;
    foot_forces[foot_num * 3] = msg.wrench.force.x;
    foot_forces[foot_num * 3 + 1] = msg.wrench.force.y;
    foot_forces[foot_num * 3 + 2] = msg.wrench.force.z;
  }

  void commandThread() {
    lcm.subscribe("robot_low_command", &LcmHandler::commandCallback, this);

    while (runThread) {
      lcm.handle();
    }
  }

  void commandCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const a1_lcm_msgs::RobotLowCommand *msg) {
    unitree_legged_msgs::MotorCmd motor_cmd;
    for (int motorId = 0; motorId < 12; motorId++) {
      motor_cmd.mode = 0x0A;
      motor_cmd.q = msg->q[motorId];
      motor_cmd.Kp = msg->kp[motorId];
      motor_cmd.dq = msg->dq[motorId];
      motor_cmd.Kd = msg->kd[motorId];
      motor_cmd.tau = msg->tau[motorId];
      servo_pub[motorId].publish(motor_cmd);
    }
  }
};
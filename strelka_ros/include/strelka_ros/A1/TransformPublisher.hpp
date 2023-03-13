#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <strelka_messages/a1_lcm_msgs/RobotState.hpp>
#include <strelka_robots/A1/constants.hpp>
#include <tf/transform_broadcaster.h>

class Handler {
  ros::Publisher poseRepublisher;
  ros::NodeHandle nh;
  ros::Time lastTransformStamp;

public:
  Handler(ros::NodeHandle n) : nh(n) { lastTransformStamp = ros::Time::now(); }

  void handle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const a1_lcm_msgs::RobotState *data) {

    static tf::TransformBroadcaster br;

    ros::Time currentTransformTime = ros::Time::now();
    ros::Duration diff = currentTransformTime - lastTransformStamp;
    if (!diff.isZero()) {
      // Bypassing redundant tf timestamps in simulation
      tf::Transform transform;
      transform.setOrigin(
          tf::Vector3(data->position[0], data->position[1], data->position[2]));
      tf::Quaternion q{data->quaternion[1], data->quaternion[2],
                       data->quaternion[3], data->quaternion[0]};
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, currentTransformTime,
                                            "odom", "trunk"));
      lastTransformStamp = currentTransformTime;
    }
  }
};

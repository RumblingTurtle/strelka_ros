
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include <strelka/common/constants.hpp>
#include <strelka_lcm_headers/RobotState.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class TransformPublisher {
  ros::Publisher poseRepublisher;
  ros::NodeHandle nh;
  ros::Time lastTransformStamp;

public:
  TransformPublisher(ros::NodeHandle n) : nh(n) {
    lastTransformStamp = ros::Time::now();
  }

  void handle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
              const strelka_lcm_headers::RobotState *data) {

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
      // Trunk link cannot have two parents (base and odom) that is why we have
      // to parent base.
      br.sendTransform(tf::StampedTransform(transform, currentTransformTime,
                                            "odom", "base"));
      lastTransformStamp = currentTransformTime;
    }
  }
};

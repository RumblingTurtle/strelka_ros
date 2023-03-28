#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <memory>
#include <ros/ros.h>
#include <strelka/nodes/LocalPlannerNode.hpp>

class RvizVisualizer {
  ros::Publisher markerPub;
  static constexpr int TRAJECTORY_SAMPLING_STEPS = 10;
  int markerId = 0;

public:
  RvizVisualizer(ros::NodeHandle &nh) {
    markerPub = nh.advertise<visualization_msgs::MarkerArray>(
        "local_planner_markers", 1);
  }

  void publishLocalPlannerInfo(
      std::shared_ptr<
          strelka::control::LocalPlannerNode<strelka::robots::UnitreeA1>>
          plannerPtr) {

    using namespace strelka::control;
    using namespace strelka;
    using namespace strelka::robots;

    visualization_msgs::MarkerArray markers;
    markerId = 0;
    // Publish footholds
    LocalPlanner &planner = plannerPtr->getLocalPlanner();
    std::shared_ptr<FootholdPlanner> footPlanner = planner.getFootPlanner();
    std::shared_ptr<GaitScheduler> scheduler = planner.getGaitScheduler();
    UnitreeA1 &robot = plannerPtr->getRobotInstance();

    Vec12<float> mpcForces = planner.mpcForces() / 100;
    FOR_EACH_LEG {
      int footholdCount = footPlanner->footholdCount(LEG_ID);
      for (int fID = 0; fID < footholdCount; fID++) {
        Vec3<float> foothold = footPlanner->getFoothold(LEG_ID, fID);
        Vec3<float> mpcForce = mpcForces.block<3, 1>(LEG_ID * 3, 0);
        Vec3<float> currentFootPos = footPlanner->currentFootPosition(LEG_ID);

        markers.markers.push_back(
            createMarker(foothold, foothold, visualization_msgs::Marker::SPHERE,
                         Vec4<float>{1.0f, 0.0f, 0.0f, 1.0f}, 0.03f));

        std::string footStateText;
        switch (scheduler->currentLegState[LEG_ID]) {
        case strelka::LegState::SWING:
          footStateText = "SWING";
          break;
        case strelka::LegState::STANCE:
          footStateText = "STANCE";
          break;
        case strelka::LegState::EARLY_CONTACT:
          footStateText = "EARLY CONTACT";
          break;
        case strelka::LegState::LOST_CONTACT:
          footStateText = "LOST CONTACT";
          break;
        }

        Vec3<float> displayOffset{
            0, (1 * (LEG_ID % 2) - 1 * ((LEG_ID + 1) % 2)) * 0.1, 0};
        markers.markers.push_back(createMarker(
            currentFootPos + robot.rotateBodyToWorldFrame(displayOffset) +
                Vec3<float>{0, 0, 0.3f},
            currentFootPos, visualization_msgs::Marker::TEXT_VIEW_FACING,
            Vec4<float>{1.0f, 1.0f, 1.0f, 1.0f}, 0.04f, footStateText));

        if (planner.footState(LEG_ID)) {
          markers.markers.push_back(
              createMarker(currentFootPos, currentFootPos + mpcForce,
                           visualization_msgs::Marker::ARROW,
                           Vec4<float>{0.0f, 1.0f, 0.0f, 0.1f}, 0.02f));
        }

        if (fID < footholdCount - 1) {
          // Sample desired trajectory
          for (int step = 0; step < TRAJECTORY_SAMPLING_STEPS; step++) {
            float t = (float)step / TRAJECTORY_SAMPLING_STEPS;
            Vec3<float> trajectoryPoint =
                footPlanner->getDesiredTrajectoryPosition(
                    foothold, footPlanner->getFoothold(LEG_ID, fID + 1), t,
                    LEG_ID);

            markers.markers.push_back(
                createMarker(trajectoryPoint, trajectoryPoint,
                             visualization_msgs::Marker::SPHERE,
                             Vec4<float>{0.0f, 0.0f, 0.0f, 0.8f}, 0.01f));
          }
        }
      }
    }
    markerPub.publish(markers);
  }

  visualization_msgs::Marker createMarker(Vec3<float> pStart, Vec3<float> pEnd,
                                          uint32_t shape, Vec4<float> color,
                                          float scale,
                                          std::string markerText = " ") {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "local_planner_markers";
    marker.id = markerId;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    if (shape == visualization_msgs::Marker::ARROW) {
      marker.points.resize(2);
      marker.points[0].x = pStart(0);
      marker.points[0].y = pStart(1);
      marker.points[0].z = pStart(2);
      marker.points[1].x = pEnd(0);
      marker.points[1].y = pEnd(1);
      marker.points[1].z = pEnd(2);
    } else {
      marker.pose.position.x = pStart(0);
      marker.pose.position.y = pStart(1);
      marker.pose.position.z = pStart(2);

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
    }

    if (shape == visualization_msgs::Marker::TEXT_VIEW_FACING) {
      marker.text = markerText;
    }

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = color(3);

    marker.lifetime = ros::Duration();
    markerId++;
    return marker;
  }
};
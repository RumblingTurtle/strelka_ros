#include <ros/ros.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <cmath>

#include <strelka/control/FootholdPlanner.hpp>
#include <strelka/control/gait/GaitScheduler.hpp>

using namespace grid_map;
using namespace strelka;
using namespace strelka::control;

class ElevationAwareFootPlanner : public FootholdPlanner {
public:
  GridMap map;
  float _searchRadius;
  bool firstMapRecieved;
  ros::Subscriber mapSub;

  float maxCurvatureThreshold;
  float sdfThreshold;
  float heightDifferenceThreshold;
  float maxDistToNominal;

  void setMap(const grid_map_msgs::GridMap &newMap);

  ElevationAwareFootPlanner(std::shared_ptr<GaitScheduler> scheduler,
                            bool updateFootholdsContinuously,
                            ros::NodeHandle &nh);

  Vec3<float> adjustFoothold(const Vec3<float> &nominalFootPosition,
                             const Vec3<float> &currentRobotPosition,
                             const Mat3<float> &currentRobotRotation, int legId,
                             robots::Robot &robot) override;

  float evalFoothold(const Eigen::Array2i &index,
                     const Vec3<float> &nominalPosition, int legId,
                     robots::Robot &robot);
};
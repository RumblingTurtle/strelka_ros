#include <strelka_ros/A1/ElevationAwareLocalPlanner.hpp>

ElevationAwareLocalPlanner::ElevationAwareLocalPlanner(
    std::shared_ptr<ElevationAwareFootPlanner> footPlanner, ros::NodeHandle nh)
    : _footPlanner(footPlanner), A1LocalPlanner(footPlanner) {}

void ElevationAwareLocalPlanner::elevationMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &map) {
  grid_map::GridMapRosConverter::fromMessage(*map, _footPlanner->map);
  if (!_footPlanner->firstMapRecieved()) {
    _footPlanner->setFirstMapRecieved();
  }
}

int main(int argc, char **argv) {
  using namespace strelka::control;
  ros::init(argc, argv, "a1_local_planner");

  ros::NodeHandle nh;

  std::shared_ptr<GaitScheduler> gaitScheduler =
      std::make_shared<GaitScheduler>(strelka::GAITS::TROT);

  std::shared_ptr<ElevationAwareFootPlanner> footPlanner =
      std::make_shared<ElevationAwareFootPlanner>(gaitScheduler, 0.15);
  ElevationAwareLocalPlanner planner{footPlanner, nh};

  ros::Subscriber mapSub =
      nh.subscribe("/elevation_mapping/elevation_map_raw", 1,
                   &ElevationAwareLocalPlanner::elevationMapCallback, &planner);

  while (!footPlanner->firstMapRecieved() && ros::ok()) {
    ros::spinOnce();
  }

  while (planner.handle() && ros::ok()) {
    ros::spinOnce();
  };

  return 0;
}
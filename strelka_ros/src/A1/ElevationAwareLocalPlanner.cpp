#include <strelka_ros/A1/ElevationAwareLocalPlanner.hpp>

ElevationAwareLocalPlanner::ElevationAwareLocalPlanner(
    ElevationAwareFootPlanner &footPlanner, ros::NodeHandle nh)
    : _footPlanner(footPlanner), A1LocalPlanner(footPlanner) {

  mapSub =
      nh.subscribe("/elevation_mapping/elevation_map_raw", 1,
                   &ElevationAwareLocalPlanner::elevationMapCallback, this);
}

void ElevationAwareLocalPlanner::elevationMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &map) {

  grid_map::GridMapRosConverter::fromMessage(*map, _footPlanner.map);
  if (!_footPlanner.firstMapRecieved()) {
    _footPlanner.setFirstMapRecieved();
  }
}

int main(int argc, char **argv) {
  using namespace strelka::control;
  ros::init(argc, argv, "a1_local_planner");

  ros::NodeHandle nh;

  std::shared_ptr<GaitScheduler> gaitScheduler =
      std::make_shared<GaitScheduler>(strelka::GAITS::TROT);

  ElevationAwareFootPlanner footPlanner{gaitScheduler};
  ElevationAwareLocalPlanner planner{footPlanner, nh};

  while (planner.handle() && ros::ok()) {
  };
  return 0;
}
#include <strelka_ros/ElevationAwareFootPlanner.hpp>

ElevationAwareFootPlanner::ElevationAwareFootPlanner(
    std::shared_ptr<GaitScheduler> scheduler, float searchRadius,
    bool updateFootholdsContinuously, ros::NodeHandle &nh)
    : FootholdPlanner(scheduler, updateFootholdsContinuously),
      _searchRadius(searchRadius), firstMapRecieved(false) {
  assert(searchRadius > 0);
  mapSub = nh.subscribe("/elevation_mapping/elevation_map_raw", 1,
                        &ElevationAwareFootPlanner::setMap, this);
}

void ElevationAwareFootPlanner::setMap(const grid_map_msgs::GridMap &newMap) {
  grid_map::GridMapRosConverter::fromMessage(newMap, map);
  if (!firstMapRecieved) {
    firstMapRecieved = true;
  }
}

Vec3<float> ElevationAwareFootPlanner::adjustFoothold(
    const Vec3<float> &nominalFootPosition,
    const Vec3<float> &currentRobotPosition,
    const Mat3<float> &currentRobotRotation, int legId, robots::Robot &robot) {
  if (!firstMapRecieved) {
    return Vec3<float>{nominalFootPosition(0), nominalFootPosition(1),
                       getFoothold(legId, 0)(2)};
  }

  Position nominalPositionCenter =
      nominalFootPosition.cast<double>().block<2, 1>(0, 0);

  Vec3<float> nominalFootPositionWorld = nominalFootPosition;
  try {
    float nominalPosElevation =
        map.atPosition("elevation_inpainted", nominalPositionCenter) +
        robot.footRadius();
    nominalFootPositionWorld(2) = nominalPosElevation;
  } catch (const std::out_of_range &ex) {
    ROS_INFO_STREAM("Failed to get elevation at" << nominalPositionCenter);
    ROS_INFO_STREAM(ex.what());
    if (updateFootholdsContinuously) {
      return prevAdjustedFoothold.col(legId);
    } else {
      return Vec3<float>{nominalFootPosition(0), nominalFootPosition(1),
                         getFoothold(legId, 0)(2)};
    }
  }

  Index nominalPosIndex;
  map.getIndex(nominalPositionCenter, nominalPosIndex);

  if (!std::isnan(
          evalFoothold(nominalPosIndex, nominalFootPositionWorld, legId))) {
    return nominalFootPositionWorld;
  }

  float bestScore = std::nanf("0");
  Index bestIndex = Index{-1, -1};
  for (grid_map::SpiralIterator iterator(map, nominalPositionCenter,
                                         _searchRadius);
       !iterator.isPastEnd(); ++iterator) {
    float score = evalFoothold(*iterator, nominalFootPositionWorld, legId);
    if (std::isnan(score)) {
      continue;
    }

    if (std::isnan(bestScore) || score < bestScore) {
      bestIndex = *iterator;
      bestScore = score;
    }
  }

  if (std::isnan(bestScore)) {
    if (updateFootholdsContinuously) {
      return prevAdjustedFoothold.col(legId);
    } else {
      return Vec3<float>{nominalFootPosition(0), nominalFootPosition(1),
                         getFoothold(legId, 0)(2)};
    }
  }

  Position3 bestPosition;
  map.getPosition3("elevation_inpainted", bestIndex, bestPosition);

  return bestPosition.cast<float>() +
         Vec3<float>{0.0f, 0.0f, robot.footRadius()};
}

float ElevationAwareFootPlanner::evalFoothold(
    const Index &index, const Vec3<float> &nominalPosition, int legId,
    float maxCurvatureThreshold, float sdfThreshold,
    float heightDifferenceThreshold, float maxDistToNominal) {

  bool positionAquired;
  float curvature;
  float sdf;
  float height;

  Position3 footholdPosition;
  try {
    positionAquired =
        map.getPosition3("elevation_inpainted", index, footholdPosition);
    curvature = map.at("curvature", index);
    sdf = map.at("sdf", index);
    height = map.at("elevation_inpainted", index);
  } catch (const std::out_of_range &ex) {
    return std::nanf("0");
  }

  if (curvature > maxCurvatureThreshold) {
    // Surface is not smooth enough
    return std::nanf("0");
  }

  if (std::abs(getFoothold(legId, 0)(2) - height) > heightDifferenceThreshold) {
    // Swing height too high
    return std::nanf("0");
  }

  if (sdf < sdfThreshold) {
    // Foothold is too close to the edge
    return std::nanf("0");
  }

  float distScore = 1;
  if (positionAquired) {
    float distToNominal =
        (nominalPosition - footholdPosition.cast<float>()).norm();

    distScore = distToNominal > maxDistToNominal ? 1 : distToNominal;
  }

  return curvature + distScore;
}
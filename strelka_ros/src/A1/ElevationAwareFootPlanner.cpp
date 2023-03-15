#include <strelka_ros/ElevationAwareFootPlanner.hpp>

ElevationAwareFootPlanner::ElevationAwareFootPlanner(
    std::shared_ptr<GaitScheduler> scheduler, float searchRadius)
    : FootholdPlanner(scheduler), _firstMapRecieved(false),
      _searchRadius(searchRadius) {
  assert(searchRadius > 0);
}

bool ElevationAwareFootPlanner::firstMapRecieved() {
  return _firstMapRecieved;
};

void ElevationAwareFootPlanner::setFirstMapRecieved() {
  _firstMapRecieved = true;
};

Vec3<float> ElevationAwareFootPlanner::adjustFoothold(
    Vec3<float> nominalFootPosition, Vec3<float> currentRobotPosition,
    Mat3<float> currentRobotRotation, int legId, robots::Robot &robot) {
  if (!firstMapRecieved()) {
    nominalFootPosition(2) = getFoothold(legId, 0)(2);
    return nominalFootPosition;
  }

  Position nominalPositionCenter =
      nominalFootPosition.cast<double>().block<2, 1>(0, 0);

  try {
    float nominalPosElevation =
        map.atPosition("elevation_inpainted", nominalPositionCenter) +
        robot.footRadius();
    nominalFootPosition(2) = nominalPosElevation;
  } catch (const std::out_of_range &ex) {
    ROS_INFO_STREAM("Failed to get elevation at" << nominalPositionCenter);
    ROS_INFO_STREAM(ex.what());
    nominalFootPosition(2) = getFoothold(legId, 0)(2);
    return nominalFootPosition;
  }

  Index nominalPosIndex;
  map.getIndex(nominalPositionCenter, nominalPosIndex);

  if (!std::isnan(evalFoothold(nominalPosIndex, nominalFootPosition, legId))) {
    return nominalFootPosition;
  }

  float bestScore = std::nanf("0");
  Index bestIndex = Index{-1, -1};
  for (grid_map::SpiralIterator iterator(map, nominalPositionCenter,
                                         _searchRadius);
       !iterator.isPastEnd(); ++iterator) {
    float score = evalFoothold(*iterator, nominalFootPosition, legId);
    if (std::isnan(score)) {
      continue;
    }
    if (std::isnan(bestScore) || score < bestScore) {
      bestIndex = *iterator;
      bestScore = score;
    }
  }
  if (std::isnan(bestScore)) {
    return nominalFootPosition;
  }

  Position3 bestPosition;

  map.getPosition3("elevation_inpainted", bestIndex, bestPosition);
  bestPosition(2) += robot.footRadius();
  return nominalFootPosition.cast<float>();
}

float ElevationAwareFootPlanner::evalFoothold(
    const Index &index, Vec3<float> nominalPosition, int legId,
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

  float currentFootholdHeight = getFoothold(legId, 0)(2);

  if (std::abs(currentFootholdHeight - height) > heightDifferenceThreshold) {
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

    distScore =
        distToNominal > maxDistToNominal ? 1 : distToNominal / maxDistToNominal;
  }

  return curvature + distScore;
}
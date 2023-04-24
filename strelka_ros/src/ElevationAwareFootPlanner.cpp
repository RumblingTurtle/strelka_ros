#include <strelka_ros/ElevationAwareFootPlanner.hpp>

ElevationAwareFootPlanner::ElevationAwareFootPlanner(
    std::shared_ptr<GaitScheduler> scheduler, bool updateFootholdsContinuously,
    ros::NodeHandle &nh)
    : FootholdPlanner(scheduler, updateFootholdsContinuously),
      firstMapRecieved(false) {
  assert(searchRadius > 0);
  mapSub = nh.subscribe("/elevation_mapping/elevation_map_raw", 1,
                        &ElevationAwareFootPlanner::setMap, this);

  nh.param<float>("/foot_planning/foothold_search_radius", _searchRadius, 0.1);
  nh.param<float>("/foot_planning/max_curvature_threshold",
                  maxCurvatureThreshold, 0.9);
  nh.param<float>("/foot_planning/sdf_threshold", sdfThreshold, 0.02);
  nh.param<float>("/foot_planning/height_difference_threshold",
                  heightDifferenceThreshold, 0.12);
  nh.param<float>("/foot_planning/max_dist_to_nominal", maxDistToNominal, 0.3);
  nh.param<float>("/foot_planning/min_dist_to_other_feet", minDistToOtherFeet,
                  0.2);
  nh.param<float>("/foot_planning/min_support_triangle_area",
                  minSupportTriangleArea, 0.2);
  nh.param<bool>("/foot_planning/test_com_inside_support_polygon",
                 testComInsideSupportPolygon, false);

  while (!firstMapRecieved) {
    ros::spinOnce();
  }
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

  if (!std::isnan(evalFoothold(nominalPosIndex, nominalFootPositionWorld, legId,
                               robot))) {
    return nominalFootPositionWorld;
  }

  float bestScore = std::nanf("0");
  Index bestIndex = Index{-1, -1};
  for (grid_map::SpiralIterator iterator(map, nominalPositionCenter,
                                         _searchRadius);
       !iterator.isPastEnd(); ++iterator) {
    float score =
        evalFoothold(*iterator, nominalFootPositionWorld, legId, robot);
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
    robots::Robot &robot) {

  float curvature;
  float sdf;
  float height;

  Position3 footholdPosition;
  try {
    map.getPosition3("elevation_inpainted", index, footholdPosition);
    curvature = map.at("curvature", index);
    sdf = map.at("sdf", index);
    height = map.at("elevation_inpainted", index);
  } catch (const std::out_of_range &ex) {
    return std::nanf("0");
  }

  Vec3<float> floatFoothold = footholdPosition.cast<float>();
  // NOTE: using current position of the robot is incorrect. Should use the
  // trunk position at the end of swing phase
  if (!robot.worldFrameIKCheck(floatFoothold, legId)) {
    // Check if the foothold is reachable from the current robot postion
    return std::nanf("0");
  }

  if (curvature > maxCurvatureThreshold) {
    // Discard foothold candidates from elevation map which are too close to 1
    return std::nanf("0");
  }

  if (std::abs(getFoothold(legId, 0)(2) - height) > heightDifferenceThreshold) {
    // Discard foothold candidates which are too high from the starting position
    return std::nanf("0");
  }

  if (sdf < sdfThreshold) {
    // Discard foothold candidates which are too close to the inclined surfaces
    return std::nanf("0");
  }

  Eigen::Vector2f pCurrent = floatFoothold.head(2);
  // Skip the first adjustment round because not all footholds are available
  if (footholdCount((legId + 1) % 4) == 0 ||
      footholdCount((legId + 2) % 4) == 0 ||
      footholdCount((legId + 3) % 4) == 0) {
    return std::nanf("0");
  }

  Eigen::Vector2f rightVCurrentDiff =
      getFoothold((legId + 1) % 4, 0).head(2) - pCurrent;
  Eigen::Vector2f acrossVCurrentDiff =
      getFoothold((legId + 2) % 4, 0).head(2) - pCurrent;
  Eigen::Vector2f leftVCurrentDiff =
      getFoothold((legId + 3) % 4, 0).head(2) - pCurrent;
  Eigen::Vector2f rightVLeftDiff = getFoothold((legId + 1) % 4, 0).head(2) -
                                   getFoothold((legId + 3) % 4, 0).head(2);

  // Maximize the distance from foothold position to all other end effectors
  // Only uses current footholds
  if (leftVCurrentDiff.norm() <= minDistToOtherFeet ||
      rightVCurrentDiff.norm() <= minDistToOtherFeet ||
      acrossVCurrentDiff.norm() <= minDistToOtherFeet) {
    return std::nanf("0");
  }

  // Maximize the triangle area constructed using current foothold position
  // and adjacent foot positions
  // Same as:
  // (rightVCurrentDiff.cross(leftVCurrentDiff)/rightVLeftDiff.norm()).norm()
  float dist = std::abs((rightVCurrentDiff(0) * leftVCurrentDiff(1) -
                         rightVCurrentDiff(1) * leftVCurrentDiff(0)) /
                        rightVLeftDiff.norm());

  if (dist < minSupportTriangleArea) {
    return std::nanf("0");
  }

  if (testComInsideSupportPolygon) {
    bool insideThePolygon = false;
    Eigen::Vector2f comPos =
        (robot.positionWorldFrame() +
         robot.rotateBodyToWorldFrame(robot.bodyToComOffset()))
            .head(2);

    for (int i = 0; i < 4; i++) {
      // PNPOLY test for point in polygon
      // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
      int j = (i + 1) % 4;
      Eigen::Vector2f p0 = getFoothold(i, 0).head(2);
      Eigen::Vector2f p1 = getFoothold(j, 0).head(2);

      if (i == legId) {
        p0 = {floatFoothold[0], floatFoothold[1]};
      }

      if (j == legId) {
        p1 = {floatFoothold[0], floatFoothold[1]};
      }

      if (((p0(1) > comPos(1)) != (p1(1) > comPos(1))) &&
          (comPos(0) <
           (p1(0) - p0(0)) * (comPos(1) - p0(1)) / (p1(1) - p0(1)) + p0(0))) {
        insideThePolygon = !insideThePolygon;
      }
    }

    if (!insideThePolygon) {
      return std::nanf("0");
    }
  }

  // Minimize distance to the nominal position
  float distToNominal = (nominalPosition - floatFoothold).norm();
  float distScore = distToNominal > maxDistToNominal ? 1 : distToNominal;
  return curvature + distScore;
}
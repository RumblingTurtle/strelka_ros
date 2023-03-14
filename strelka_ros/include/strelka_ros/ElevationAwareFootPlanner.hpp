#include <grid_map_core/grid_map_core.hpp>
#include <strelka/control/FootholdPlanner.hpp>
#include <strelka/control/gait/GaitScheduler.hpp>

using namespace grid_map;
using namespace strelka;
using namespace strelka::control;

class ElevationAwareFootPlanner : public FootholdPlanner {
  bool _firstMapRecieved;

public:
  GridMap map;
  float _searchRadius;
  ElevationAwareFootPlanner(std::shared_ptr<GaitScheduler> scheduler,
                            float searchRadius)
      : FootholdPlanner(scheduler), _firstMapRecieved(false),
        _searchRadius(searchRadius) {
    assert(searchRadius > 0);
  }

  bool firstMapRecieved() { return _firstMapRecieved; };

  void setFirstMapRecieved() { _firstMapRecieved = true; };

  virtual Vec3<float> adjustFoothold(Vec3<float> nominalFootPosition,
                                     Vec3<float> currentRobotPosition,
                                     Mat3<float> currentRobotRotation,
                                     int legId, robots::Robot &robot) override {
    if (!firstMapRecieved()) {
      nominalFootPosition(2) = getFoothold(legId, 0)(2);
      return nominalFootPosition;
    }
    Position nominalPositionCenter =
        nominalFootPosition.cast<double>().block<2, 1>(0, 0);

    try {
      nominalFootPosition(2) =
          map.atPosition("elevation_inpainted", nominalPositionCenter);
    } catch (const std::out_of_range &ex) {
      ROS_INFO_STREAM("Failed to get elevation at" << nominalPositionCenter);
      ROS_INFO_STREAM(ex.what());
      nominalFootPosition(2) = getFoothold(legId, 0)(2);
      return nominalFootPosition;
    }

    float maxCurvatureThreshold = 0.9;
    float sdfThreshold = 0.02;
    float heightDifferenceThreshold = 0.12;
    float maxDistToNominal = 0.3;

    Position3 footholdPosition;
    // Set height of the nominal foothold

    float bestScore;
    for (grid_map::SpiralIterator iterator(map, nominalPositionCenter,
                                           _searchRadius);
         !iterator.isPastEnd(); ++iterator) {

      bool positionAquired;
      float curvature;
      float sdf;
      float height;

      try {
        positionAquired = map.getPosition3("elevation_inpainted", *iterator,
                                           footholdPosition);
        curvature = map.at("curvature", *iterator);
        sdf = map.at("sdf", *iterator);
        height = map.at("elevation_inpainted", *iterator);
      } catch (const std::out_of_range &ex) {
        continue;
      }

      if (curvature > maxCurvatureThreshold) {
        // Surface is not smooth enough
        continue;
      }

      float currentFootholdHeight = getFoothold(legId, 0)(2);

      if (std::abs(currentFootholdHeight - height) >
          heightDifferenceThreshold) {
        // Swing height too high
        continue;
      }

      if (sdf < sdfThreshold) {
        // Foothold is too close to the edge
        continue;
      }

      float distScore = 1;
      if (positionAquired) {
        float distToNominal =
            (nominalFootPosition - footholdPosition.cast<float>()).norm();

        distScore = distToNominal > maxDistToNominal
                        ? 1
                        : distToNominal / maxDistToNominal;
      }

      float totalScore = curvature + distScore;
    }
    return nominalFootPosition;
  }
};
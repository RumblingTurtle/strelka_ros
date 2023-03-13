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

  ElevationAwareFootPlanner(std::shared_ptr<GaitScheduler> scheduler)
      : FootholdPlanner(scheduler), _firstMapRecieved(false) {}

  bool firstMapRecieved() { return _firstMapRecieved; };

  void setFirstMapRecieved() { _firstMapRecieved = true; };

  virtual Vec3<float> adjustFoothold(Vec3<float> nominalFootPosition,
                                     Vec3<float> currentRobotPosition,
                                     Mat3<float> currentRobotRotation,
                                     int legId, robots::Robot &robot) {
    nominalFootPosition(2) = getFoothold(legId, 0)(2);
    return nominalFootPosition;
    if (!firstMapRecieved()) {
      return nominalFootPosition;
    }
    return nominalFootPosition;
  }
};
#ifndef SCENARIO_MANAGER_H_
#define SCENARIO_MANAGER_H_

#include <vector>
#include "reference_point.h"
#include "Obstacle.h"
#include "path_struct.h"

struct ScenarioConfig {
  std::vector<ReferencePoint> reference_line;
  std::vector<double> accumulated_s;
  std::vector<Obstacle> obstacles;
  InitialConditions vehicle_state;
  double target_speed;
  int scenario_id;
};

ScenarioConfig build_scenario(int scenario_id);

#endif

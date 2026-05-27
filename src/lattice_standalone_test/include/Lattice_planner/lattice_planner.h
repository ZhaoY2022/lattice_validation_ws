#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include <limits>
#include "lattice_trajectory1d.h"
#include "trajectory_evaluator.h"
#include "trajectory1d_generator.h"
#include "PlanningTarget.h"
#include "path_time_graph.h"
#include "collision_checker.h"
#include "constraint_checker.h"
#include "trajectory_combiner.h"

struct LatticeTrajectoryResult
{
  DiscretizedTrajectory trajectory;
  bool constraint_valid;  // passed kinematic constraint check
  bool collision_free;    // passed collision check
  double cost;            // trajectory pair cost (lower = better)
};

class LatticePlanner
{
public:
  LatticePlanner();
  ~LatticePlanner() = default;
  DiscretizedTrajectory LatticePlan(
      const InitialConditions &planning_init_point,
      const PlanningTarget &planning_target,
      const std::vector<const Obstacle *> &obstacles,
      const std::vector<double> &accumulated_s,
      const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
      const double &init_relative_time, const double &lon_decision_horizon,
      std::vector<LatticeTrajectoryResult> *all_trajectories = nullptr,
      double preferred_lat_sign = 0.0,
      DrivingMode driving_mode = DrivingMode::LCC,
      double current_speed = 0.0,
      double distance_to_obstacle = std::numeric_limits<double>::max(),
      double target_lat_offset = 0.0);

private:
  TrajectoryCombiner trajectorycombiner;
  ConstraintChecker constraintchecker_;
  double last_matched_s_ = 0.0;
  // ros::Publisher Obstacle_Prediction_;
};

#endif
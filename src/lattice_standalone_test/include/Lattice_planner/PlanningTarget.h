#ifndef PLANNINGTARGET_H
#define PLANNINGTARGET_H
#include <vector>
#include <iostream>

enum class DrivingMode
{
  LCC = 0,   // Lane Centering Control
  ALC = 1,   // Auto Lane Change
  NUDGE = 2  // Nudge / Obstacle Avoidance
};

class PlanningTarget
{
public:
  PlanningTarget();
  PlanningTarget(const double speed, const double stop_point, const std::vector<double> &reference_s);
  ~PlanningTarget();
  double cruise_speed() const;
  bool has_stop_point() const;
  void set_stop_point();
  double stop_point() const;

  double target_lat_offset() const { return target_lat_offset_; }
  void set_target_lat_offset(double v) { target_lat_offset_ = v; }

private:
  std::vector<double> reference_s_;
  double speed_;
  double stop_point_;
  double target_lat_offset_ = 0.0;
  bool has;
};
#endif
#include "scenario_manager.h"
#include "common/math_utils.h"
#include <cmath>

namespace {

void build_straight_ref_line(double x_start, double x_end, double y_const,
                             double resolution,
                             std::vector<ReferencePoint>& ref_line,
                             std::vector<double>& acc_s)
{
  ref_line.clear();
  acc_s.clear();
  double s = 0.0;
  for (double x = x_start; x <= x_end; x += resolution) {
    ref_line.push_back(ReferencePoint(0.0, 0.0, x, y_const, 0.0, s));
    acc_s.push_back(s);
    s += resolution;
  }
}

void init_scenario_1(ScenarioConfig& cfg)
{
  build_straight_ref_line(0.0, 500.0, 0.0, 0.5, cfg.reference_line, cfg.accumulated_s);
  cfg.target_speed = 22.0;

  cfg.vehicle_state.x_init = 0.0;
  cfg.vehicle_state.y_init = 0.5;
  cfg.vehicle_state.z_init = 0.0;
  cfg.vehicle_state.theta_init = 0.0;
  cfg.vehicle_state.kappa_init = 0.0;
  cfg.vehicle_state.dkappa_init = 0.0;
  cfg.vehicle_state.v_init = 22.0;
  cfg.vehicle_state.a_init = 0.0;
  cfg.vehicle_state.d0 = 0.5;
  cfg.vehicle_state.dd0 = 0.0;
  cfg.vehicle_state.ddd0 = 0.0;
  cfg.vehicle_state.s0 = 0.0;
  cfg.vehicle_state.ds0 = 22.0;
  cfg.vehicle_state.dds0 = 0.0;
  cfg.vehicle_state.init_relative_time = 0.0;
}

void init_scenario_2(ScenarioConfig& cfg)
{
  build_straight_ref_line(0.0, 500.0, 0.0, 0.5, cfg.reference_line, cfg.accumulated_s);
  cfg.target_speed = 22.0;

  cfg.vehicle_state.x_init = 0.0;
  cfg.vehicle_state.y_init = 0.0;
  cfg.vehicle_state.z_init = 0.0;
  cfg.vehicle_state.theta_init = 0.0;
  cfg.vehicle_state.kappa_init = 0.0;
  cfg.vehicle_state.dkappa_init = 0.0;
  cfg.vehicle_state.v_init = 22.0;
  cfg.vehicle_state.a_init = 0.0;
  cfg.vehicle_state.d0 = 0.0;
  cfg.vehicle_state.dd0 = 0.0;
  cfg.vehicle_state.ddd0 = 0.0;
  cfg.vehicle_state.s0 = 0.0;
  cfg.vehicle_state.ds0 = 22.0;
  cfg.vehicle_state.dds0 = 0.0;
  cfg.vehicle_state.init_relative_time = 0.0;

  Obstacle obs;
  obs.obstacle_id = "static_obs_1";
  obs.obstacle_type = 1;
  obs.obstacle_shape = 0;
  obs.obstacle_length = 4.0;
  obs.obstacle_width = 2.0;
  obs.obstacle_height = 2.0;
  obs.obstacle_velocity = 0.0;
  obs.obstacle_acc = 0.0;
  obs.obstacle_threa = 0.0;
  obs.obstacle_radius = std::sqrt(2.0 * 2.0 + 1.0 * 1.0);
  obs.centerpoint.position.x = 95.0;
  obs.centerpoint.position.y = -1.6;
  obs.centerpoint.position.z = 0.0;
  obs.centerpoint.orientation.w = 1.0;

  double hw = obs.obstacle_width / 2.0;
  double hl = obs.obstacle_length / 2.0;
  obs.polygon_points = {
      Vec2d(95.0 - hl, -1.6 - hw), Vec2d(95.0 + hl, -1.6 - hw),
      Vec2d(95.0 + hl, -1.6 + hw), Vec2d(95.0 - hl, -1.6 + hw)
  };
  obs.SetPerceptionBoundingBox(
      Box2d(Vec2d(95.0, -1.6), 0.0, obs.obstacle_length, obs.obstacle_width));
  obs.SetPerceptionPolygon(common::math::Polygon2d(obs.polygon_points));

  for (auto& pt : obs.polygon_points) {
    geometry_msgs::msg::Pose p;
    p.position.x = pt.x();
    p.position.y = pt.y();
    p.position.z = 0.0;
    p.orientation.w = 1.0;
    obs.pinnacle.poses.push_back(p);
  }

  cfg.obstacles.push_back(obs);
}

void init_scenario_3(ScenarioConfig& cfg)
{
  build_straight_ref_line(0.0, 500.0, 0.0, 0.5, cfg.reference_line, cfg.accumulated_s);
  cfg.target_speed = 22.0;

  cfg.vehicle_state.x_init = 0.0;
  cfg.vehicle_state.y_init = 0.0;
  cfg.vehicle_state.z_init = 0.0;
  cfg.vehicle_state.theta_init = 0.0;
  cfg.vehicle_state.kappa_init = 0.0;
  cfg.vehicle_state.dkappa_init = 0.0;
  cfg.vehicle_state.v_init = 22.0;
  cfg.vehicle_state.a_init = 0.0;
  cfg.vehicle_state.d0 = 0.0;
  cfg.vehicle_state.dd0 = 0.0;
  cfg.vehicle_state.ddd0 = 0.0;
  cfg.vehicle_state.s0 = 0.0;
  cfg.vehicle_state.ds0 = 22.0;
  cfg.vehicle_state.dds0 = 0.0;
  cfg.vehicle_state.init_relative_time = 0.0;
}

void init_scenario_5(ScenarioConfig& cfg)
{
  // 4.2km continuous S-curve highway with clothoid transitions.
  // Curvature keypoint profile: {s [m], kappa [1/m]} pairs.
  const std::vector<std::pair<double, double>> kp = {
    {0.0,    0.0},          {200.0,  0.0},
    {300.0,  1.0 / 500.0},  {800.0,  1.0 / 500.0},   // 500m radius left curve
    {900.0,  0.0},          {1200.0, 0.0},
    {1350.0, -1.0 / 700.0}, {2050.0, -1.0 / 700.0},  // 700m radius right curve
    {2200.0, 0.0},          {2500.0, 0.0},
    {2700.0, 1.0 / 1000.0}, {3700.0, 1.0 / 1000.0},  // 1000m radius left curve
    {3900.0, 0.0},          {4200.0, 0.0}
  };

  const double ds = 0.5;
  const double total_s = 4200.0;

  cfg.reference_line.clear();
  cfg.accumulated_s.clear();

  double x = 0.0, y = 0.0, heading = 0.0;
  size_t kpi = 0;

  for (double s = 0.0; s <= total_s + 1e-9; s += ds) {
    while (kpi + 1 < kp.size() && s > kp[kpi + 1].first + 1e-9) {
      kpi++;
    }

    double s0_kp = kp[kpi].first;
    double s1_kp = kp[kpi + 1].first;
    double k0 = kp[kpi].second;
    double k1 = kp[kpi + 1].second;
    double frac = (s - s0_kp) / (s1_kp - s0_kp);
    double kappa = k0 + frac * (k1 - k0);
    double dkappa = (k1 - k0) / (s1_kp - s0_kp);

    heading += kappa * ds;
    if (std::abs(heading) > 100.0 * M_PI) {
      heading = std::fmod(heading + M_PI, 2.0 * M_PI) - M_PI;
    }
    x += std::cos(heading) * ds;
    y += std::sin(heading) * ds;

    cfg.reference_line.push_back(ReferencePoint(kappa, dkappa, x, y, heading, s));
    cfg.accumulated_s.push_back(s);
  }

  cfg.target_speed = 22.0;

  cfg.vehicle_state.x_init = 0.0;
  cfg.vehicle_state.y_init = 0.0;
  cfg.vehicle_state.z_init = 0.0;
  cfg.vehicle_state.theta_init = 0.0;
  cfg.vehicle_state.kappa_init = 0.0;
  cfg.vehicle_state.dkappa_init = 0.0;
  cfg.vehicle_state.v_init = 22.0;
  cfg.vehicle_state.a_init = 0.0;
  cfg.vehicle_state.d0 = 0.0;
  cfg.vehicle_state.dd0 = 0.0;
  cfg.vehicle_state.ddd0 = 0.0;
  cfg.vehicle_state.s0 = 0.0;
  cfg.vehicle_state.ds0 = 22.0;
  cfg.vehicle_state.dds0 = 0.0;
  cfg.vehicle_state.init_relative_time = 0.0;
}

}  // anonymous namespace

ScenarioConfig build_scenario(int scenario_id)
{
  ScenarioConfig cfg;
  cfg.scenario_id = scenario_id;

  switch (scenario_id) {
    case 1: init_scenario_1(cfg); break;
    case 2: init_scenario_2(cfg); break;
    case 3: init_scenario_3(cfg); break;
    case 5: init_scenario_5(cfg); break;
    default: init_scenario_1(cfg); break;
  }
  return cfg;
}

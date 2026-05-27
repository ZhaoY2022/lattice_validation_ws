#pragma once

#include <vector>
#include "yaml-cpp/yaml.h"

class Param_Configs
{
public:
  YAML::Node config;

  // 基础配置
  double FLAGS_vehicle_width;
  double FLAGS_vehicle_length;
  double front_edge_to_center;
  double back_edge_to_center;
  double FLAGS_default_reference_line_width;

  // 轨迹时间/空间参数
  double FLAGS_trajectory_time_length;
  double FLAGS_trajectory_time_resolution;
  double FLAGS_trajectory_space_resolution;
  double FLAGS_polynomial_minimal_param;

  // 速度规划
  double FLAGS_num_velocity_sample;
  double FLAGS_min_velocity_sample_gap;

  // 约束参数
  double FLAGS_speed_lower_bound;
  double FLAGS_speed_upper_bound;
  double FLAGS_longitudinal_acceleration_lower_bound;
  double FLAGS_longitudinal_acceleration_upper_bound;
  double FLAGS_comfort_acceleration_factor;
  double FLAGS_longitudinal_jerk_lower_bound;
  double FLAGS_longitudinal_jerk_upper_bound;
  double FLAGS_kappa_bound;
  double FLAGS_lateral_acceleration_bound;

  // 代价权重
  double FLAGS_weight_target_speed;
  double FLAGS_weight_dist_travelled;
  double FLAGS_weight_lon_objective;
  double FLAGS_weight_lon_jerk;
  double FLAGS_weight_lon_collision;
  double FLAGS_weight_centripetal_acceleration;
  double FLAGS_weight_lat_offset;
  double FLAGS_weight_lat_comfort;

  // 纵向评价视界
  double FLAGS_speed_lon_decision_horizon;

  // 纵向碰撞代价
  double FLAGS_lon_collision_cost_std;
  double FLAGS_lon_collision_yield_buffer;
  double FLAGS_lon_collision_overtake_buffer;

  // 碰撞缓冲区
  double FLAGS_lon_collision_buffer;
  double FLAGS_lat_collision_buffer;
  double FLAGS_lattice_stop_buffer;

  // 数值计算
  double FLAGS_numerical_epsilon;
  double FLAGS_bound_buffer;
  double FLAGS_nudge_buffer;

  // ST 图 / 障碍物采样
  double FLAGS_default_lon_buffer;
  double FLAGS_time_min_density;
  double FLAGS_num_sample_follow_per_timestamp;
  double FLAGS_prediction_total_time;
  double eval_time_interval;

  //---------------------------------- 模式特定横向采样参数 ----------------------------------------//
  // LCC
  std::vector<double> LCC_end_d_candidates;
  double LCC_smin_multiplier;
  std::vector<double> LCC_end_s_deltas;

  // ALC
  std::vector<double> ALC_end_d_fractions;
  std::vector<double> ALC_end_s_times;

  // NUDGE
  std::vector<double> NUDGE_end_d_candidates;
  double NUDGE_kinematic_smin_multiplier;
  double NUDGE_safe_smin_min;
  double NUDGE_safe_smin_offset;
  std::vector<double> NUDGE_end_s_deltas;

  //---------------------------------- 模式特定评价权重 ----------------------------------------//
  // LCC
  double LCC_w_lat_offset;
  double LCC_w_opposite_side;
  double LCC_w_same_side;
  double LCC_w_lat_comfort;
  double LCC_w_lat_offset_bound;

  // ALC
  double ALC_w_lat_offset;
  double ALC_w_opposite_side;
  double ALC_w_same_side;
  double ALC_w_lat_comfort;
  double ALC_w_lat_offset_bound;
  double ALC_overshoot_penalty;

  // NUDGE
  double NUDGE_w_lat_offset;
  double NUDGE_w_opposite_side;
  double NUDGE_w_same_side;
  double NUDGE_w_lat_comfort;
  double NUDGE_w_lat_offset_bound;

  // 横向偏好 tiebreaker
  double preferred_lat_sign_tiebreaker;

public:
  Param_Configs();
  ~Param_Configs();
};

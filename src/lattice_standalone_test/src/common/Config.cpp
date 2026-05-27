#include "Configs.h"
#include <iostream>
#include <cstdlib>

Param_Configs::Param_Configs()
{
  const char* config_path = std::getenv("LATTICE_CONFIG_PATH");
  if (config_path) {
    try {
      config = YAML::LoadFile(config_path);
    } catch (...) {
      std::cerr << "[Config] Failed to load " << config_path << ", using defaults" << std::endl;
    }
  }

  // 基础配置
  FLAGS_vehicle_width = config["FLAGS_vehicle_width"].as<double>();
  FLAGS_vehicle_length = config["FLAGS_vehicle_length"].as<double>();
  front_edge_to_center = config["front_edge_to_center"].as<double>();
  back_edge_to_center = config["back_edge_to_center"].as<double>();
  FLAGS_default_reference_line_width = config["FLAGS_default_reference_line_width"].as<double>();

  // 轨迹时间/空间参数
  FLAGS_trajectory_time_length = config["FLAGS_trajectory_time_length"].as<double>();
  FLAGS_trajectory_time_resolution = config["FLAGS_trajectory_time_resolution"].as<double>();
  FLAGS_trajectory_space_resolution = config["FLAGS_trajectory_space_resolution"].as<double>();
  FLAGS_polynomial_minimal_param = config["FLAGS_polynomial_minimal_param"].as<double>();

  // 速度规划
  FLAGS_num_velocity_sample = config["FLAGS_num_velocity_sample"].as<double>();
  FLAGS_min_velocity_sample_gap = config["FLAGS_min_velocity_sample_gap"].as<double>();

  // 约束参数
  FLAGS_speed_lower_bound = config["FLAGS_speed_lower_bound"].as<double>();
  FLAGS_speed_upper_bound = config["FLAGS_speed_upper_bound"].as<double>();
  FLAGS_longitudinal_acceleration_lower_bound = config["FLAGS_longitudinal_acceleration_lower_bound"].as<double>();
  FLAGS_longitudinal_acceleration_upper_bound = config["FLAGS_longitudinal_acceleration_upper_bound"].as<double>();
  FLAGS_comfort_acceleration_factor = config["FLAGS_comfort_acceleration_factor"].as<double>();
  FLAGS_longitudinal_jerk_lower_bound = config["FLAGS_longitudinal_jerk_lower_bound"].as<double>();
  FLAGS_longitudinal_jerk_upper_bound = config["FLAGS_longitudinal_jerk_upper_bound"].as<double>();
  FLAGS_kappa_bound = config["FLAGS_kappa_bound"].as<double>();
  FLAGS_lateral_acceleration_bound = config["FLAGS_lateral_acceleration_bound"].as<double>();

  // 代价权重
  FLAGS_weight_target_speed = config["FLAGS_weight_target_speed"].as<double>();
  FLAGS_weight_dist_travelled = config["FLAGS_weight_dist_travelled"].as<double>();
  FLAGS_weight_lon_objective = config["FLAGS_weight_lon_objective"].as<double>();
  FLAGS_weight_lon_jerk = config["FLAGS_weight_lon_jerk"].as<double>();
  FLAGS_weight_lon_collision = config["FLAGS_weight_lon_collision"].as<double>();
  FLAGS_weight_centripetal_acceleration = config["FLAGS_weight_centripetal_acceleration"].as<double>();
  FLAGS_weight_lat_offset = config["FLAGS_weight_lat_offset"].as<double>();
  FLAGS_weight_lat_comfort = config["FLAGS_weight_lat_comfort"].as<double>();

  // 纵向评价视界
  FLAGS_speed_lon_decision_horizon = config["FLAGS_speed_lon_decision_horizon"].as<double>();

  // 纵向碰撞代价
  FLAGS_lon_collision_cost_std = config["FLAGS_lon_collision_cost_std"].as<double>();
  FLAGS_lon_collision_yield_buffer = config["FLAGS_lon_collision_yield_buffer"].as<double>();
  FLAGS_lon_collision_overtake_buffer = config["FLAGS_lon_collision_overtake_buffer"].as<double>();

  // 碰撞缓冲区
  FLAGS_lon_collision_buffer = config["FLAGS_lon_collision_buffer"].as<double>();
  FLAGS_lat_collision_buffer = config["FLAGS_lat_collision_buffer"].as<double>();
  FLAGS_lattice_stop_buffer = config["FLAGS_lattice_stop_buffer"].as<double>();

  // 数值计算
  FLAGS_numerical_epsilon = config["FLAGS_numerical_epsilon"].as<double>();
  FLAGS_bound_buffer = config["FLAGS_bound_buffer"].as<double>();
  FLAGS_nudge_buffer = config["FLAGS_nudge_buffer"].as<double>();

  // ST 图 / 障碍物采样
  FLAGS_default_lon_buffer = config["FLAGS_default_lon_buffer"].as<double>();
  FLAGS_time_min_density = config["FLAGS_time_min_density"].as<double>();
  FLAGS_num_sample_follow_per_timestamp = config["FLAGS_num_sample_follow_per_timestamp"].as<double>();
  FLAGS_prediction_total_time = config["FLAGS_prediction_total_time"].as<double>();
  eval_time_interval = config["eval_time_interval"].as<double>();

  //---------------------------------- 模式特定横向采样参数 ----------------------------------------//
  // LCC
  LCC_end_d_candidates = config["LCC_end_d_candidates"].as<std::vector<double>>();
  LCC_smin_multiplier = config["LCC_smin_multiplier"].as<double>();
  LCC_end_s_deltas = config["LCC_end_s_deltas"].as<std::vector<double>>();

  // ALC
  ALC_end_d_fractions = config["ALC_end_d_fractions"].as<std::vector<double>>();
  ALC_end_s_times = config["ALC_end_s_times"].as<std::vector<double>>();

  // NUDGE
  NUDGE_end_d_candidates = config["NUDGE_end_d_candidates"].as<std::vector<double>>();
  NUDGE_kinematic_smin_multiplier = config["NUDGE_kinematic_smin_multiplier"].as<double>();
  NUDGE_safe_smin_min = config["NUDGE_safe_smin_min"].as<double>();
  NUDGE_safe_smin_offset = config["NUDGE_safe_smin_offset"].as<double>();
  NUDGE_end_s_deltas = config["NUDGE_end_s_deltas"].as<std::vector<double>>();

  //---------------------------------- 模式特定评价权重 ----------------------------------------//
  // LCC
  LCC_w_lat_offset = config["LCC_w_lat_offset"].as<double>();
  LCC_w_opposite_side = config["LCC_w_opposite_side"].as<double>();
  LCC_w_same_side = config["LCC_w_same_side"].as<double>();
  LCC_w_lat_comfort = config["LCC_w_lat_comfort"].as<double>();
  LCC_w_lat_offset_bound = config["LCC_w_lat_offset_bound"].as<double>();

  // ALC
  ALC_w_lat_offset = config["ALC_w_lat_offset"].as<double>();
  ALC_w_opposite_side = config["ALC_w_opposite_side"].as<double>();
  ALC_w_same_side = config["ALC_w_same_side"].as<double>();
  ALC_w_lat_comfort = config["ALC_w_lat_comfort"].as<double>();
  ALC_w_lat_offset_bound = config["ALC_w_lat_offset_bound"].as<double>();
  ALC_overshoot_penalty = config["ALC_overshoot_penalty"].as<double>();

  // NUDGE
  NUDGE_w_lat_offset = config["NUDGE_w_lat_offset"].as<double>();
  NUDGE_w_opposite_side = config["NUDGE_w_opposite_side"].as<double>();
  NUDGE_w_same_side = config["NUDGE_w_same_side"].as<double>();
  NUDGE_w_lat_comfort = config["NUDGE_w_lat_comfort"].as<double>();
  NUDGE_w_lat_offset_bound = config["NUDGE_w_lat_offset_bound"].as<double>();

  // 横向偏好 tiebreaker
  preferred_lat_sign_tiebreaker = config["preferred_lat_sign_tiebreaker"].as<double>();
}

Param_Configs::~Param_Configs()
{
}

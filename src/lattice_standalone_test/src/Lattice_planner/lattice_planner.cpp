#include "lattice_planner.h"
#include <fstream>

/*
1.将参考线转变为离散地图点（省去,因为我们的参考线是根据中心线生成的，本身点是离散的，不用再离散化）
2.计算参考线上初始规划点的匹配点
3.根据匹配点计算Frenet帧的初始状态
4.解析决策，得到规划目标
5.分别生成纵向和横向一维轨迹束
6.评价：首先，根据动态约束条件对一维轨迹的可行性进行评价；其次，评估可行的纵向和横向轨迹对，并根据成本进行排序。
7.返回无碰撞的、符合条件的轨迹
*/

namespace
{
  std::vector<PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint> &ref_points)
  {
    double s = 0.0;
    std::vector<PathPoint> path_points;
    for (const auto &ref_point : ref_points)
    {
      PathPoint path_point;
      path_point.set_x(ref_point.x_);
      path_point.set_y(ref_point.y_);
      path_point.set_theta(ref_point.heading());
      path_point.set_kappa(ref_point.kappa());
      path_point.set_dkappa(ref_point.dkappa());

      if (!path_points.empty())
      {
        double dx = path_point.x - path_points.back().x;
        double dy = path_point.y - path_points.back().y;
        s += std::sqrt(dx * dx + dy * dy);
      }
      path_point.set_s(s);
      path_points.push_back(std::move(path_point));
    }
    return path_points;
  }

  void ComputeInitFrenetState(const ReferencePoint &matched_point,
                              const InitialConditions &cartesian_state,
                              std::array<double, 3> *ptr_s,
                              std::array<double, 3> *ptr_d)
  {
    CartesianFrenetConverter::cartesian_to_frenet(
        matched_point.accumulated_s_, matched_point.x_, matched_point.y_,
        matched_point.heading_, matched_point.kappa_, matched_point.dkappa_,
        cartesian_state.x_init, cartesian_state.y_init,
        cartesian_state.v_init, cartesian_state.a_init,
        cartesian_state.theta_init,
        cartesian_state.kappa_init, ptr_s, ptr_d);
  }

} // namespace

LatticePlanner::LatticePlanner()
{
  // Obstacle_Prediction_ = nh_.advertise<geometry_msgs::msg::PoseArray>("/xsj/obstacle/obstacle_prediction", 10); // 发布预测轨迹
}

/*轨迹的生成规划*/
DiscretizedTrajectory LatticePlanner::LatticePlan(
    const InitialConditions &planning_init_point,
    const PlanningTarget &planning_target,
    const std::vector<const Obstacle *> &obstacles,
    const std::vector<double> &accumulated_s,
    const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
    const double &init_relative_time, const double &lon_decision_horizon,
    std::vector<LatticeTrajectoryResult> *all_trajectories,
    double preferred_lat_sign,
    DrivingMode driving_mode,
    double current_speed,
    double distance_to_obstacle,
    double target_lat_offset)
{
  DiscretizedTrajectory Optim_trajectory;

  std::ofstream dbg("/tmp/lattice_debug.log", std::ios::app);
  dbg << "[LATTICE] LatticePlan called: ref_pts=" << reference_points.size()
      << " acc_s_size=" << accumulated_s.size()
      << " veh=(" << planning_init_point.x_init << "," << planning_init_point.y_init << ")"
      << " theta=" << planning_init_point.theta_init
      << " v=" << planning_init_point.v_init
      << " kappa_init=" << planning_init_point.kappa_init << std::endl;

  // auto beforeTime = std::chrono::steady_clock::now(); // 计时开始

  // 1. compute the matched point of the init planning point on the reference line.
  // Use a windowed search around the last matched s to prevent matching from
  // jumping between different arms of the route on curves and U-turns.
  ReferencePoint matched_point;
  const double kWindowRadius = 30.0;  // search ±30m around last match
  if (last_matched_s_ > 0.0)
  {
    double s_min = last_matched_s_ - kWindowRadius;
    double s_max = last_matched_s_ + kWindowRadius;
    size_t lo = 0, hi = reference_points.size();
    for (size_t i = 0; i < reference_points.size(); ++i)
    {
      if (reference_points[i].accumulated_s_ < s_min)
        lo = i + 1;
      if (reference_points[i].accumulated_s_ <= s_max)
        hi = i + 1;
    }

    double best_dist = 1e9;
    size_t best_i = lo;
    for (size_t i = lo; i < hi && i < reference_points.size(); ++i)
    {
      double dx = reference_points[i].x_ - planning_init_point.x_init;
      double dy = reference_points[i].y_ - planning_init_point.y_init;
      double d2 = dx * dx + dy * dy;
      if (d2 < best_dist)
      {
        best_dist = d2;
        best_i = i;
      }
    }
    // Only accept windowed result if it's reasonably close (< 5m)
    if (best_dist < 25.0)
    {
      // Use interpolated projection for accurate Frenet conversion
      size_t idx_start = (best_i == 0) ? best_i : best_i - 1;
      size_t idx_end = (best_i + 1 == reference_points.size()) ? best_i : best_i + 1;
      if (idx_start == idx_end)
      {
        matched_point = reference_points[best_i];
      }
      else
      {
        matched_point = PathMatcher::FindProjectionPoint(
            reference_points[idx_start], reference_points[idx_end],
            planning_init_point.x_init, planning_init_point.y_init);
      }
    }
    else
    {
      // Fall back to full search if window result is too far
      matched_point = PathMatcher::MatchToPath(reference_points, planning_init_point.x_init,
                                               planning_init_point.y_init);
    }
  }
  else
  {
    matched_point = PathMatcher::MatchToPath(reference_points, planning_init_point.x_init,
                                              planning_init_point.y_init);
  }
  last_matched_s_ = matched_point.accumulated_s_;
  dbg << "[LATTICE] ref_line size=" << reference_points.size()
      << " vehicle=(" << planning_init_point.x_init << "," << planning_init_point.y_init << ")"
      << " theta=" << planning_init_point.theta_init
      << " matched=(" << matched_point.x_ << "," << matched_point.y_ << ")"
      << " matched_s=" << matched_point.accumulated_s_ << std::endl;

  // 2. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
  // Fix: ensure longitudinal velocity is positive when vehicle is moving forward.
  // Cartesian→Frenet conversion can yield negative s_dot when vehicle heading
  // opposes reference line direction at the matched point.
  if (init_s[1] < -0.5)
  {
    init_s[1] = std::abs(init_s[1]);
  }
  // Carry forward the lateral deviation from the previous planning cycle
  // only in NUDGE mode with nearby obstacles. Otherwise use actual Frenet
  // d so the vehicle naturally returns to the reference line.
  double d_from_frenet = init_d[0];
  if (driving_mode == DrivingMode::NUDGE && !obstacles.empty() &&
      std::abs(planning_init_point.d0) > 0.05 && std::abs(init_d[0]) < 0.5)
  {
    init_d[0] = planning_init_point.d0;
    init_d[1] = planning_init_point.dd0;
    init_d[2] = planning_init_point.ddd0;
  }
  dbg << "[LATTICE] init_s=[" << init_s[0] << "," << init_s[1] << "," << init_s[2] << "]"
      << " init_d=[" << init_d[0] << "," << init_d[1] << "," << init_d[2] << "]"
      << " d_frenet=" << d_from_frenet
      << " d0_inherited=" << planning_init_point.d0 << std::endl;
  std::cout << "[LATTICE] init_s[0]=" << init_s[0] << " init_d=["
            << init_d[0] << "," << init_d[1] << "," << init_d[2] << "]"
            << " d_frenet=" << d_from_frenet
            << " d0_inherited=" << planning_init_point.d0
            << " mode=" << (driving_mode == DrivingMode::LCC ? "LCC" :
                           driving_mode == DrivingMode::NUDGE ? "NUDGE" : "ALC")
            << std::endl;

  // 与Apollo不同，我们的前探距离不使用lon_decision_horizon，因为我们的仿真的参考线不长
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles, reference_points, init_s[0],
                                                             init_s[0] + 30, // 前瞻多少m lon_decision_horizon
                                                             0.0, Config_.FLAGS_trajectory_time_length, init_d);

  auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_points));
  // 通过预测得到障碍物list
  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(obstacles, ptr_reference_line);
  // 显示预测轨迹
  // visualization_obstacle_trajectory(ptr_path_time_graph, obstacles);

  // 3.生成纵向和横向轨迹
  Trajectory1dGenerator trajectory1d_generator(init_s, init_d, ptr_path_time_graph, ptr_prediction_querier, lateral_optimization);

  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle,
                                                    driving_mode, current_speed, distance_to_obstacle, target_lat_offset);

  std::cout << "[LATTICE] lon_bundle=" << lon_trajectory1d_bundle.size()
            << " lat_bundle=" << lat_trajectory1d_bundle.size()
            << " total_pairs=" << (lon_trajectory1d_bundle.size() * lat_trajectory1d_bundle.size()) << std::endl;

  // 4.计算每条轨迹的代价,并得出优先级队列
  TrajectoryEvaluator trajectory_evaluator(planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
                                           init_s, ptr_path_time_graph, reference_points, preferred_lat_sign,
                                           driving_mode, target_lat_offset);

  // 5.轨迹拼接和最后的筛选
  int checked = 0;
  const int MAX_COLLECT = (all_trajectories != nullptr) ? 200 : 1;
  bool found_optimal = false;

  while (trajectory_evaluator.has_more_trajectory_pairs())
  {
    checked++;

    double trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = trajectorycombiner.Combine(accumulated_s, *trajectory_pair.first, *trajectory_pair.second,
                                                          reference_points, init_relative_time);

    bool constraint_valid = true;
    bool collision_free = true;

    // 采样时候才调用，二次规划不用
    if (lateral_optimization == false)
    {
      // check longitudinal and lateral acceleration
      // considering trajectory curvatures
      auto result = constraintchecker_.ValidTrajectory(combined_trajectory);
      if (result != ConstraintChecker::Result::VALID)
      {
        constraint_valid = false;
      }
      else
      {
        // Get instance of collision checker and constraint checker
        CollisionChecker collision_checker(obstacles, init_s[0], init_d[0], reference_points, ptr_path_time_graph);
        // 碰撞检测
        if (collision_checker.InCollision(combined_trajectory))
        {
          collision_free = false;
        }
      }
    }

    // Collect trajectory results for diagnostics
    if (all_trajectories != nullptr && (int)all_trajectories->size() < MAX_COLLECT)
    {
      all_trajectories->push_back({std::move(combined_trajectory), constraint_valid, collision_free, trajectory_pair_cost});
    }

    if (constraint_valid && collision_free && !found_optimal)
    {
      if (all_trajectories != nullptr && !all_trajectories->empty())
      {
        Optim_trajectory = all_trajectories->back().trajectory;
      }
      else
      {
        Optim_trajectory = std::move(combined_trajectory);
      }
      found_optimal = true;
      if (all_trajectories == nullptr)
        break; // Original behavior: stop at first valid trajectory
    }
  }

  std::cout << "[LATTICE] checked=" << checked << " traj_pairs, result_size=" << Optim_trajectory.size()
            << " collected=" << (all_trajectories ? (int)all_trajectories->size() : -1) << std::endl;
  dbg << "[LATTICE] result: checked=" << checked << " traj_pairs, result_size="
      << Optim_trajectory.size() << " lon_bundle=" << lon_trajectory1d_bundle.size()
      << " lat_bundle=" << lat_trajectory1d_bundle.size() << std::endl;
  dbg.close();

  return Optim_trajectory;
}


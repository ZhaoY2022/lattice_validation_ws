#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pnc_msg/msg/traj_points.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <fstream>
#include <ctime>
#include <cmath>

#include "lattice_planner.h"
#include "path_matcher.h"
#include "path_struct.h"
#include "reference_point.h"
#include "Obstacle.h"
#include "Configs.h"
#include "scenario_manager.h"

class LatticeTestNode : public rclcpp::Node
{
public:
  LatticeTestNode()
    : Node("lattice_test_node")
    , sim_time_(0.0)
    , frame_count_(0)
    , viz_interval_(1)   // publish every frame (10 Hz) to keep green trajectory aligned
  {
    this->declare_parameter("scenario", 1);
    int scenario = this->get_parameter("scenario").as_int();
    RCLCPP_INFO(get_logger(), "Starting Lattice Test Node - Scenario %d", scenario);

    this->declare_parameter("persist_history", false);
    persist_history_ = this->get_parameter("persist_history").as_bool();
    this->declare_parameter("history_frame_interval", 5);
    history_frame_interval_ = this->get_parameter("history_frame_interval").as_int();
    if (persist_history_) {
      RCLCPP_INFO(get_logger(), "History persistence ENABLED (every %d frames)",
                  history_frame_interval_);
    }

    // Load scenario via scenario_manager (shared with simulator)
    auto sc = build_scenario(scenario);
    scenario_ = sc.scenario_id;
    reference_line_ = std::move(sc.reference_line);
    road_reference_line_ = reference_line_;
    accumulated_s_ = std::move(sc.accumulated_s);
    obstacles_ = std::move(sc.obstacles);
    vehicle_state_ = sc.vehicle_state;
    target_speed_ = sc.target_speed;

    // Publishers — QoS depth 5 to reduce flicker from occasional frame drops
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lattice_test/reference_path", rclcpp::QoS(5));
    opt_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lattice_test/optimal_trajectory", rclcpp::QoS(5));
    candidate_trajs_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lattice_test/candidate_trajectories", rclcpp::QoS(5));
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lattice_test/viz_markers", rclcpp::QoS(5));
    ref_line_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lattice_test/ref_line", rclcpp::QoS(5));
    lattice_traj_pub_ = this->create_publisher<pnc_msg::msg::TrajPoints>(
        "/lattice_trajectory", rclcpp::QoS(5));
    road_boundary_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lattice_test/road_boundaries", rclcpp::QoS(5).transient_local());

    // Delay planning start by 1.5s so RViz2 subscribers are ready
    // and DDS discovery completes before road boundaries + reference path are published.
    startup_delay_ = true;
    startup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1500),
        [this]() {
          startup_delay_ = false;
          startup_timer_->cancel();
          startup_timer_.reset();
          publish_reference_path();
          publish_road_boundaries();
          RCLCPP_INFO(get_logger(), "Startup delay complete — beginning planning");
        });

    // 10Hz planning timer (skips work while startup_delay_ is true)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LatticeTestNode::timer_callback, this));

    // Open CSV log for post-analysis
    {
      char filename[128];
      auto t = std::time(nullptr);
      auto tm = std::localtime(&t);
      std::strftime(filename, sizeof(filename),
                    "/tmp/lattice_log_%Y%m%d_%H%M%S.csv", tm);
      csv_log_.open(filename);
      csv_log_ << "timestamp,frame,sim_time,s0,d0,v,mode,result_pts,collected" << std::endl;
      RCLCPP_INFO(get_logger(), "CSV log: %s", filename);
    }

    // Subscribe to interactive lane change commands (scenario 5+)
    lane_change_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/lattice_test/lane_change_cmd", rclcpp::QoS(5),
        std::bind(&LatticeTestNode::lane_change_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Initialized: ref_line=%zu points, veh=(%.1f,%.1f) v=%.1f",
                reference_line_.size(),
                vehicle_state_.x_init, vehicle_state_.y_init,
                vehicle_state_.v_init);
  }

  void lane_change_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    if (driving_mode_ == DrivingMode::ALC && !alc_completed_flag_) {
      RCLCPP_WARN(get_logger(),
          "Lane change command (val=%d) rejected: ALC already in progress", msg->data);
      return;
    }
    if (msg->data != 1 && msg->data != -1) {
      RCLCPP_WARN(get_logger(), "Invalid lane change command: %d (expected -1 or 1)", msg->data);
      return;
    }

    double offset = (msg->data == 1) ? 3.75 : -3.75;
    RCLCPP_INFO(get_logger(),
        "==== Lane change requested: target offset = %.2fm (val=%d) ====", offset, msg->data);

    driving_mode_ = DrivingMode::ALC;
    target_lat_offset_ = offset;
    alc_completed_flag_ = false;
    alc_complete_debounce_counter_ = 0;
  }

private:
  int scenario_;
  double timestep_ = 0.1;
  double target_speed_ = 5.0;
  int frame_count_;
  int viz_interval_;
  bool persist_history_ = false;
  int history_frame_interval_ = 5;

  std::vector<ReferencePoint> reference_line_;
  std::vector<ReferencePoint> road_reference_line_;
  std::vector<double> accumulated_s_;
  std::vector<Obstacle> obstacles_;
  InitialConditions vehicle_state_;
  double sim_time_;

  LatticePlanner planner_;

  // CSV log for post-analysis
  std::ofstream csv_log_;

  // Post-obstacle LCC return-to-center tracking
  bool post_obstacle_lcc_ = false;

  // ALC completion state machine
  bool alc_completed_flag_ = false;
  int alc_complete_debounce_counter_ = 0;
  bool clear_reference_markers_ = false;

  // Persistent driving mode and lateral offset (shared between timer and lane_change callback)
  DrivingMode driving_mode_ = DrivingMode::LCC;
  double target_lat_offset_ = 0.0;

  // Interactive lane change subscription
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr lane_change_sub_;

  // Road boundary publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_boundary_pub_;

  // Startup delay: block planning until RViz subscribers are ready
  bool startup_delay_ = false;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_trajs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ref_line_pub_;
  rclcpp::Publisher<pnc_msg::msg::TrajPoints>::SharedPtr lattice_traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ----------------------------------------------------------------
  // Main loop
  // ----------------------------------------------------------------
  void timer_callback()
  {
    // Skip planning during startup delay (RViz subscribers not ready yet)
    if (startup_delay_) return;

    // Build obstacle pointer vector
    std::vector<const Obstacle*> obstacle_ptrs;
    for (const auto& obs : obstacles_) {
      obstacle_ptrs.push_back(&obs);
    }

    // Build planning target
    PlanningTarget planning_target(
        target_speed_, accumulated_s_.back(), accumulated_s_);

    // ---- Determine driving mode and target_lat_offset from scenario state machine ----
    double preferred_lat_sign = 0.0;
    if (scenario_ == 2)
    {
      if (!post_obstacle_lcc_ && !obstacles_.empty())
      {
        // Compute obstacle s-range and average d from polygon points
        double obs_max_s = 0.0;
        double obs_avg_d = 0.0;
        for (const auto& pt : obstacles_[0].polygon_points)
        {
          auto frenet = PathMatcher::GetPathFrenetCoordinate(reference_line_, pt.x(), pt.y());
          if (frenet.first > obs_max_s) obs_max_s = frenet.first;
          obs_avg_d += frenet.second;
        }
        obs_avg_d /= obstacles_[0].polygon_points.size();  // average d of obstacle center

        // Prefer the side OPPOSITE to the obstacle
        preferred_lat_sign = (obs_avg_d > 0.0) ? -1.0 : 1.0;

        // Trigger when vehicle passes obstacle rear end + 2m buffer
        if (vehicle_state_.s0 > obs_max_s + 2.0)
        {
          post_obstacle_lcc_ = true;
          RCLCPP_INFO(get_logger(), "Passed obstacle max_s=%.1f at ego s=%.1f d=%.2f — switching to LCC",
                      obs_max_s, vehicle_state_.s0, vehicle_state_.d0);
        }
      }
      driving_mode_ = post_obstacle_lcc_ ? DrivingMode::LCC : DrivingMode::NUDGE;
      target_lat_offset_ = 0.0;
      if (post_obstacle_lcc_) preferred_lat_sign = 0.0;  // LCC: no directional bias, natural centering
    }
    else if (scenario_ == 3)
    {
      if (!alc_completed_flag_) {
        driving_mode_ = DrivingMode::ALC;
        target_lat_offset_ = -3.75;
      } else {
        driving_mode_ = DrivingMode::LCC;
        target_lat_offset_ = 0.0;
      }
    }
    else if (scenario_ == 5)
    {
      // S-curve highway: default LCC. ALC mode is set by lane_change_callback().
      // Guard with alc_completed_flag_ (same pattern as scenario 3) so that
      // after ALC completion we force LCC and never re-trigger on stale data.
      if (alc_completed_flag_) {
        driving_mode_ = DrivingMode::LCC;
        target_lat_offset_ = 0.0;
      }
    }
    else  // scenario 1 (default)
    {
      driving_mode_ = DrivingMode::LCC;
      target_lat_offset_ = 0.0;
    }

    // Apply persistent target_lat_offset_ to the planning target
    planning_target.set_target_lat_offset(target_lat_offset_);

    // Compute distance to nearest obstacle for dynamic NUDGE smin
    double distance_to_obstacle = std::numeric_limits<double>::max();
    if (!obstacles_.empty())
    {
      double obs_min_s = std::numeric_limits<double>::max();
      for (const auto& pt : obstacles_[0].polygon_points)
      {
        auto frenet = PathMatcher::GetPathFrenetCoordinate(reference_line_, pt.x(), pt.y());
        if (frenet.first < obs_min_s) obs_min_s = frenet.first;
      }
      distance_to_obstacle = obs_min_s - vehicle_state_.s0;
    }

    // Call lattice planner
    std::vector<LatticeTrajectoryResult> all_trajectories;
    DiscretizedTrajectory best_path = planner_.LatticePlan(
        vehicle_state_,
        planning_target,
        obstacle_ptrs,
        accumulated_s_,
        reference_line_,
        false,  // lateral_optimization = false (sampling-based)
        0.0,    // init_relative_time
        30.0,   // lon_decision_horizon
        &all_trajectories,
        preferred_lat_sign,    // NUDGE: push to side opposite of obstacle; else 0.0
        driving_mode_,
        vehicle_state_.v_init,  // current speed for smin calculation
        distance_to_obstacle,
        planning_target.target_lat_offset()
    );

    // Truncate the optimal trajectory to the first 4.0 seconds.
    // The planner still generates full 8.0s trajectories internally so the
    // quintic polynomial has enough distance to converge smoothly, but we only
    // consume the first 4.0s per cycle to keep the vehicle on a tighter leash.
    {
      double max_time = 4.0 + Config_.FLAGS_numerical_epsilon;
      auto it = best_path.begin();
      while (it != best_path.end() && it->relative_time <= max_time) {
        ++it;
      }
      best_path.erase(it, best_path.end());
    }

    // Publish 40-point resampled trajectory on /lattice_trajectory
    publish_lattice_trajectory(best_path, driving_mode_);

    // Update vehicle state by teleporting to optimal trajectory
    if (best_path.size() > 2) {
      TrajectoryPoint pt = best_path.Evaluate(timestep_);
      update_vehicle_state(pt);
    }
    // If no valid trajectory, vehicle coasts with current state

    // ALC completion check: when lateral error and heading error are small
    // for 5 consecutive frames, shift the reference line to the target lane
    // and switch back to LCC mode (production-grade state transition).
    if (driving_mode_ == DrivingMode::ALC) {
      double target_d = planning_target.target_lat_offset();
      double lat_error = std::abs(vehicle_state_.d0 - target_d);

      ReferencePoint match_pt = PathMatcher::MatchToPath(vehicle_state_.s0, reference_line_);
      double heading_error = std::abs(common::math::NormalizeAngle(
          vehicle_state_.theta_init - match_pt.heading_));

      if (lat_error < 0.20 && heading_error < 0.03) {
        alc_complete_debounce_counter_++;
      } else {
        alc_complete_debounce_counter_ = 0;
      }

      if (alc_complete_debounce_counter_ >= 5) {
        alc_completed_flag_ = true;

        // Shift the entire reference line to the new lane center
        // using the normal direction (perpendicular to heading) — works on curved roads
        for (auto& pt : reference_line_) {
          double nx = -std::sin(pt.heading_);
          double ny =  std::cos(pt.heading_);
          pt.x_ += target_d * nx;
          pt.y_ += target_d * ny;
        }

        // Recompute headings, accumulated_s, kappas, dkappas on the shifted
        // geometry.  On curved roads the shifted points have slightly different
        // arc lengths and curvatures than the original reference line.  Without
        // this the Frenet-Cartesian conversions in later planning cycles use
        // stale data and can produce incorrect lateral offsets.
        {
          std::vector<std::pair<double, double>> xy_points;
          xy_points.reserve(reference_line_.size());
          for (const auto& pt : reference_line_) {
            xy_points.emplace_back(pt.x_, pt.y_);
          }
          std::vector<double> new_headings, new_s, new_kappas, new_dkappas;
          PathMatcher::ComputePathProfile(
              xy_points, &new_headings, &new_s, &new_kappas, &new_dkappas);
          for (size_t i = 0; i < reference_line_.size(); ++i) {
            reference_line_[i].set_heading(new_headings[i]);
            reference_line_[i].set_s(new_s[i]);
            reference_line_[i].set_kappa(new_kappas[i]);
            reference_line_[i].set_dkappa(new_dkappas[i]);
          }
          accumulated_s_.clear();
          accumulated_s_.reserve(reference_line_.size());
          for (const auto& pt : reference_line_) {
            accumulated_s_.push_back(pt.accumulated_s_);
          }
        }

        // Reference path and reference line marker are now published every
        // frame (VOLATILE, 10 Hz) — they pick up the shifted reference_line_
        // automatically on the next publish_all() call later in this callback.
        // Only road boundaries need an explicit republish here since they are
        // normally published once at startup (TRANSIENT_LOCAL, infinite lifetime).
        publish_road_boundaries();
        clear_reference_markers_ = true;

        // Recompute vehicle Frenet state on the shifted reference line.
        // Without this, vehicle_state_.d0 still holds the pre-shift value
        // (-3.75 or +3.75), which would cause the next LCC planning cycle
        // to momentarily see the wrong lateral offset and generate a
        // trajectory that pulls the vehicle back toward the original lane.
        {
          auto frenet = PathMatcher::GetPathFrenetCoordinate(
              reference_line_, vehicle_state_.x_init, vehicle_state_.y_init);
          vehicle_state_.s0 = frenet.first;
          vehicle_state_.ds0 = vehicle_state_.v_init;
          vehicle_state_.d0 = frenet.second;
          vehicle_state_.dd0 = 0.0;
          vehicle_state_.ddd0 = 0.0;
        }

        driving_mode_ = DrivingMode::LCC;
        target_lat_offset_ = 0.0;
        alc_complete_debounce_counter_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "==== [ALC -> LCC] Lane change complete! Ref line shifted by %.2fm, back to LCC (new d0=%.2f) ====",
            target_d, vehicle_state_.d0);
        RCLCPP_INFO(this->get_logger(),
            "==== [DEBUG] After shift: ref_line[0]=(%.3f,%.3f) veh=(%.3f,%.3f) d0=%.3f s0=%.1f ====",
            reference_line_[0].x_, reference_line_[0].y_,
            vehicle_state_.x_init, vehicle_state_.y_init,
            vehicle_state_.d0, vehicle_state_.s0);
      }
    } else {
      alc_complete_debounce_counter_ = 0;
    }

    RCLCPP_INFO(get_logger(), "[frame=%d] s0=%.1f d0=%.2f v=%.1f pos=(%.1f,%.1f) result=%zu collected=%zu mode=%s",
                frame_count_, vehicle_state_.s0, vehicle_state_.d0, vehicle_state_.v_init,
                vehicle_state_.x_init, vehicle_state_.y_init,
                best_path.size(), all_trajectories.size(),
                driving_mode_ == DrivingMode::LCC ? "LCC" : (driving_mode_ == DrivingMode::NUDGE ? "NUDGE" : "ALC"));

    // CSV log for post-analysis
    csv_log_ << this->now().seconds() << ","
             << frame_count_ << ","
             << sim_time_ << ","
             << vehicle_state_.s0 << ","
             << vehicle_state_.d0 << ","
             << vehicle_state_.v_init << ","
             << (driving_mode_ == DrivingMode::LCC ? "LCC" : (driving_mode_ == DrivingMode::NUDGE ? "NUDGE" : "ALC")) << ","
             << best_path.size() << ","
             << all_trajectories.size()
             << std::endl;

    sim_time_ += timestep_;

    // Road boundaries are published once at startup and never again —
    // periodic republishing causes RViz flicker (see reference project pattern).

    // Publish visualization (throttled to every viz_interval_ cycles)
    if ((frame_count_ % viz_interval_) == 0)
    {
      publish_reference_path();
      publish_all(best_path, all_trajectories);
    }

    frame_count_++;
  }

  // ----------------------------------------------------------------
  // Publish 40-point resampled trajectory to /lattice_trajectory
  // ----------------------------------------------------------------
  void publish_lattice_trajectory(const DiscretizedTrajectory& traj, DrivingMode mode)
  {
    if (traj.size() < 2) return;

    pnc_msg::msg::TrajPoints msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.sf_trajectory_cnt = 40;
    msg.sf_trajectory_time_stamp = static_cast<float>(sim_time_);
    msg.sf_trajectory_flag = 0;
    msg.sf_trajectory_front_vehicle_velx = 0.0f;
    msg.sf_trajectory_front_vehicle_accx = 0.0f;

    switch (mode) {
      case DrivingMode::LCC:
        msg.sf_trajectory_driving_lateral_flag = 0;
        msg.sf_trajectory_driving_longitudinal_flag = 0;
        break;
      case DrivingMode::ALC:
        msg.sf_trajectory_driving_lateral_flag = 1;
        msg.sf_trajectory_driving_longitudinal_flag = 0;
        break;
      case DrivingMode::NUDGE:
        msg.sf_trajectory_driving_lateral_flag = 2;
        msg.sf_trajectory_driving_longitudinal_flag = 0;
        break;
    }
    msg.sf_trajectory_active_safety_lateral_flag = 0;
    msg.sf_trajectory_active_safety_longitudinal_flag = 0;

    // Resample to exactly 40 equidistant time points, starting from
    // timestep_ so the trajectory aligns with the current vehicle state
    // (which has already been teleported one timestep forward).
    double t_start = timestep_;
    double t_end = traj.back().relative_time;
    double dt = (t_end - t_start) / 39.0;

    std::vector<TrajectoryPoint> raw_pts(40);
    for (size_t i = 0; i < 40; ++i) {
      raw_pts[i] = traj.Evaluate(t_start + i * dt);
    }

    msg.trajectory_points.resize(40);
    double s_start = raw_pts[0].s;
    for (size_t i = 0; i < 40; ++i) {
      const auto& pt = raw_pts[i];
      auto& out = msg.trajectory_points[i];

      float vx = static_cast<float>(pt.v * std::cos(pt.theta));
      float vy = static_cast<float>(pt.v * std::sin(pt.theta));
      float ax = static_cast<float>(pt.a * std::cos(pt.theta));
      float ay = static_cast<float>(pt.a * std::sin(pt.theta));

      out.time = static_cast<float>(sim_time_ + pt.relative_time);
      out.length = static_cast<float>(pt.s - s_start);
      out.x = static_cast<float>(pt.x);
      out.y = static_cast<float>(pt.y);
      out.vx = vx;
      out.vy = vy;
      out.acc_x = ax;
      out.acc_y = ay;
      out.heading = static_cast<float>(pt.theta);
      out.curvature = static_cast<float>(pt.kappa);
      out.pinch = static_cast<float>(pt.dkappa);
      out.jerk = 0.0f;  // filled below
      out.reserved = 0.0f;
    }

    // Compute tangential jerk via central/forward/backward finite differences
    for (size_t i = 0; i < 40; ++i) {
      float da_dt;
      if (i == 0) {
        // Forward difference
        da_dt = (raw_pts[1].a - raw_pts[0].a) / static_cast<float>(dt);
      } else if (i == 39) {
        // Backward difference
        da_dt = (raw_pts[39].a - raw_pts[38].a) / static_cast<float>(dt);
      } else {
        // Central difference
        da_dt = (raw_pts[i+1].a - raw_pts[i-1].a) / static_cast<float>(2.0 * dt);
      }
      msg.trajectory_points[i].jerk = da_dt;
    }

    lattice_traj_pub_->publish(msg);
  }

  void update_vehicle_state(const TrajectoryPoint& pt)
  {
    vehicle_state_.x_init = pt.x;
    vehicle_state_.y_init = pt.y;
    vehicle_state_.z_init = 0.0;
    vehicle_state_.theta_init = pt.theta;
    vehicle_state_.kappa_init = pt.kappa;
    vehicle_state_.v_init = pt.v;
    vehicle_state_.a_init = pt.a;
    vehicle_state_.d0 = pt.d;
    vehicle_state_.dd0 = pt.d_d;
    vehicle_state_.ddd0 = pt.d_dd;
    vehicle_state_.s0 = pt.s;
    vehicle_state_.ds0 = pt.s_d;
    vehicle_state_.dds0 = pt.s_dd;
    vehicle_state_.init_relative_time = sim_time_;
  }

  const char* mode_to_string(DrivingMode mode) const
  {
    switch (mode) {
      case DrivingMode::LCC:
        return "LCC";
      case DrivingMode::ALC:
        return "ALC";
      case DrivingMode::NUDGE:
        return "NUDGE";
    }
    return "UNKNOWN";
  }

  // ----------------------------------------------------------------
  // Visualization
  // ----------------------------------------------------------------
  void publish_reference_path()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    const int kSubsample = 5;  // 8401 → ~1680 poses, avoids DDS/RViz2 large-message issues
    for (size_t i = 0; i < reference_line_.size(); i += kSubsample) {
      const auto& rp = reference_line_[i];
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = rp.x_;
      ps.pose.position.y = rp.y_;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, rp.heading_);
      ps.pose.orientation = tf2::toMsg(q);
      msg.poses.push_back(ps);
    }
    ref_path_pub_->publish(msg);
  }

  void publish_road_boundaries()
  {
    auto now = this->now();
    visualization_msgs::msg::MarkerArray ma;

    // Clear any stale road markers from previous DDS sessions
    {
      visualization_msgs::msg::Marker del_all;
      del_all.header.stamp = now;
      del_all.header.frame_id = "map";
      del_all.ns = "road";
      del_all.id = 0;
      del_all.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(del_all);
    }

    // Helper to build a LINE_STRIP road marker with persistent lifetime.
    auto make_line = [&](int id, double y_offset, double r, double g, double b, double a,
                         double line_width) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = now;
      m.header.frame_id = "map";
      m.ns = "road";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = line_width;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
      m.lifetime = rclcpp::Duration::max();  // infinite
      m.frame_locked = true;
      const auto& road_line = road_reference_line_.empty() ? reference_line_ : road_reference_line_;
      for (const auto& rp : road_line) {
        double nx = -std::sin(rp.heading_);
        double ny =  std::cos(rp.heading_);
        geometry_msgs::msg::Point p;
        p.x = rp.x_ + y_offset * nx;
        p.y = rp.y_ + y_offset * ny;
        p.z = 0.10;
        m.points.push_back(p);
      }
      return m;
    };

    // Helper to build a dashed LINE_LIST marker (dash/gap pattern along ref line).
    auto make_dashed_line = [&](int id, double y_offset, double r, double g, double b, double a,
                                double line_width) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = now;
      m.header.frame_id = "map";
      m.ns = "road";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = line_width;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
      m.lifetime = rclcpp::Duration::max();  // infinite
      m.frame_locked = true;

      const double dash_len = 3.0;
      const double gap_len = 3.0;
      const double cycle = dash_len + gap_len;
      double dist = 0.0;

      const auto& road_line = road_reference_line_.empty() ? reference_line_ : road_reference_line_;
      for (size_t i = 0; i + 1 < road_line.size(); ++i) {
        double dx = road_line[i + 1].x_ - road_line[i].x_;
        double dy = road_line[i + 1].y_ - road_line[i].y_;
        double seg_len = std::sqrt(dx * dx + dy * dy);

        double dist_in_cycle = std::fmod(dist, cycle);
        if (dist_in_cycle < dash_len) {
          double nx0 = -std::sin(road_line[i].heading_);
          double ny0 =  std::cos(road_line[i].heading_);
          double nx1 = -std::sin(road_line[i + 1].heading_);
          double ny1 =  std::cos(road_line[i + 1].heading_);
          geometry_msgs::msg::Point p0, p1;
          p0.x = road_line[i].x_ + y_offset * nx0;
          p0.y = road_line[i].y_ + y_offset * ny0;
          p0.z = 0.10;
          p1.x = road_line[i + 1].x_ + y_offset * nx1;
          p1.y = road_line[i + 1].y_ + y_offset * ny1;
          p1.z = 0.10;
          m.points.push_back(p0);
          m.points.push_back(p1);
        }
        dist += seg_len;
      }
      return m;
    };

    if (scenario_ == 3 || scenario_ == 5) {
      // Two-lane display for ALC / interactive lane change.
      // Source lane: center at d=0, target lane: center at d=-3.75.
      // Lane width = 3.75m → half_width = 1.875m for tight seam at divider.
      const double hw = 3.75 / 2.0;
      const double tgt = -3.75;

      // 1. Right boundary of source lane (outermost right, solid white)
      ma.markers.push_back(make_line(0, +hw, 1.0, 1.0, 1.0, 1.0, 0.20));
      // 2. Lane divider between source and target lanes (dashed look, semi-transparent)
      ma.markers.push_back(make_line(1, -hw, 1.0, 1.0, 1.0, 0.6, 0.12));
      // 3. Left boundary of target lane (outermost left, solid white)
      ma.markers.push_back(make_line(2, tgt - hw, 1.0, 1.0, 1.0, 1.0, 0.20));
      // 4. Source lane center line (yellow, dashed)
      ma.markers.push_back(make_dashed_line(3, 0.0, 1.0, 1.0, 0.0, 0.5, 0.10));
      // 5. Target lane center line (yellow, dashed)
      ma.markers.push_back(make_dashed_line(4, tgt, 1.0, 1.0, 0.0, 0.5, 0.10));
    } else {
      // Standard single-lane display for LCC and NUDGE.
      const double hw = 2.0;

      // 1. Left boundary (white solid)
      ma.markers.push_back(make_line(0, +hw, 1.0, 1.0, 1.0, 1.0, 0.25));
      // 2. Right boundary (white solid)
      ma.markers.push_back(make_line(1, -hw, 1.0, 1.0, 1.0, 1.0, 0.25));
      // 3. Center line (yellow, dashed)
      ma.markers.push_back(make_dashed_line(2, 0.0, 1.0, 1.0, 0.0, 0.8, 0.15));
    }

    road_boundary_pub_->publish(ma);
  }

  void publish_all(const DiscretizedTrajectory& opt_traj,
                   const std::vector<LatticeTrajectoryResult>& all_trajs)
  {
    auto now = this->now();
    const int kSubsample = 3;  // keep every 3rd point to reduce geometry
    const bool has_opt_traj = !opt_traj.empty();

    // 1. Optimal trajectory as Path (green) — evaluate from current vehicle time
    {
      nav_msgs::msg::Path msg;
      msg.header.stamp = now;
      msg.header.frame_id = "map";
      if (has_opt_traj) {
        double t_end = opt_traj.back().relative_time;
        for (double t = timestep_; t <= t_end + 1e-6; t += 0.05) {
          TrajectoryPoint pt = opt_traj.Evaluate(t);
          geometry_msgs::msg::PoseStamped ps;
          ps.header = msg.header;
          ps.pose.position.x = pt.x;
          ps.pose.position.y = pt.y;
          ps.pose.position.z = 0.0;
          tf2::Quaternion q;
          q.setRPY(0, 0, pt.theta);
          ps.pose.orientation = tf2::toMsg(q);
          msg.poses.push_back(ps);
        }
      }
      opt_traj_pub_->publish(msg);
    }

    // 2. Candidate trajectories — publish empty to clear the Path display
    {
      nav_msgs::msg::Path msg;
      msg.header.stamp = now;
      msg.header.frame_id = "map";
      candidate_trajs_pub_->publish(msg);
    }

    // 3. MarkerArray
    {
      visualization_msgs::msg::MarkerArray ma;

      // On first frame, clear ALL namespaces used by this display to remove
      // any stale markers lingering from a previous run in DDS shared memory.
      if (frame_count_ == 0) {
        for (const char* ns : {"ego", "obstacles", "reference", "candidates", "optimal",
                               "mode_status", "history_candidates", "history_optimal"}) {
          visualization_msgs::msg::Marker del;
          del.header.stamp = now;
          del.header.frame_id = "map";
          del.ns = ns;
          del.id = 0;
          del.action = visualization_msgs::msg::Marker::DELETEALL;
          ma.markers.push_back(del);
        }
      }

      // 3a. Explicitly DELETE any stale ego CUBE marker from previous runs
      {
        visualization_msgs::msg::Marker del_ego;
        del_ego.header.stamp = now;
        del_ego.header.frame_id = "map";
        del_ego.ns = "ego";
        del_ego.id = 0;
        del_ego.action = visualization_msgs::msg::Marker::DELETE;
        ma.markers.push_back(del_ego);
      }
      // Ego vehicle is now rendered as URDF RobotModel (no CUBE needed).

      // 3b. Current driving mode label — shown above the vehicle in RViz.
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = "map";
        m.ns = "mode_status";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = vehicle_state_.x_init + 1;
        m.pose.position.y = vehicle_state_.y_init + 2;
        m.pose.position.z = 3;
        m.pose.orientation.w = 1.0;
        m.scale.z = 1;
        m.text = mode_to_string(driving_mode_);
        if (driving_mode_ == DrivingMode::ALC) {
          m.color.r = 1.0;
          m.color.g = 0.8;
          m.color.b = 0.0;
        } else if (driving_mode_ == DrivingMode::NUDGE) {
          m.color.r = 1.0;
          m.color.g = 0.35;
          m.color.b = 0.0;
        } else {
          m.color.r = 0.35;
          m.color.g = 0.75;
          m.color.b = 1.0;
        }
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(0.3);
        m.frame_locked = false;
        ma.markers.push_back(m);
      }

      // 3c. Obstacles — publish once at frame 0 (persistent, like reference line).
      // Re-publishing every frame at 10Hz causes visual flickering, especially
      // when candidate trajectories near the obstacle toggled between
      // collision-free (cyan) and in-collision (orange) each cycle.
      if (frame_count_ == 0) {
        const int MAX_OBS = 20;
        int obs_id = 0;
        for (const auto& obs : obstacles_) {
          visualization_msgs::msg::Marker m;
          m.header.stamp = now;
          m.header.frame_id = "map";
          m.ns = "obstacles";
          m.id = obs_id;
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.pose.position.x = obs.centerpoint.position.x;
          m.pose.position.y = obs.centerpoint.position.y;
          m.pose.position.z = obs.obstacle_height / 2.0;
          m.pose.orientation.w = 1.0;
          m.scale.x = obs.obstacle_length;
          m.scale.y = obs.obstacle_width;
          m.scale.z = obs.obstacle_height;
          m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.5;
          m.lifetime = rclcpp::Duration::max();  // persistent
          m.frame_locked = true;
          ma.markers.push_back(m);
          ++obs_id;
        }
        // DELETE any unused obstacle IDs so stale markers from previous
        // runs (which may have had more obstacles) don't linger.
        for (int del_id = obs_id; del_id < MAX_OBS; ++del_id) {
          visualization_msgs::msg::Marker del;
          del.header.stamp = now;
          del.header.frame_id = "map";
          del.ns = "obstacles";
          del.id = del_id;
          del.action = visualization_msgs::msg::Marker::DELETE;
          ma.markers.push_back(del);
        }
      }

      // 3d. Reference line as LINE_STRIP — published every frame (like
      // optimal trajectory) so RViz2 always shows the current position.
      // Uses a rotating marker ID (0..99) each frame so RViz2 treats each
      // publish as a fresh marker rather than a no-op "update" to the same ID.
      // On frame 0 a DELETEALL clears stale markers from previous runs.
      {
        visualization_msgs::msg::MarkerArray ref_ma;
        if (frame_count_ == 0 || clear_reference_markers_) {
          visualization_msgs::msg::Marker del_all;
          del_all.header.stamp = now;
          del_all.header.frame_id = "map";
          del_all.ns = "reference";
          del_all.id = 0;
          del_all.action = visualization_msgs::msg::Marker::DELETEALL;
          ref_ma.markers.push_back(del_all);
        }
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = "map";
        m.ns = "reference";
        m.id = static_cast<int>(frame_count_ % 100);
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.08;
        m.color.r = 0.0; m.color.g = 0.4; m.color.b = 1.0; m.color.a = 0.6;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        m.frame_locked = false;
        for (size_t i = 0; i < reference_line_.size(); i += kSubsample) {
          geometry_msgs::msg::Point p;
          p.x = reference_line_[i].x_; p.y = reference_line_[i].y_; p.z = 0.0;
          m.points.push_back(p);
        }
        ref_ma.markers.push_back(m);
        ref_line_pub_->publish(ref_ma);
        clear_reference_markers_ = false;
      }

      // 3e. Candidate trajectories — render BEFORE optimal so green line is on top
      {
        const int MAX_CANDIDATES = 25;
        int cand_id = 0;
        for (const auto& result : all_trajs) {
          if (cand_id >= MAX_CANDIDATES) break;
          visualization_msgs::msg::Marker m;
          m.header.stamp = now;
          m.header.frame_id = "map";
          m.ns = "candidates";
          m.id = cand_id;
          m.type = visualization_msgs::msg::Marker::LINE_STRIP;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.08;
          m.lifetime = rclcpp::Duration::from_seconds(2.0);
          if (result.constraint_valid && result.collision_free) {
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.8;
          } else if (result.constraint_valid) {
            m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.65;
          } else {
            m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.35;
          }
          // z=0.02 so candidates float above ground plane, distinguishable from green optimal
          for (size_t i = 0; i < result.trajectory.size(); i += kSubsample) {
            geometry_msgs::msg::Point p;
            p.x = result.trajectory[i].x; p.y = result.trajectory[i].y; p.z = 0.02;
            m.points.push_back(p);
          }
          if (!result.trajectory.empty() && (result.trajectory.size() - 1) % kSubsample != 0) {
            geometry_msgs::msg::Point p;
            p.x = result.trajectory.back().x;
            p.y = result.trajectory.back().y;
            p.z = 0.02;
            m.points.push_back(p);
          }
          ma.markers.push_back(m);
          ++cand_id;
        }
        // Explicitly DELETE remaining candidate IDs so stale markers
        // from previous frames don't linger for their full lifetime.
        for (int del_id = cand_id; del_id < MAX_CANDIDATES; ++del_id) {
          visualization_msgs::msg::Marker del;
          del.header.stamp = now;
          del.header.frame_id = "map";
          del.ns = "candidates";
          del.id = del_id;
          del.action = visualization_msgs::msg::Marker::DELETE;
          ma.markers.push_back(del);
        }
      }

      // 3f. Optimal trajectory — evaluate from current vehicle time
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = "map";
        m.ns = "optimal";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.12;
        bool valid = opt_traj.size() > 1;
        m.color.r = valid ? 0.0 : 1.0;
        m.color.g = valid ? 1.0 : 0.0;
        m.color.b = 0.0;
        m.color.a = 0.9;
        m.lifetime = rclcpp::Duration::from_seconds(2.0);
        if (has_opt_traj) {
          double t_end = opt_traj.back().relative_time;
          for (double t = timestep_; t <= t_end + 1e-6; t += 0.1) {
            TrajectoryPoint pt = opt_traj.Evaluate(t);
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = 0.0;
            m.points.push_back(p);
          }
        }
        ma.markers.push_back(m);
      }

      // 3g. History snapshots — persist with infinite lifetime for post-run review
      if (persist_history_ && (frame_count_ % history_frame_interval_ == 0)) {
        const int HISTORY_ID_OFFSET = 1000000;
        const int MAX_HIST_CANDIDATES = 50;

        // History candidate trajectories
        int hist_id = 0;
        for (const auto& result : all_trajs) {
          if (hist_id >= MAX_HIST_CANDIDATES) break;
          visualization_msgs::msg::Marker m;
          m.header.stamp = now;
          m.header.frame_id = "map";
          m.ns = "history_candidates";
          m.id = HISTORY_ID_OFFSET + frame_count_ * MAX_HIST_CANDIDATES + hist_id;
          m.type = visualization_msgs::msg::Marker::LINE_STRIP;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.06;
          m.lifetime = rclcpp::Duration::from_seconds(0.0);  // infinite
          if (result.constraint_valid && result.collision_free) {
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.5;
          } else if (result.constraint_valid) {
            m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.4;
          } else {
            m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.2;
          }
          for (size_t i = 0; i < result.trajectory.size(); i += kSubsample) {
            geometry_msgs::msg::Point p;
            p.x = result.trajectory[i].x; p.y = result.trajectory[i].y; p.z = 0.04;
            m.points.push_back(p);
          }
          if (!result.trajectory.empty() && (result.trajectory.size() - 1) % kSubsample != 0) {
            geometry_msgs::msg::Point p;
            p.x = result.trajectory.back().x;
            p.y = result.trajectory.back().y;
            p.z = 0.04;
            m.points.push_back(p);
          }
          ma.markers.push_back(m);
          ++hist_id;
        }

        // History optimal trajectory
        {
          visualization_msgs::msg::Marker m;
          m.header.stamp = now;
          m.header.frame_id = "map";
          m.ns = "history_optimal";
          m.id = HISTORY_ID_OFFSET + frame_count_;
          m.type = visualization_msgs::msg::Marker::LINE_STRIP;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.10;
          m.color.r = 0.0; m.color.g = 0.8; m.color.b = 0.0; m.color.a = 0.7;
          m.lifetime = rclcpp::Duration::from_seconds(0.0);
          for (size_t i = 0; i < opt_traj.size(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = opt_traj[i].x; p.y = opt_traj[i].y; p.z = 0.04;
            m.points.push_back(p);
          }
          ma.markers.push_back(m);
        }
      }

      viz_pub_->publish(ma);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LatticeTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

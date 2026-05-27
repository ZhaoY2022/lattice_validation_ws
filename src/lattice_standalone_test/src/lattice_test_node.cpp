#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <fstream>
#include <ctime>

#include "lattice_planner.h"
#include "path_matcher.h"
#include "path_struct.h"
#include "reference_point.h"
#include "Obstacle.h"
#include "Configs.h"

class LatticeTestNode : public rclcpp::Node
{
public:
  LatticeTestNode()
    : Node("lattice_test_node")
    , sim_time_(0.0)
    , frame_count_(0)
    , viz_interval_(1)   // publish every frame (10 Hz) — no throttling needed
  {
    this->declare_parameter("scenario", 1);
    int scenario = this->get_parameter("scenario").as_int();
    RCLCPP_INFO(get_logger(), "Starting Lattice Test Node - Scenario %d", scenario);

    init_scenario(scenario);

    // Publishers — QoS depth 5 to reduce flicker from occasional frame drops
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lattice_test/reference_path", rclcpp::QoS(5).transient_local());
    opt_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lattice_test/optimal_trajectory", rclcpp::QoS(5));
    candidate_trajs_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/lattice_test/candidate_trajectories", rclcpp::QoS(5));
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lattice_test/viz_markers", rclcpp::QoS(5));

    // 10Hz planning timer (visualization throttled by frame_count_)
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

    // Publish static reference path once
    publish_reference_path();

    RCLCPP_INFO(get_logger(), "Initialized: ref_line=%zu points, veh=(%.1f,%.1f) v=%.1f",
                reference_line_.size(),
                vehicle_state_.x_init, vehicle_state_.y_init,
                vehicle_state_.v_init);
  }

private:
  int scenario_;
  double timestep_ = 0.1;
  double target_speed_ = 5.0;
  int frame_count_;
  int viz_interval_;

  std::vector<ReferencePoint> reference_line_;
  std::vector<double> accumulated_s_;
  std::vector<Obstacle> obstacles_;
  InitialConditions vehicle_state_;
  double sim_time_;

  LatticePlanner planner_;

  // CSV log for post-analysis
  std::ofstream csv_log_;

  // Post-obstacle LCC return-to-center tracking
  bool post_obstacle_lcc_ = false;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_trajs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ----------------------------------------------------------------
  // Scenario initialization
  // ----------------------------------------------------------------
  void init_scenario(int scenario)
  {
    scenario_ = scenario;
    switch (scenario) {
      case 1: init_scenario_1(); break;
      case 2: init_scenario_2(); break;
      case 3: init_scenario_3(); break;
      default: init_scenario_1(); break;
    }
  }

  void build_straight_ref_line(double x_start, double x_end, double y_const,
                               double resolution = 0.5)
  {
    reference_line_.clear();
    accumulated_s_.clear();
    double s = 0.0;
    for (double x = x_start; x <= x_end; x += resolution) {
      reference_line_.push_back(
          ReferencePoint(0.0, 0.0, x, y_const, 0.0, s));
      accumulated_s_.push_back(s);
      s += resolution;
    }
  }

  void init_scenario_1()
  {
    // Lateral offset recovery: straight line at y=0, vehicle offset at y=0.5
    build_straight_ref_line(0.0, 500.0, 0.0);
    target_speed_ = 22.0;

    vehicle_state_.x_init = 0.0;
    vehicle_state_.y_init = 0.5;
    vehicle_state_.z_init = 0.0;
    vehicle_state_.theta_init = 0.0;
    vehicle_state_.kappa_init = 0.0;
    vehicle_state_.dkappa_init = 0.0;
    vehicle_state_.v_init = 22.0;
    vehicle_state_.a_init = 0.0;
    vehicle_state_.d0 = 0.5;
    vehicle_state_.dd0 = 0.0;
    vehicle_state_.ddd0 = 0.0;
    vehicle_state_.s0 = 0.0;
    vehicle_state_.ds0 = 22.0;
    vehicle_state_.dds0 = 0.0;
    vehicle_state_.init_relative_time = 0.0;
    obstacles_.clear();
  }

  void init_scenario_2()
  {
    // Static obstacle avoidance
    build_straight_ref_line(0.0, 500.0, 0.0);
    target_speed_ = 22.0;

    vehicle_state_.x_init = 0.0;
    vehicle_state_.y_init = 0.0;
    vehicle_state_.z_init = 0.0;
    vehicle_state_.theta_init = 0.0;
    vehicle_state_.kappa_init = 0.0;
    vehicle_state_.dkappa_init = 0.0;
    vehicle_state_.v_init = 22.0;
    vehicle_state_.a_init = 0.0;
    vehicle_state_.d0 = 0.0;
    vehicle_state_.dd0 = 0.0;
    vehicle_state_.ddd0 = 0.0;
    vehicle_state_.s0 = 0.0;
    vehicle_state_.ds0 = 22.0;
    vehicle_state_.dds0 = 0.0;
    vehicle_state_.init_relative_time = 0.0;

    // Place static obstacle at x=30, y=0
    obstacles_.clear();
    Obstacle obs;
    obs.obstacle_id = "static_obs_1";
    obs.obstacle_type = 1;   // CAR
    obs.obstacle_shape = 0;  // BOUNDING_BOX
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

    // Set polygon points for collision checking (aligned with centerpoint y=-1.6)
    double hw = obs.obstacle_width / 2.0;
    double hl = obs.obstacle_length / 2.0;
    obs.polygon_points = {
        Vec2d(95.0 - hl, -1.6 - hw), Vec2d(95.0 + hl, -1.6 - hw),
        Vec2d(95.0 + hl, -1.6 + hw), Vec2d(95.0 - hl, -1.6 + hw)
    };
    obs.SetPerceptionBoundingBox(
        Box2d(Vec2d(95.0, -1.6), 0.0, obs.obstacle_length, obs.obstacle_width));
    obs.SetPerceptionPolygon(common::math::Polygon2d(obs.polygon_points));

    // Pinnacle for visualization
    for (auto& pt : obs.polygon_points) {
      geometry_msgs::msg::Pose p;
      p.position.x = pt.x();
      p.position.y = pt.y();
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      obs.pinnacle.poses.push_back(p);
    }

    obstacles_.push_back(obs);
  }

  void init_scenario_3()
  {
    // Lane change: straight reference line y=0.
    // ALC mode uses target_lat_offset = -3.75 to generate trajectories that
    // transition from the current lane (d=0) to the target lane (d=-3.75).
    // This is more realistic: in real driving you don't get a pre-curved
    // reference line — the planner must generate the lane-change path.
    build_straight_ref_line(0.0, 500.0, 0.0);
    target_speed_ = 22.0;

    vehicle_state_.x_init = 0.0;
    vehicle_state_.y_init = 0.0;
    vehicle_state_.z_init = 0.0;
    vehicle_state_.theta_init = 0.0;
    vehicle_state_.kappa_init = 0.0;
    vehicle_state_.dkappa_init = 0.0;
    vehicle_state_.v_init = 22.0;
    vehicle_state_.a_init = 0.0;
    vehicle_state_.d0 = 0.0;
    vehicle_state_.dd0 = 0.0;
    vehicle_state_.ddd0 = 0.0;
    vehicle_state_.s0 = 0.0;
    vehicle_state_.ds0 = 22.0;
    vehicle_state_.dds0 = 0.0;
    vehicle_state_.init_relative_time = 0.0;
    obstacles_.clear();
  }

  // ----------------------------------------------------------------
  // Main loop
  // ----------------------------------------------------------------
  void timer_callback()
  {
    // Build obstacle pointer vector
    std::vector<const Obstacle*> obstacle_ptrs;
    for (const auto& obs : obstacles_) {
      obstacle_ptrs.push_back(&obs);
    }

    // Build planning target
    PlanningTarget planning_target(
        target_speed_, accumulated_s_.back(), accumulated_s_);

    // Scenario 3 (ALC): set target lane offset for realistic lane change.
    // The planner will generate trajectories from current d≈0 to d≈-3.75.
    if (scenario_ == 3) {
      planning_target.set_target_lat_offset(-3.75);
    }

    // Determine driving mode (with post-obstacle LCC return-to-center for S2)
    DrivingMode driving_mode = DrivingMode::LCC;
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
        // obs_avg_d > 0 (obstacle right) → preferred_lat_sign = -1 (go left)
        // obs_avg_d < 0 (obstacle left)  → preferred_lat_sign = +1 (go right)
        preferred_lat_sign = (obs_avg_d > 0.0) ? -1.0 : 1.0;

        // Trigger when vehicle passes obstacle rear end + 2m buffer
        if (vehicle_state_.s0 > obs_max_s + 2.0)
        {
          post_obstacle_lcc_ = true;
          RCLCPP_INFO(get_logger(), "Passed obstacle max_s=%.1f at ego s=%.1f d=%.2f — switching to LCC",
                      obs_max_s, vehicle_state_.s0, vehicle_state_.d0);
        }
      }
      driving_mode = post_obstacle_lcc_ ? DrivingMode::LCC : DrivingMode::NUDGE;
      if (post_obstacle_lcc_) preferred_lat_sign = 0.0;  // LCC: no directional bias, natural centering
    }
    else if (scenario_ == 3)
      driving_mode = DrivingMode::ALC;

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
        driving_mode,
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

    // Update vehicle state by teleporting to optimal trajectory
    if (best_path.size() > 2) {
      TrajectoryPoint pt = best_path.Evaluate(timestep_);
      update_vehicle_state(pt);
    }
    // If no valid trajectory, vehicle coasts with current state

    RCLCPP_INFO(get_logger(), "[frame=%d] s0=%.1f d0=%.2f v=%.1f result=%zu collected=%zu mode=%s",
                frame_count_, vehicle_state_.s0, vehicle_state_.d0, vehicle_state_.v_init,
                best_path.size(), all_trajectories.size(),
                driving_mode == DrivingMode::LCC ? "LCC" : (driving_mode == DrivingMode::NUDGE ? "NUDGE" : "ALC"));

    // CSV log for post-analysis
    csv_log_ << this->now().seconds() << ","
             << frame_count_ << ","
             << sim_time_ << ","
             << vehicle_state_.s0 << ","
             << vehicle_state_.d0 << ","
             << vehicle_state_.v_init << ","
             << (driving_mode == DrivingMode::LCC ? "LCC" : (driving_mode == DrivingMode::NUDGE ? "NUDGE" : "ALC")) << ","
             << best_path.size() << ","
             << all_trajectories.size()
             << std::endl;

    sim_time_ += timestep_;

    // Publish visualization (throttled to every viz_interval_ cycles)
    if ((frame_count_ % viz_interval_) == 0)
    {
      publish_all(best_path, all_trajectories);
    }
    frame_count_++;
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

  // ----------------------------------------------------------------
  // Visualization
  // ----------------------------------------------------------------
  void publish_reference_path()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    for (const auto& rp : reference_line_) {
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

  void publish_all(const DiscretizedTrajectory& opt_traj,
                   const std::vector<LatticeTrajectoryResult>& all_trajs)
  {
    auto now = this->now();
    const int kSubsample = 3;  // keep every 3rd point to reduce geometry

    // 1. Optimal trajectory as Path (green)
    {
      nav_msgs::msg::Path msg;
      msg.header.stamp = now;
      msg.header.frame_id = "map";
      for (size_t i = 0; i < opt_traj.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = opt_traj[i].x;
        ps.pose.position.y = opt_traj[i].y;
        ps.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, opt_traj[i].theta);
        ps.pose.orientation = tf2::toMsg(q);
        msg.poses.push_back(ps);
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

    // 3. MarkerArray — ADD-only, ego+obstacle first so RViz renders them before candidate LINE_STRIPs
    {
      visualization_msgs::msg::MarkerArray ma;

      // 3a. Ego vehicle — unique ID per frame so all historical poses persist
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = "map";
        m.ns = "ego";
        m.id = frame_count_;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = vehicle_state_.x_init;
        m.pose.position.y = vehicle_state_.y_init;
        m.pose.position.z = 0.75;
        tf2::Quaternion q;
        q.setRPY(0, 0, vehicle_state_.theta_init);
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 3.0; m.scale.y = 1.6; m.scale.z = 1.5;
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.9;
        m.lifetime = rclcpp::Duration::from_seconds(0.3);
        ma.markers.push_back(m);
      }

      // 3b. Obstacles (persistent — fixed ID, no lifetime)
      {
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
          ma.markers.push_back(m);
          ++obs_id;
        }
        if (obstacles_.empty()) {
          visualization_msgs::msg::Marker m;
          m.header.stamp = now;
          m.header.frame_id = "map";
          m.ns = "obstacles";
          m.id = 0;
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.0; m.scale.y = 0.0; m.scale.z = 0.0;
          m.color.a = 0.0;
          ma.markers.push_back(m);
        }
      }

      // 3c. Reference line as LINE_STRIP (persistent fixed ID, 0.5s lifetime)
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = "map";
        m.ns = "reference";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.08;
        m.color.r = 0.0; m.color.g = 0.4; m.color.b = 1.0; m.color.a = 0.6;
        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        for (size_t i = 0; i < reference_line_.size(); i += kSubsample) {
          geometry_msgs::msg::Point p;
          p.x = reference_line_[i].x_; p.y = reference_line_[i].y_; p.z = 0.0;
          m.points.push_back(p);
        }
        ma.markers.push_back(m);
      }

      // 3d. Candidate trajectories — render BEFORE optimal so green line is on top
      {
        const int MAX_CANDIDATES = 50;
        int cand_id = 0;
        for (const auto& result : all_trajs) {
          if (cand_id >= MAX_CANDIDATES) break;
          visualization_msgs::msg::Marker m;
          m.header.stamp = now;
          m.header.frame_id = "map";
          m.ns = "candidates";
          m.id = frame_count_ * MAX_CANDIDATES + cand_id;
          m.type = visualization_msgs::msg::Marker::LINE_STRIP;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.08;
          m.lifetime = rclcpp::Duration::from_seconds(0.3);
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
      }

      // 3e. Optimal trajectory — renders on top of candidates (drawn last)
      {
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = "map";
        m.ns = "optimal";
        m.id = frame_count_;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.12;
        bool valid = !opt_traj.empty();
        m.color.r = valid ? 0.0 : 1.0;
        m.color.g = valid ? 1.0 : 0.0;
        m.color.b = 0.0;
        m.color.a = 0.9;
        m.lifetime = rclcpp::Duration::from_seconds(0.3);
        for (size_t i = 0; i < opt_traj.size(); ++i) {
          geometry_msgs::msg::Point p;
          p.x = opt_traj[i].x; p.y = opt_traj[i].y; p.z = 0.0;
          m.points.push_back(p);
        }
        ma.markers.push_back(m);
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

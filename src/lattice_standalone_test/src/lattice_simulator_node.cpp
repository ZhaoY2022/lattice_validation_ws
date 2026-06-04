#include <rclcpp/rclcpp.hpp>
#include <pnc_msg/msg/traj_points.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class LatticeSimulatorNode : public rclcpp::Node
{
public:
  LatticeSimulatorNode()
    : Node("lattice_simulator_node")
    , current_speed_(0.0)
    , frame_count_(0)
    , first_traj_received_(false)
    , traj_t0_(0.0)
  {
    this->declare_parameter("init_x", 0.0);
    this->declare_parameter("init_y", 0.0);
    this->declare_parameter("init_heading", 0.0);
    pos_x_ = this->get_parameter("init_x").as_double();
    pos_y_ = this->get_parameter("init_y").as_double();
    heading_ = this->get_parameter("init_heading").as_double();

    node_start_time_ = this->now();
    init_x_ = pos_x_;
    init_y_ = pos_y_;

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    traj_sub_ = this->create_subscription<pnc_msg::msg::TrajPoints>(
        "/lattice_trajectory", rclcpp::QoS(5),
        std::bind(&LatticeSimulatorNode::trajectory_callback, this, std::placeholders::_1));

    // 100 Hz for smooth TF motion
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&LatticeSimulatorNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Lattice Simulator Node started (init: %.1f, %.1f, hdg=%.2f)",
                pos_x_, pos_y_, heading_);
  }

private:
  void trajectory_callback(const pnc_msg::msg::TrajPoints::SharedPtr msg)
  {
    if (rclcpp::Time(msg->header.stamp) < node_start_time_) {
      return;
    }

    if (msg->trajectory_points.empty()) return;

    // Reject stale trajectories whose starting position is far from the
    // current tracking position (handles zombie processes from prior runs
    // that are still publishing on the same topic).
    {
      double dx = msg->trajectory_points[0].x - pos_x_;
      double dy = msg->trajectory_points[0].y - pos_y_;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist > 50.0) {
        RCLCPP_WARN(get_logger(),
            "Rejected stale trajectory: pos0=(%.1f,%.1f) is %.1fm from current=(%.1f,%.1f)",
            msg->trajectory_points[0].x, msg->trajectory_points[0].y,
            dist, pos_x_, pos_y_);
        return;
      }
    }

    // pts[0].time is now an absolute planner time (sim_time + relative_time).
    // Record wall-clock arrival so timer_callback can add elapsed wall time
    // to stay synchronised with the planner (both advance at 1.0 s / wall-s).
    traj_arrival_time_ = this->now();
    traj_t0_ = static_cast<double>(msg->trajectory_points[0].time);
    latest_traj_ = msg;

    if (!first_traj_received_) {
      first_traj_received_ = true;
      RCLCPP_INFO(get_logger(), "First trajectory: %zu pts, t0=%.2f, pos0=(%.1f,%.1f)",
                  msg->trajectory_points.size(), traj_t0_,
                  msg->trajectory_points[0].x, msg->trajectory_points[0].y);
    }
  }

  void timer_callback()
  {
    // Freeze until the first trajectory arrives (planner has 1.5 s startup delay).
    if (!first_traj_received_) return;

    // Compute target time from wall-clock elapsed since the latest
    // trajectory arrived, added to that trajectory's start time.
    // Both planner and simulator advance at 1.0 simulation-second per
    // wall-second, so this stays aligned with the planner's sim_time_.
    double elapsed = (this->now() - traj_arrival_time_).seconds();
    double target_time = traj_t0_ + elapsed;

    if (latest_traj_ && latest_traj_->trajectory_points.size() > 2) {
      const auto& pts = latest_traj_->trajectory_points;

      if (target_time < pts[0].time) target_time = pts[0].time;
      if (target_time > pts.back().time) target_time = pts.back().time;

      size_t idx = 0;
      for (size_t i = 0; i < pts.size() - 1; ++i) {
        if (pts[i].time <= target_time && pts[i + 1].time >= target_time) {
          idx = i;
          break;
        }
      }

      double t0 = pts[idx].time;
      double t1 = pts[idx + 1].time;
      double ratio = (t1 > t0) ? (target_time - t0) / (t1 - t0) : 0.0;

      double interp_x = pts[idx].x + ratio * (pts[idx + 1].x - pts[idx].x);
      double interp_y = pts[idx].y + ratio * (pts[idx + 1].y - pts[idx].y);
      double interp_h = pts[idx].heading + ratio * (pts[idx + 1].heading - pts[idx].heading);
      double interp_v = std::sqrt(pts[idx].vx * pts[idx].vx + pts[idx].vy * pts[idx].vy);
      double interp_v2 = std::sqrt(pts[idx + 1].vx * pts[idx + 1].vx + pts[idx + 1].vy * pts[idx + 1].vy);
      double interp_speed = interp_v + ratio * (interp_v2 - interp_v);

      pos_x_ = interp_x;
      pos_y_ = interp_y;
      heading_ = interp_h;
      current_speed_ = interp_speed;
    }

    // Broadcast TF: map -> base_footprint
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = "map";
      t.child_frame_id = "base_footprint";

      t.transform.translation.x = pos_x_;
      t.transform.translation.y = pos_y_;
      t.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, heading_);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      broadcaster_->sendTransform(t);
    }

    // Debug: log position every 100 frames (1s)
    if (++frame_count_ % 100 == 0) {
      RCLCPP_INFO(get_logger(), "TF: target_t=%.1f pos=(%.1f, %.1f) hdg=%.2f speed=%.1f has_traj=%d",
                  target_time, pos_x_, pos_y_, heading_, current_speed_,
                  (latest_traj_ && !latest_traj_->trajectory_points.empty()) ? 1 : 0);
    }
  }

  // ---------- member variables ----------
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<pnc_msg::msg::TrajPoints>::SharedPtr traj_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  pnc_msg::msg::TrajPoints::SharedPtr latest_traj_;
  rclcpp::Time node_start_time_;
  rclcpp::Time traj_arrival_time_;

  double pos_x_, pos_y_, heading_;
  double init_x_, init_y_;
  double current_speed_;
  double traj_t0_;
  int frame_count_;
  bool first_traj_received_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LatticeSimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

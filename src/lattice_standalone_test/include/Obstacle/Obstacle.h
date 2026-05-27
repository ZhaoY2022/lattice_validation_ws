#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "Obstacle_avoid.h"
#include "math_utils.h"
#include "ObjectDecision.h"
#include "boundarys.h"

using namespace Eigen;

// 障碍物的类，无非就是id、长宽高(可不用，换成4个顶点位置)、中心位置以及朝向
class Obstacle
{
public:
  Obstacle() = default;
  ~Obstacle() = default;

  void SetSLBoundary(SL_Boundary &sl_boundary);
  void SetSTBoundary(ST_Boundary &st_boundary);
  const SL_Boundary &PerceptionSLBoundary() const;
  const ST_Boundary &path_st_boundary() const;
  bool IsStatic() const;
  bool IsVirtual() const;

  bool HasTrajectory() const;

  void SetLongitudinalDecision(ObjectNudge::Type type);
  void SetLateralDecision();

  const ObjectDecisionType &LongitudinalDecision() const;
  const ObjectDecisionType &LateralDecision() const;
  bool HasLateralDecision() const;
  bool HasLongitudinalDecision() const;

  Box2d PerceptionBoundingBox() const;
  Box2d GetBoundingBox(const TrajectoryPoint &point) const;
  const common::math::Polygon2d &PerceptionPolygon() const;

  TrajectoryPoint GetPointAtTime(const double relative_time) const;

  const prediction::Ob_Trajectory &Trajectory() const
  {
    return trajectory_prediction;
  }

  // Setters for perception data (used by standalone test node)
  void SetPerceptionBoundingBox(const Box2d &box) { perception_bounding_box_ = box; }
  void SetPerceptionPolygon(const common::math::Polygon2d &poly) { perception_polygon_ = poly; }

private:
  SL_Boundary sl_boundary_;
  ST_Boundary path_st_boundary_;
  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;
  common::math::Polygon2d perception_polygon_;
  Box2d perception_bounding_box_;

  std::vector<std::pair<double, double>> pinnacles;

public:
  // 障碍物的id
  std::string obstacle_id;
  // 障碍物的类型
  /*
    uint8 UNKNOWN=0
    uint8 CAR=1
    uint8 TRUCK=2
    uint8 BUS=3
    uint8 BICYCLE=4
    uint8 MOTORBIKE=5
    uint8 PEDESTRIAN=6
    uint8 ANIMAL=7
  */
  int obstacle_type;
  // 障碍物的形状
  /*
    uint8 BOUNDING_BOX=0
    uint8 CYLINDER=1
    uint8 POLYGON=2
  */
  int obstacle_shape;
  // 障碍物4个顶点位置, 两种表达方式
  geometry_msgs::msg::PoseArray pinnacle;
  std::vector<Vec2d> polygon_points;

  // 中心点位置
  geometry_msgs::msg::Pose centerpoint;
  // 障碍物的速度
  double obstacle_velocity;
  // 障碍物的加速度
  double obstacle_acc;
  // 障碍物朝向
  double obstacle_threa;

  // 扣一圈的半径大小，适用于frenet下的圆形和方形
  double obstacle_radius;

  // 障碍物的长宽高
  double obstacle_length;
  double obstacle_width;
  double obstacle_height;

  // 障碍物标签
  ObjectDecisionType decision_;

  // 时间戳
  builtin_interfaces::msg::Time timestamp_;

  // 动态障碍物的预测轨迹
  prediction::Ob_Trajectory trajectory_prediction;
};

extern std::vector<Obstacle> AllObstacle;

#endif // OBSTACLE_H
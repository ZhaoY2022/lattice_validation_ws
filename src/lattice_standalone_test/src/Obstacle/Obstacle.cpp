#include "Obstacle.h"
#include "path_struct.h"
#include <string>

std::vector<Obstacle> AllObstacle;

// 求障碍物顶点
PPoint ob_left_front;
PPoint ob_left_buttom;
PPoint ob_right_front;
PPoint ob_right_buttom;

void Obstacle::SetSLBoundary(SL_Boundary &sl_boundary) // Lattice专用
{
  sl_boundary_ = std::move(sl_boundary);
}
void Obstacle::SetSTBoundary(ST_Boundary &st_boundary) // Lattice专用
{
  path_st_boundary_ = std::move(st_boundary);
}
const SL_Boundary &Obstacle::PerceptionSLBoundary() const
{
  return sl_boundary_;
}
const ST_Boundary &Obstacle::path_st_boundary() const
{
  return path_st_boundary_;
}

bool Obstacle::IsStatic() const
{
  return obstacle_velocity < 0.2;
}
bool Obstacle::IsVirtual() const
{
  return false; // 假设都不是虚拟的
}

bool Obstacle::HasTrajectory() const
{
  return !(trajectory_prediction.trajectory_point_size() == 0); // 没有预测轨迹就是静态障碍物
}

const ObjectDecisionType &Obstacle::LongitudinalDecision() const
{
  return longitudinal_decision_;
}

void Obstacle::SetLongitudinalDecision(ObjectNudge::Type type)
{
  longitudinal_decision_.nudge_.type = type;
}

void Obstacle::SetLateralDecision()
{
  if (sl_boundary_.start_l_ > 0) // 障碍物在左边，EM专用
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::RIGHT_NUDGE;
  }
  if (sl_boundary_.end_l_ < 0) // 障碍物在右边，EM专用
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::LEFT_NUDGE;
  }
  if (sl_boundary_.end_l_ > -Config_.FLAGS_numerical_epsilon && sl_boundary_.start_l_ < Config_.FLAGS_numerical_epsilon)
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::NO_NUDGE;
  }
}

const ObjectDecisionType &Obstacle::LateralDecision() const
{
  return lateral_decision_;
}
bool Obstacle::HasLateralDecision() const
{
  return lateral_decision_.object_tag_case() != ObjectDecisionType::Type::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLongitudinalDecision() const
{
  return longitudinal_decision_.object_tag_case() != ObjectDecisionType::Type::OBJECT_TAG_NOT_SET;
}

Box2d Obstacle::PerceptionBoundingBox() const
{
  // 必须在订阅之后调用
  return Box2d({centerpoint.position.x, centerpoint.position.y}, obstacle_threa, obstacle_length, obstacle_width);
}

Box2d Obstacle::GetBoundingBox(const TrajectoryPoint &point) const
{
  return Box2d({point.x, point.y}, point.theta, obstacle_length,
               obstacle_width);
}

// 获得障碍物在当前时刻的TrajectoryPoint,为了求GetBoundingBox
TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const
{
  const auto &points = trajectory_prediction.trajectory_point();
  // std::cout<<"points.size():"<<points.size()<<"\n";
  if (points.size() < 2) // 认为是静态障碍物
  {
    TrajectoryPoint point;
    point.set_x(centerpoint.position.x);
    point.set_y(centerpoint.position.y);
    point.set_z(0);
    point.set_theta(obstacle_threa);
    point.set_s(0.0);
    point.set_kappa(0.0);
    point.set_dkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  }
  else // 认为是一个运动的障碍物
  {
    for (size_t i = 0; i < points.size(); i++)
    {
      if (points[i].has_path_point() == false)
      {
        std::cout << "has_path_point_:" << i << ","
                  << "x:" << points[i].x << ","
                  << "y:" << points[i].y << "\n";
      }
    }
    if (relative_time >= points.back().relative_time)
    {
      return points.back();
    }

    auto comp = [](const TrajectoryPoint p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);
    if (it_lower == points.begin())
    {
      return points.front();
    }
    else if (it_lower == points.end())
    {
      return points.back();
    }
    return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
  }
}

const common::math::Polygon2d &Obstacle::PerceptionPolygon() const
{
  return perception_polygon_;
}
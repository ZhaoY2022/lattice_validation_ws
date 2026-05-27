#include "Obstacle_avoid.h"
#include "path_matcher.h"

Obstacle_avoid::Obstacle_avoid()
{
}

Obstacle_avoid::~Obstacle_avoid()
{
}

// 求有效范围内障碍物与车的角度

// 已知载具（矩形）的中心点坐标、长、宽和倾斜角度，求载具（矩形）四个边界点
//************************************
//  Method:    GenerateCarBoundaryPoint
//  FullName:  GenerateCarBoundaryPoint
//  Access:    public
//  Returns:   void
//  Qualifier:根据载具中心坐标生成载具四个边界点的坐标
//  Parameter: double car_length		载具长
//  Parameter: double car_width		载具宽
//  Parameter: PPoint & center 		中心点
//  Parameter: double angle			倾斜角度 非弧度角 [0,360) x轴正方向为0度
//  Parameter: PPoint & front_left	左前点
//  Parameter: PPoint & back_left	左后点
//  Parameter: PPoint & back_right	右后点
//  Parameter: PPoint & front_right	右前点
//************************************
void Obstacle_avoid::CalculateCarBoundaryPoint(double car_length, double car_width, PPoint &center, double angle,
                                               PPoint &front_left, PPoint &back_left, PPoint &back_right,
                                               PPoint &front_right)
{
  // 角度为负值，错误输入，返回
  if (angle < 0)
    return;
  double X1, Y1, X2, Y2, X3, Y3, X4, Y4;
  // 以(x,y)为中心点，不旋转的情况下四个顶点的坐标
  back_right.x = (center.x - car_length / 2);
  back_right.y = (center.y - car_width / 2);
  back_left.x = (center.x - car_length / 2);
  back_left.y = (center.y + car_width / 2);
  front_left.x = (center.x + car_length / 2);
  front_left.y = (center.y + car_width / 2);
  front_right.x = (center.x + car_length / 2);
  front_right.y = (center.y - car_width / 2);
  if (angle <= 0.00001)
    return;
  else
  {
    // 按逆时针旋转角度center.x后的四个点坐标
    X1 = (back_right.x - center.x) * cos(angle) - (back_right.y - center.y) * sin(angle) + center.x;
    Y1 = (back_right.y - center.y) * cos(angle) + (back_right.x - center.x) * sin(angle) + center.y;
    X2 = (back_left.x - center.x) * cos(angle) - (back_left.y - center.y) * sin(angle) + center.x;
    Y2 = (back_left.y - center.y) * cos(angle) + (back_left.x - center.x) * sin(angle) + center.y;
    X3 = (front_left.x - center.x) * cos(angle) - (front_left.y - center.y) * sin(angle) + center.x;
    Y3 = (front_left.y - center.y) * cos(angle) + (front_left.x - center.x) * sin(angle) + center.y;
    X4 = (front_right.x - center.x) * cos(angle) - (front_right.y - center.y) * sin(angle) + center.x;
    Y4 = (front_right.y - center.y) * cos(angle) + (front_right.x - center.x) * sin(angle) + center.y;
    back_right.x = X1;
    back_right.y = Y1;
    back_left.x = X2;
    back_left.y = Y2;
    front_left.x = X3;
    front_left.y = Y3;
    front_right.x = X4;
    front_right.y = Y4;
  }
}

// 产生pre_timepre_time秒的预测
void Obstacle_avoid::Generater_Trajectory(prediction::Ob_Trajectory &result, geometry_msgs::msg::Pose ob_pos, double pre_time,
                                          double obstacle_threa, double obstacle_velocity, double obstacle_acc)
{

  std::vector<TrajectoryPoint> Trajectories;
  double s = 0.0;
  double prev_x = 0.0;
  double prev_y = 0.0;
  double relative_time = 0.0;
  bool empty_path = true;
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  std::vector<std::pair<double, double>> xy_points;

  prediction_points(ob_pos, xy_points, pre_time,
                    obstacle_threa, obstacle_velocity, obstacle_acc);

  if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas))
  {
    std::cout << "obstacle prediction trajectory generate failed!"
              << "\n";
  }
  // std::cout << "xy_points.size():" << xy_points.size() << "\n";

  for (int i = 0; i < xy_points.size(); i++) // 遍历每个预测轨迹点
  {
    TrajectoryPoint tra_;
    tra_.set_path_point();
    tra_.set_x(xy_points[i].first);  // x
    tra_.set_y(xy_points[i].second); // y
    tra_.set_theta(headings[i]);
    tra_.set_v(obstacle_velocity); //
    tra_.set_a(obstacle_acc);      // 匀速的话，那么a=0
    tra_.set_kappa(kappas[i]);
    tra_.set_dkappa(dkappas[i]);
    tra_.set_s(accumulated_s[i]);
    tra_.set_relative_time(relative_time);

    Trajectories.emplace_back(tra_);
    relative_time += Config_.FLAGS_trajectory_time_resolution;
  }

  result.Set_Trajectory(Trajectories);
}

void Obstacle_avoid::prediction_points(geometry_msgs::msg::Pose ob_pos, std::vector<std::pair<double, double>> &output, double pre_time,
                                       double obstacle_threa, double obstacle_velocity, double obstacle_acc)
{
  double relative_time = 0.0;
  while (relative_time <= pre_time)
  {
    double lon = obstacle_velocity * relative_time + 0.5 * obstacle_acc * relative_time * relative_time;
    std::pair<double, double> point;
    point.first = ob_pos.position.x + lon * cos(obstacle_threa);
    point.second = ob_pos.position.y + lon * sin(obstacle_threa);

    output.push_back(point);

    relative_time += Config_.FLAGS_trajectory_time_resolution;
  }
}
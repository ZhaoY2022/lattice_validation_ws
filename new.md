目标：构建真实高速公路测试场景，增加rviz2交互按钮，可以手动激活左换道和右换道，下面是具体的内容，如果觉得有不合理的地方你可以修改。

请帮我深度重构当前 ROS2 Lattice 轨迹规划工作空间中的 `src/lattice_standalone_test/src/scenario_manager.cpp` 和 `lattice_test_node.cpp`。本次重构包含两个核心目标：
1. **生成高阶测试场景 (Scenario 5)**：构建一条长达 4.2 公里的曲率连续（包含缓和曲线 Clothoid）的高速公路连续 S 弯赛道。
2. **实现交互式换道与闭环状态机**：新增换道话题订阅随时触发换道，并在换道结束后严格按照量产逻辑（平移参考线、目标偏距归零、无缝切回 LCC）完成状态闭环。

---

### 第一部分：修改 `src/lattice_standalone_test/src/scenario_manager.cpp`

**1. 新增曲率插值基础工具**
在文件顶部（或匿名命名空间中）添加结构体和插值函数：
```cpp
struct CurvatureKeypoint {
    double s;
    double kappa;
};

void GetKappaAndDkappa(const std::vector<CurvatureKeypoint>& profile, double s, double& kappa, double& dkappa) {
    if (s <= profile.front().s) { kappa = profile.front().kappa; dkappa = 0.0; return; }
    if (s >= profile.back().s) { kappa = profile.back().kappa; dkappa = 0.0; return; }
    for (size_t i = 0; i < profile.size() - 1; ++i) {
        if (s >= profile[i].s && s <= profile[i + 1].s) {
            double ds = profile[i + 1].s - profile[i].s;
            dkappa = (profile[i + 1].kappa - profile[i].kappa) / ds;
            kappa = profile[i].kappa + dkappa * (s - profile[i].s);
            return;
        }
    }
}
2. 实现 build_scenario_5 连续 S 弯场景
新增以下函数生成 4.2km 曲率赛道（无障碍物，专测极限横向动力学）
void build_scenario_5(std::vector<PathPoint>& reference_line /*请补全与原有build_scenario一致的其他参数*/) {
    std::vector<CurvatureKeypoint> kappa_profile = {
        {0.0, 0.0}, {200.0, 0.0},
        {300.0, 1.0/500.0}, {800.0, 1.0/500.0},     // 500m 左弯
        {900.0, 0.0}, {1200.0, 0.0},
        {1350.0, -1.0/700.0}, {2050.0, -1.0/700.0}, // 700m 右弯
        {2200.0, 0.0}, {2500.0, 0.0},
        {2700.0, 1.0/1000.0}, {3700.0, 1.0/1000.0}, // 1000m 左弯
        {3900.0, 0.0}, {4200.0, 0.0}
    };

    double ds = 0.5;
    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    reference_line.clear();

    for (double s = 0.0; s <= 4200.0; s += ds) {
        double kappa = 0.0, dkappa = 0.0;
        GetKappaAndDkappa(kappa_profile, s, kappa, dkappa);
        
        current_theta += kappa * ds;
        current_x += std::cos(current_theta) * ds;
        current_y += std::sin(current_theta) * ds;

        PathPoint pt; // 请适配实际方法名
        pt.set_x(current_x); pt.set_y(current_y);
        pt.set_theta(current_theta);
        pt.set_kappa(kappa); pt.set_dkappa(dkappa);
        pt.set_s(s);
        reference_line.push_back(pt);
    }
}
3. 场景注册：在 load_scenario 函数中，确保 scenario == 5 时调用 build_scenario_5。
第二部分：修改 src/lattice_standalone_test/src/lattice_test_node.cpp
1. 引入头文件与定义类成员

顶部添加 #include <std_msgs/msg/int8.hpp> 和 #include <cmath>。

在 LatticeTestNode 类中添加以下私有成员：
rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr lane_change_sub_;
bool alc_completed_flag_ = false;
int alc_complete_debounce_counter_ = 0;

2. 实现换道指令回调函数
在类中新增以下方法接收外部指令：
void lane_change_cmd_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    if (driving_mode_ == DrivingMode::ALC) {
        RCLCPP_WARN(this->get_logger(), "车辆正在换道中，忽略新的换道指令！");
        return;
    }
    if (msg->data == 1 || msg->data == -1) {
        double target_offset = (msg->data == 1) ? 3.75 : -3.75;
        planning_target_.set_target_lat_offset(target_offset);
        driving_mode_ = DrivingMode::ALC;
        alc_complete_debounce_counter_ = 0;
        alc_completed_flag_ = false;
        RCLCPP_INFO(this->get_logger(), ">>> 收到交互指令：开始换道 (目标偏距: %.2f) <<<", target_offset);
    }
}
3. 构造函数与场景初始化

在节点构造函数中初始化订阅器：
lane_change_sub_ = this->create_subscription<std_msgs::msg::Int8>("/lattice_test/lane_change_cmd", 10, std::bind(&LatticeTestNode::lane_change_cmd_callback, this, std::placeholders::_1));

在 init_scenario() 中支持场景 5。当 scenario_ == 5 时，默认设置 driving_mode_ = DrivingMode::LCC，target_speed_ = 22.0，planning_target_.set_target_lat_offset(0.0)，确保初始车辆横向偏差 vehicle_state_.d0 = 0.0。

4. timer_callback() 中的换道结束状态机闭环
在 timer_callback() 核心逻辑区域（获取车辆状态后，且在发布之前），嵌入完整的换道结束判定与参考线平移逻辑：

if (driving_mode_ == DrivingMode::ALC) {
    double target_d = planning_target_.target_lat_offset();
    double lat_error = std::abs(vehicle_state_.d0 - target_d); 
    
    // 计算航向误差（请适配实际的参考线投影点获取逻辑）
    double ref_theta = match_point.theta(); 
    double heading_error = std::abs(common::math::NormalizeAngle(vehicle_state_.theta - ref_theta));

    if (lat_error < 0.20 && heading_error < 0.03) {
        alc_complete_debounce_counter_++;
    } else {
        alc_complete_debounce_counter_ = 0;
    }

    if (alc_complete_debounce_counter_ >= 5) { // 连续5帧满足条件
        alc_completed_flag_ = true;
        
        // 【核心】平移参考线（法向平移）
        for (auto& pt : reference_line_) {
            double nx = -std::sin(pt.theta());
            double ny = std::cos(pt.theta());
            pt.set_x(pt.x() + target_d * nx);
            pt.set_y(pt.y() + target_d * ny);
        }
        
        // 偏距归零，切回LCC
        planning_target_.set_target_lat_offset(0.0);
        driving_mode_ = DrivingMode::LCC;
        alc_complete_debounce_counter_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "==== [ALC -> LCC] 换道完成！参考线已平移，切回 LCC 模式 ====");
    }
} else {
    alc_complete_debounce_counter_ = 0;
    alc_completed_flag_ = false;
}

(注意：这里将平移逻辑升级为了沿法向量 nx, ny 平移，因为 Scenario 5 包含曲线，单纯改 y 坐标不再适用。)

执行要求：请仔细匹配项目中原有的变量名（如 driving_mode_, match_point, vehicle_state_.d0 等）。修改完成后自动执行 colcon build 确保无编译错误。


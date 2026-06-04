# Lattice Trajectory Planning — Standalone ROS2 Validation Workspace

将 Lattice 轨迹规划算法从 CARLA 集成项目（`auto_driving_carla_ros2_humble`）中提取，构建为独立的 ROS2 Humble 验证工作空间。通过闭环仿真验证轨迹规划 — 每帧将车辆"瞬移"到最优轨迹点，在 RViz2 中可视化。

## 目录结构

```
lattice_validation_ws/
├── src/pnc_msg/                     # 控制接口消息包
│   ├── msg/
│   │   ├── TrajectoryPoint.msg      # 单轨迹点：time/length/x/y/vx/vy/acc_x/acc_y/heading/curvature/pinch/jerk
│   │   └── TrajPoints.msg           # 轨迹消息：header + 业务flags + TrajectoryPoint[40]
│   ├── CMakeLists.txt
│   └── package.xml
├── src/lattice_standalone_test/
│   ├── CMakeLists.txt              # 构建配置
│   ├── package.xml                  # ROS2 包清单
│   ├── config/configs.yaml          # 规划器参数（从原始项目提取）
│   ├── include/                     # 头文件（扁平结构，保留原始 #include 路径）
│   │   ├── Lattice_planner/         # 14 个头文件 — 核心规划器、评估器、组合器、检查器、采样器
│   │   ├── Frenet/                  # cartesian_frenet_conversion.h — 坐标系转换
│   │   ├── Polynomial/              # 7 个头文件 — Curve1d、Quintic/Quartic 多项式
│   │   ├── Spline/                  # CubicSpline1D.h、CubicSpline2D.h — 参考线插值
│   │   ├── path/                    # 5 个头文件 — TrajectoryPoint、PathPoint、FrenetPath
│   │   ├── ReferenceLine/           # reference_point.h — 参考线点
│   │   ├── common/                  # 10 个头文件 — Vec2d、Box2d、Polygon2d、Configs、数学工具
│   │   └── Obstacle/                # 5 个头文件 — 障碍物模型、边界、预测、避障
│   ├── src/                         # 源文件（与 include/ 镜像）
│   │   ├── Lattice_planner/         # 14 个 .cpp 文件
│   │   ├── Frenet/ Polynomial/ Spline/ path/ ReferenceLine/ common/ Obstacle/
│   │   ├── lattice_test_node.cpp    # 规划节点
│   │   ├── lattice_simulator_node.cpp  # 仿真节点（独立）
│   │   └── scenario_manager.cpp     # 场景管理器（共用库）
│   ├── launch/run_scenario.launch.py # 启动文件（5 进程）
│   ├── rviz/lattice_test.rviz       # RViz2 配置（深色主题）
│   └── urdf/lattice_car.urdf        # 车辆 3D 模型
└── README.md
```

## 构建

```bash
cd /home/zy/claude_workspace/test02/lattice_validation_ws
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

**依赖**: ROS2 Humble (rclcpp, nav_msgs, visualization_msgs, geometry_msgs, tf2_ros)、Eigen3 (header-only)、yaml-cpp (conda 版本以避免 ABI 兼容性问题)

**DDS**: 默认使用 CycloneDDS（`rmw_cyclonedds_cpp`），无共享内存残留，大消息吞吐更优。备用方案：Fast-DDS 纯 UDPv4（`config/fastdds_no_shm.xml`）

## 运行

```bash
source install/setup.bash

# Scenario 1: 横向偏移恢复
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=1

# Scenario 2: 静态障碍物避让
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=2

# Scenario 3: 变道
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=3
```

# Scenario 5: 4.2km S 弯高速赛道
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=5

# 交互式换道（Scenario 5 运行时）
ros2 topic pub --once /lattice_test/lane_change_cmd std_msgs/msg/Int8 "data: 1"   # 左换道 +3.75m
ros2 topic pub --once /lattice_test/lane_change_cmd std_msgs/msg/Int8 "data: -1"  # 右换道 -3.75m
```

## 四种验证场景

### Scenario 1 — 横向偏移恢复（LCC 模式）
- 参考线：y=0 直线，x∈[0, 500]
- 车辆初始位置：(0, 0.5)，横向偏离参考线 0.5m
- 初始速度：22.0 m/s
- 无障碍物
- 驾驶模式：**LCC**（车道保持），横向采样 5×7=35 条，l∈[-0.3, 0.3]
- **预期行为**：规划器生成收敛到参考线的轨迹，车辆从 y=0.5 逐渐回到 y=0

### Scenario 2 — 静态障碍物避让（NUDGE 模式）
- 参考线：y=0 直线，x∈[0, 500]
- 车辆初始位置：(0, 0)，初始速度 22.0 m/s
- 静态障碍物：4m×2m 矩形，位于 (95, 0.2)（偏离中线 0.2m）
- 驾驶模式：**NUDGE**（避让），横向采样 5×11=55 条，l∈[-2.5, 2.5]，smin 动态计算（见下方公式）
- **预期行为**：规划器检测障碍物并生成绕行轨迹，通过后回到参考线

### Scenario 3 — 变道（ALC 模式）
- 参考线：y=0 直线，x∈[0, 500]（仅表示自车当前车道中心线）
- 目标车道：d=-3.75（参考线右侧 3.75m，通过 `PlanningTarget::set_target_lat_offset(-3.75)` 设定）
- 车辆初始位置：(0, 0)，初始速度 22.0 m/s
- 驾驶模式：**ALC**（变道），横向采样 4×4=16 条，end_d 按目标比例生成 `{tgt, tgt*0.85, tgt*0.6, 0}`（比例自适应，任意偏距下无端点超调），end_s 直接按时间定义 `current_speed × {7.5, 7.8, 8.2, 8.6}s`，**任意速度下换道时间恒定**（规划器优选 8.0–8.2s）
- **预期行为**：规划器生成从 d≈0 到 d→-3.75 的轨迹簇，车辆从当前车道平滑过渡到目标车道。**注意**：d 是 Frenet 横向偏差，不是 Cartesian y。车辆从 y≈0 变道到 y≈-3.75
- **换道完成判定**（`lattice_test_node.cpp` timer_callback）：当 `|d0 - target_d| < 0.20m` 且 `|heading_error| < 0.03 rad` 连续满足 5 帧（0.5s 防抖），触发量产级状态跳转：(1) 将整条 `reference_line_` 的 y 坐标平移 `target_d`，参考线物理移动到目标车道中心；(2) `target_lat_offset` 归零（通过 `alc_completed_flag_` 守卫，下一帧不再设为 -3.75）；(3) `driving_mode` 切回 `LCC`，车道保持权重自然维持车辆在目标车道中心
- **设计理念**：真实自动驾驶中不存在预先弯曲的"期望轨迹"参考线。参考线通常只是当前车道中心线，规划器必须通过目标车道偏移量（`target_lat_offset`）自主生成变道路径。换道完成后，参考线沿道路法向平移到新车道 + 偏距归零 + 切回 LCC，符合量产标准逻辑

### Scenario 5 — 4.2km S 弯高速赛道 + 交互式换道（LCC / ALC 模式）
- 参考线：从分段线性曲率剖面数值积分生成，4200m 连续 S 弯（3 个弯道：500m/700m/1000m 半径 + 缓和曲线过渡），ds=0.5m，约 8401 个参考点
- 车辆初始位置：(0, 0)，初始速度 22.0 m/s，初始航向 0
- 无障碍物，默认驾驶模式：**LCC**（车道保持）
- **交互式换道**：订阅 `/lattice_test/lane_change_cmd`（`std_msgs/Int8`），1=左换道 +3.75m，-1=右换道 -3.75m。换道完成后参考线沿道路法向平移，偏距归零，自动切回 LCC。支持反复换道
- **预期行为**：车辆在 S 弯上自动保持车道中心。收到换道指令后平滑变道，完成后继续在目标车道 LCC 行驶

## 可视化 Topics

所有话题均使用 `map` 坐标系。

| Topic | 类型 | 颜色/样式 | 说明 |
|-------|------|-----------|------|
| `/lattice_trajectory` | `pnc_msg/TrajPoints` | 40 点重采样轨迹 | **控制接口**：最优轨迹 40 点离散化输出，含 curvature/pinch/jerk/vx/vy/acc_x/acc_y，用于下游控制模块和调试 |
| `/lattice_test/reference_path` | `nav_msgs/Path` | 蓝色 | 参考线 |
| `/lattice_test/optimal_trajectory` | `nav_msgs/Path` | 绿色 | 最优轨迹 |
| `/lattice_test/candidate_trajectories` | `nav_msgs/Path` | 空消息 | 候选轨迹（通过 MarkerArray 的 `candidates` 命名空间以独立 LINE_STRIP 显示） |
| `/lattice_test/road_boundaries` | `MarkerArray` | 白色/黄色 LINE_STRIP | 道路边界（左右白色 + 中线黄色虚线），lifetime=0 永久 |
| `/lattice_test/lane_change_cmd` | `std_msgs/Int8` | 订阅 | **交互式换道指令**：1=左换道 +3.75m, -1=右换道 -3.75m。换道进行中拒绝新指令 |
| `/lattice_test/viz_markers` | `MarkerArray` | 颜色编码 | 8 个命名空间：`reference`（蓝色 LINE_STRIP lifetime=2.0s）、`optimal`（绿色 LINE_STRIP lifetime=1.0s）、`candidates`（青色=有效+无碰撞 / 橙色=仅运动学有效 / 灰色=无效, lifetime=1.0s）、`obstacles`（红色半透明 CUBE α=0.5）、`ego`（黄色半透明 CUBE α=0.9, 3.0m×1.6m×1.5m, lifetime=1.0s）、`history_candidates`（历史候选快照 lifetime=∞）、`history_optimal`（历史最优快照 lifetime=∞）。Frame 0 发送 DELETEALL 清除上次运行残留 |

### 轨迹簇线宽和颜色修改

**修改位置**: `src/lattice_test_node.cpp` — `publish_all()` 中候选轨迹 `candidates` 命名空间

候选轨迹渲染在最优轨迹**之前**，使绿色最优线绘制在最上层。候选点设置 `z=0.02` 的微小高度偏移，在 3D 视图中浮于地平面之上。

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `m.scale.x` | `0.08` | LINE_STRIP 线宽（ROS marker 的 scale.x） |
| 有效+无碰颜色 | 青色 (0,1,1) α=0.8 | `constraint_valid && collision_free` |
| 仅运动学有效颜色 | 橙色 (1,0.5,0) α=0.65 | `constraint_valid && !collision_free` |
| 无效颜色 | 灰色 (0.5,0.5,0.5) α=0.35 | 运动学约束不通过 |
| 候选数量上限 | `50` | `MAX_CANDIDATES = 50` |
| 轨迹点降采样 | `kSubsample = 3` | 每隔 3 个点取 1 个减少几何体 |

其他可视化参数:
| 元素 | 参数 |
|------|------|
| 参考线 (reference) | `m.scale.x = 0.08`, lifetime 2.0s |
| 最优轨迹 (optimal) | `m.scale.x = 0.12`, lifetime 1.0s |
| 候选轨迹 (candidates) | `m.scale.x = 0.08`, lifetime 1.0s, z=0.02 |
| 自车 (ego) | CUBE 3.0×1.6×1.5m, lifetime 1.0s |
| 障碍物 (obstacles) | CUBE, `frame_locked=true`, `lifetime=∞`（仅 frame 0 发布一次，持久化）。**无障碍物时不添加任何标记**（避免零尺寸退化 CUBE 导致 RViz2 渲染警告） |
| 历史候选 (history_candidates) | `m.scale.x = 0.06`, lifetime=∞, z=0.04, 颜色同实时但更透明 |
| 历史最优 (history_optimal) | `m.scale.x = 0.10`, lifetime=∞, z=0.04, 深绿色 α=0.7 |
| QoS 深度 | 5（除 transient_local 参考线外） |

> **注意**: LCC 模式下候选轨迹横向范围仅 l∈[-0.3, 0.3]，在 RViz 远距离视角下几乎不可见。NUDGE 模式（l∈[-2.5, 2.5]）可看到明显的轨迹簇扇出。

## 历史轨迹持久化

新增 `persist_history` 参数，开启后在每次运行结束时保留所有历史时刻的轨迹簇和最优轨迹。

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `persist_history` | bool | false | 开启历史轨迹持久化 |
| `history_frame_interval` | int | 5 | 每隔多少帧记录一次快照（5 = 0.5s 间隔） |

### 使用方式

```bash
# 开启持久化（每 0.5s 一张快照）
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=1 persist_history:=true

# 每 1 秒一张快照（适合长时间运行）
ros2 launch lattice_standalone_test run_scenario.launch.py scenario:=2 persist_history:=true history_frame_interval:=10
```

### 实现机制

当 `persist_history=true` 时，每 `history_frame_interval` 帧在 MarkerArray 中追加一组「历史快照」：
- 命名空间 `history_candidates`：50 条候选轨迹 LINE_STRIP，lifetime=(0,0) 永不过期，颜色编码与实时一致但更透明
- 命名空间 `history_optimal`：最优轨迹 LINE_STRIP，深绿色 α=0.7，lifetime=(0,0)
- z=0.04，浮在实时轨迹上方（实时 z=0.00~0.02）
- ID 使用 1,000,000 偏移量，不与实时标记冲突
- RViz 配置中 `history_candidates` 和 `history_optimal` 命名空间默认开启，可在面板中独立开关

### 效果

- 运行中：实时轨迹正常显示 + 历史快照逐渐积累
- 停止后 Ctrl+C：实时轨迹在 1 秒内淡出，所有历史轨迹永久保留
- 10 秒运行约产生 20 组快照 × 50 = 1000 条历史 LINE_STRIP，RViz2 可正常渲染

## 控制接口：`/lattice_trajectory`

每规划周期发布一条 `pnc_msg::msg::TrajPoints` 消息，包含将最优轨迹按时间均匀重采样为 40 个点后的完整运动学信息。

### TrajPoints 消息结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | ROS 时间戳 + `frame_id="map"` |
| `sf_trajectory_cnt` | `uint32` | 固定 40 |
| `sf_trajectory_time_stamp` | `float32` | 仿真时间 [s] |
| `sf_trajectory_flag` | `uint16` | 预留 |
| `sf_trajectory_driving_lateral_flag` | `uint16` | 驾驶模式：0=LCC, 1=ALC, 2=NUDGE |
| `sf_trajectory_driving_longitudinal_flag` | `uint16` | 纵向驾驶标志（预留） |
| `sf_trajectory_active_safety_lateral_flag` | `uint16` | 横向安全标志（预留） |
| `sf_trajectory_active_safety_longitudinal_flag` | `uint16` | 纵向安全标志（预留） |
| `sf_trajectory_front_vehicle_velx` | `float32` | 前车速度（预留） |
| `sf_trajectory_front_vehicle_accx` | `float32` | 前车加速度（预留） |
| `trajectory_points` | `TrajectoryPoint[40]` | 40 点均匀采样轨迹 |

### TrajectoryPoint 字段

| 字段 | 类型 | 计算方式 | 说明 |
|------|------|----------|------|
| `time` | `float32` | `i × T/39` | 相对规划局部时间 [s]，范围 0→4.0 |
| `length` | `float32` | `s - s_start` | 沿轨迹的相对弧长 [m] |
| `x, y` | `float32` | 直接取值 | GROUND 系位置 |
| `vx, vy` | `float32` | `v × (cosθ, sinθ)` | GROUND 系速度分量 |
| `acc_x, acc_y` | `float32` | `a × (cosθ, sinθ)` | GROUND 系加速度分量 |
| `heading` | `float32` | `θ` | 航向角 [rad] |
| `curvature` | `float32` | `κ` | 曲率 [m⁻¹] |
| `pinch` | `float32` | `dkappa` | 曲率变化率 dκ/ds [m⁻²] |
| `jerk` | `float32` | 中心差分 da/dt | 加速度变化率 [m/s³] |
| `reserved` | `float32` | 0.0 | 预留 |

### 重采样方法

- 计算最优轨迹的时间跨度 `T = trajectory.back().relative_time`（截断后约 4.0s）
- 在 [0, T] 上均匀生成 40 个时间点 `t_i = i × T / 39`
- 通过 `DiscretizedTrajectory::Evaluate(t_i)` 线性插值获取各点状态
- Jerk 通过三点中心差分（首尾点前向/后向差分）计算切向加加速度 `da/dt`

## 架构

```
run_scenario.launch.py
├── lattice_test_node         (10Hz)  规划器 — LatticePlan + 可视化发布
├── lattice_simulator_node   (100Hz) 仿真器 — TF 广播 map→base_footprint, 增量运动
├── robot_state_publisher     (—)    URDF RobotModel 发布
├── joint_state_publisher     (—)    关节状态发布
└── rviz2                     (30fps) 可视化
```

```
lattice_test_node (10Hz wall timer)
    │
    ├── Scenario Init (scenario_manager) ──→ reference_line_, vehicle_state_, obstacles_
    │
    ├── tf2 Buffer lookup (map→base_footprint) ──→ 获取车辆位姿
    │
    ├── LatticePlan()
    │   ├── ComputeInitFrenetState()       # 笛卡尔 → Frenet 坐标
    │   ├── Trajectory1dGenerator           # 纵向 + 横向 1D 轨迹束
    │   ├── TrajectoryEvaluator             # 轨迹对代价评估（优先级队列）
    │   └── TrajectoryCombiner::Combine()   # Frenet → 笛卡尔转换
    │       ├── PathMatcher                 # 参考线匹配
    │       └── CartesianFrenetConverter    # 坐标转换
    │
    ├── publish_lattice_trajectory()       # 40点重采样 → /lattice_trajectory
    └── publish_all()                      # 发布可视化 topics
```

## 日志输出说明

每次运行自动在 `/tmp/` 下生成 CSV 文件：`lattice_log_YYYYMMDD_HHMMSS.csv`

### CSV 文件格式

| 列名 | 含义 |
|------|------|
| `timestamp` | ROS 时间戳（秒） |
| `frame` | 规划周期编号（frame × 0.1s = 经过时间） |
| `sim_time` | 仿真时间（秒） |
| `s0` | 车辆 Frenet **纵向**位置，沿参考线的弧长 (m) |
| `d0` | 车辆 Frenet **横向**偏差，垂直参考线方向 (m)。d>0=左侧 |
| `v` | 当前车速 (m/s) |
| `mode` | 驾驶模式：`LCC`（车道保持）、`NUDGE`（避让）、`ALC`（变道） |
| `result_pts` | 最优轨迹的点数（4 秒截断后，约 41 个点） |
| `collected` | 本周期收集的候选轨迹对数量（上限 200） |

### 终端日志字段解释

以实际输出为例：

```
[frame=45] s0=101.2 d0=0.02 v=22.0 result=41 collected=200 mode=LCC
```

| 字段 | 值 | 含义 |
|------|-----|------|
| `frame` | 45 | 第 45 个规划周期（45 × 0.1s = 4.5s 已运行） |
| `s0` | 101.2 | 沿参考线行驶了 101.2m |
| `d0` | 0.02 | 距参考线中心 2hucm，几乎居中 |
| `v` | 22.0 | 当前速度 22 m/s |
| `result` | 41 | 最优轨迹含 41 个离散点（4s / 0.1s 分辨率） |
| `collected` | 200 | 收集了 200 条候选轨迹（上限） |
| `mode` | LCC | 车道保持模式 |

```
init_s[0]=101.2 init_d=[0.018,-0.002,0.0001] d_frenet=0.018 d0_inherited=0.018 mode=LCC
```

| 字段 | 含义 |
|------|------|
| `init_s[0]` | Frenet 纵向位置 s |
| `init_d` | Frenet 横向状态向量 `[d, d_dot, d_ddot]`：`d=0.018` 偏离 1.8cm，`d_dot=-0.002` 以 2mm/s 向中心漂移，`d_ddot=0.0001` 横向加速度可忽略 |
| `d_frenet` | 本周期 Cartesian→Frenet 转换算出的实际横向偏差 |
| `d0_inherited` | 上一周期车辆状态继承的横向偏差（LCC 模式下 d_frenet == d0_inherited，NUDGE 避障期间可能不同） |

```
lon_bundle=50 lat_bundle=35 total_pairs=1750
```

| 字段 | 含义 |
|------|------|
| `lon_bundle` | 纵向 1D 轨迹束数量（四次多项式速度剖面，由 9 个时间采样点 × 6 个速度采样点生成） |
| `lat_bundle` | 横向 1D 轨迹束数量（五次多项式 d(t)，LCC: 5s×7d=35, NUDGE: 5s×11d=55, ALC: 6s×9d=54） |
| `total_pairs` | 纵向×横向的组合总数 = 1750 对 |

```
checked=1015 traj_pairs, result_size=81 collected=200
```

| 字段 | 含义 |
|------|------|
| `checked` | 从优先级队列中**实际评估**的轨迹对数量（检查运动学约束和碰撞） |
| `result_size` | 最优轨迹的**完整点数**（8s 规划时域约 81 个点，发布前截断至 4s≈41 点） |
| `collected` | 存入 `all_trajectories` 的结果数（即 RViz 中候选轨迹的来源，上限 200） |

> **为什么 `checked` < `total_pairs`（1015 < 1750）：** 优先级队列按代价从低到高排序，只需评估到收集满 200 条结果即停止，其余跳过以保持实时性。

## 关键修改（相对于原始 CARLA 项目）

### 剥离的依赖
- **CARLA**: `derived_object_msgs`、`carla_msgs`、`dynamic_routing_node`
- **QP 求解器**: OSQP、qpOASES（~20 文件，横向 QP 优化）
- **EM Planner**: `path_data`、`path_boundary`、`path_decision`、`speed_data`、`reference_line`
- **替代规划器**: `FrenetOptimalTrajectory`

### 修改的文件
1. **`common/Config.cpp`** — 将硬编码的绝对配置路径替换为 `$LATTICE_CONFIG_PATH` 环境变量
2. **`Obstacle/Obstacle.h/.cpp`** — 移除 `setObstaclesFromCarla()`、`derived_object_msgs` 头文件、`GetObstacle` ROS2 节点类；新增 `SetPerceptionBoundingBox()` 和 `SetPerceptionPolygon()` 公共 setter
3. **`Lattice_planner/lattice_planner.h/.cpp`** — 移除 ROS 可视化方法（`visualization_obstacle_trajectory`、`Show_ob_prediction`）
4. **`Lattice_planner/trajectory1d_generator.h/.cpp`** — 注释 QP 横向优化分支，仅使用基于采样的横向规划
5. **`Obstacle/Obstacle_avoid.h/.cpp`** — 清理未使用的 ROS2 头文件
6. **`path/path_struct.h`** — 移除 `VehicleConfig` 类和 `nav_msgs/msg/odometry.hpp` 头文件

### Bug 修复
1. **`common/math_utils.cpp:118-145`** — `InterpolateUsingLinearApproximation`: 当 TrajectoryPoint 的 `path_point` 未设置时，回退到使用 TrajectoryPoint 自身的直接字段（x, y, theta, kappa, dkappa, s, d, d_d, d_dd, s_d, s_dd）进行插值。修复了 `DiscretizedTrajectory::Evaluate()` 返回零点导致车辆状态归零的关键 bug
2. **`Lattice_planner/trajectory_combiner.cpp`** — 在组合轨迹时为每个 TrajectoryPoint 填充 `PathPoint` 并调用 `set_path_point()`，确保下游插值能够正常工作
3. **`Lattice_planner/lattice_planner.cpp:117-130`** — 窗口化搜索不再直接取离散参考点，改用 `PathMatcher::FindProjectionPoint()` 在相邻参考点之间进行插值投影，得到精确的垂足匹配点。修复了离散匹配导致的 Frenet d 不准确问题（车辆横向偏差被错误计算，导致轨迹偏离）
4. **`lattice_test_node.cpp`** — 多项可视化修复：
   - 参考线分辨率从 1.0m 提高到 0.5m，提高匹配精度
   - 新增 `publish_reference_path()` 方法，参考线使用 `transient_local` QoS 仅发布一次（静态数据）
   - 候选轨迹 Path 消息改为发布空消息（多个 LINE_STRIP 无法通过单个 Path 消息正确显示）
   - MarkerArray 采用 ADD-only 模式 + lifetime 持久化：不使用 DELETEALL，每帧 ADD 覆盖同名 marker，陈旧 ID 以透明 marker 清理
   - 可视化节流至每 3 帧（~3.3Hz），减少渲染负载
   - 轨迹点降采样（每隔 3 个点取 1 个），减少几何体数量
   - 候选轨迹数量上限 20 条，颜色编码：青色=有效+无碰撞，橙色=仅运动学有效，灰色=无效
   - 线宽调整：参考线 0.08，最优轨迹 0.12，候选轨迹 0.03
5. **`lattice_planner.cpp:167-172`** — 状态继承改为条件化：仅在 NUDGE 模式且附近有障碍物时才从前一周期继承横向偏移 d0。其他情况使用 Cartesian→Frenet 转换的实际 d 值，使车辆在避障后能自然回到参考线
6. **`end_condition_sampler.cpp` / `trajectory_evaluator.cpp`** — 所有模式特定的横向采样参数和评价权重从 C++ 硬编码迁移到 `configs.yaml`，修改参数无需重编译。包含：LCC/ALC/NUDGE 的 end_d/end_s 候选集、smin 系数、权重配置、超调惩罚系数、tiebreaker
7. **`trajectory_evaluator.cpp`** — 轨迹评价权重按驾驶模式分别设置，LCC 使用强居中权重（lat_offset=1.0），ALC 使用弱居中权重（lat_offset=0.1），NUDGE 使用中等权重。opposite_side 和 same_side 权重统一为 1.0，消除返回参考线的惩罚不对称性
8. **`config/configs.yaml`** — 速度/加速度/加加速度约束放宽以适应 22 m/s 工况（speed_upper_bound: 5→40, accel_lower: -1.0→-4.0, accel_upper: 0.5→2.0, jerk: ±1.0→±4.0, lat_accel_bound: 4.0→10.0, kappa_bound: 1.0→2.0）
9. **`config/configs.yaml`** — `FLAGS_speed_lon_decision_horizon`: 50→**250**。LCC 模式的横向终点 s 值在 93-118m（smin=88），原来 50m 的评价视界内所有横向轨迹 d≈0，轨迹簇不可见。扩大到 250m 后完全覆盖了横向轨迹发散区，轨迹簇可视
10. **`config/configs.yaml` + `Config.h` + `Config.cpp`** — 删除所有未被 lattice planner 实际使用的参数（EM Planner、QP 求解器、Open Space 等模块的参数），从 268 行精简到 58 行，Config.h 从 274 行精简到 68 行，Config.cpp 从 272 行精简到 81 行
11. **`end_condition_sampler.cpp:59-66`** — NUDGE 模式 smin 动态化：基于障碍物实际距离计算，替代硬编码的 speed×2。`safe_smin = max(20, distance_to_obstacle - 10)`, `kinematic_smin = speed × 1.8`, `smin_nudge = max(safe_smin, kinematic_smin)`。end_s 候选: {smin_nudge, +10, +15, +20, +30}
12. **`trajectory_evaluator.cpp:174-180`** — NUDGE 模式评价权重调整：w_opposite_side_ 4.0→1.0（不惩罚避障方向），w_lat_offset_bound_ 0.65→3.0（宽边界容纳 d=-2.5），w_lat_offset 2.0→1.0。组合效果使避障轨迹成本合理，避免因代价过高而被淘汰
13. **`lattice_test_node.cpp:269-286`** — S2 避障后自动回中：读取障碍物 polygon_points，用 `PathMatcher::GetPathFrenetCoordinate()` 计算 `obs_max_s`（障碍物尾端 Frenet s）。车辆通过后（`s0 > obs_max_s + 2.0m`）立即切换为 LCC。**不人工清零任何状态**——Cartesian→Frenet 自动算出当前横向偏差 d，LCC 的 w_lat_offset=4 自然驱动五次多项式收敛到 d=0
14. **`lattice_test_node.cpp:111,135`** — S1/S2 参考线从 300m 延长至 500m。原因：在 22 m/s 速度下，8s 规划时域内四次多项式速度规划可能使 s 终端值超过 300m（v_upper 可达 38 m/s，s(8s)≈363m），轨迹组合器中 `s > s_ref_max` 检查会截断轨迹，导致 result_size 骤降至 2、车辆冻结
15. **`trajectory_evaluator.cpp:160-165`** — LCC 模式 w_opposite_side_ 从 8.0 降至 1.0。原因：当 d≈0 时 `lat_offset * lat_offset_start < 0` 的符号每周期因数值噪声翻转，8.0× 非对称惩罚导致最优轨迹在左右侧之间跳变。对称权重（1.0）消除跳变，w_lat_offset=4 + 紧密边界=0.3 已提供足够的居中力
16. **`lattice_test_node.cpp`** — 可视化优化：(a) viz_interval_ 3→1（10Hz 无节流），消除 300ms 更新间隙；(b) MarkerArray 顺序调整：ego CUBE + obstacle CUBE 放在最前，确保 RViz 先渲染关键对象；(c) 候选轨迹上限 50→20，减小 MarkerArray 消息体积
17. **`lattice_test_node.cpp`** — **RViz2 黄色警告修复**：(a) 删除无障碍物时的零尺寸退化 CUBE 标记（scale=0,0,0 α=0.0），某些 RViz2 版本中该标记导致 MarkerArray 渲染异常和黄色警告；(b) 所有动态标记 lifetime 0.3s→1.0s（参考线 0.5s→2.0s），防止因消息传输/渲染延迟导致标记过早过期；(c) Frame 0 发送一次性 DELETEALL 清除上次运行残留
18. **`lattice_test_node.cpp` + `rviz/lattice_test.rviz`** — **历史轨迹持久化**：新增 `persist_history` 和 `history_frame_interval` 两个 ROS2 参数。开启后每 N 帧生成一组 lifetime=∞ 的历史快照标记（`history_candidates` + `history_optimal` 命名空间），ID 使用 1,000,000 偏移量避免与实时标记冲突。RViz 配置增加对应命名空间默认开启。运行结束后实时标记淡出，历史轨迹永久保留在 RViz2 中供回看分析
19. **`math_utils.cpp:145-155`** — 修复 `InterpolateUsingLinearApproximation(TrajectoryPoint)` 在 `has_path_point()==true` 分支漏掉 Frenet 字段插值的 bug。轨迹组合器中每个点都设置了 `has_path_point_=true`，导致 `Evaluate()` 返回的 TrajectoryPoint 中 `d/d_d/d_dd/s_d/s_dd` 为未初始化垃圾值（表现为 `d0=4.28e+102`）
20. **`trajectory_evaluator.cpp:337-365`** — **LatOffsetCost 自归一化比例失效 bug（关键修复）**: 原实现使用 `sum(w*d²) / sum(w*|d|)` 自归一化比例作为横向偏移代价。这导致 `w_opposite_side_` 惩罚乘子完全失效——因为乘子同时影响分子和分母，最终比例值几乎不变（始终 ≈1.0）。后果：即使在 LCC 模式下设置 `w_opposite_side_=8.0`，从正 d 出发的轨迹跨越 d=0 到负 d 时，经过 d≈0 的点产生小的 `|d/bound|` 值，使比例反而更低，评价器持续选择负方向轨迹（`end_d=-0.3`），导致回中严重过冲至 d=-1.79。修复：改为均值平方和 `mean(w * (d/bound)²)`，使 `w_opposite_side_` 真正生效——从 d=0.49 出发，选择 end_d=-0.3 的代价 1.83 ≫ 选择 end_d=0.0 的代价 0.55。修复后车辆平滑回中（0.49→0.49→0.46→0.34→0.11），不再过冲
17. **NUDGE 动态 smin 重构** — `distance_to_obstacle` 参数贯穿调用链（`lattice_test_node` → `LatticePlan` → `GenerateTrajectoryBundles` → `GenerateLateralTrajectoryBundle` → `SampleLatEndConditions`）。障碍物前沿 Frenet s 由 polygon_points 计算得出。`distance_to_obstacle = obs_min_s - ego_s`，safe_smin 保证横向机动在障碍物前 10m 完成，kinematic_smin 保证物理可行性
21. **Scenario 3 重构为真实变道（target_lat_offset 贯穿）** — 详见下方
22. **pnc_msg 控制接口包** — 新增 `src/pnc_msg/` 包，定义 `TrajectoryPoint.msg`（13 字段：time/length/x/y/vx/vy/acc_x/acc_y/heading/curvature/pinch/jerk/reserved）和 `TrajPoints.msg`（header + 业务 flags + TrajectoryPoint[40]）。`lattice_test_node` 新增 `/lattice_trajectory` publisher，每周期将最优轨迹 40 点均匀重采样后发布，包含 curvature(κ)、pinch(dκ/ds)、jerk(da/dt 中心差分) 计算。用于下游控制模块接入与白盒调试
23. **仿真独立化 (9.1)** — 新建 `lattice_simulator_node` 独立可执行文件，100Hz 增量运动 + TF 广播 map→base_footprint；提取 `scenario_manager.h/cpp` 管理场景配置，规划器和仿真器共用；`lattice_test_node` 删除 TF broadcast 和 init_scenario 方法，改为从 scenario_manager 加载场景、通过 tf2 Buffer 查询 map→base_footprint 获取车辆位姿
24. **可视化增强 (9.3)** — 新增 URDF 车辆模型 (`urdf/lattice_car.urdf`)：黄色底盘 3.0×1.6×1.5m + 4 个圆柱车轮；新增道路边界 Marker (`/lattice_test/road_boundaries`)：左右白色边界线 + 黄色中线；RViz 升级：RobotModel Display + TF Display + ThirdPersonFollower 相机 (35m)；Launch 文件添加 robot_state_publisher + joint_state_publisher + lattice_simulator_node
25. **ALC 换道结束状态机** — `lattice_test_node.cpp` 新增 `alc_completed_flag_` + `alc_complete_debounce_counter_` 成员变量。换道完成判定：`|d0 - target_d| < 0.20m` AND `|heading_error| < 0.03 rad` 连续 5 帧防抖。完成后：(1) 整条 `reference_line_` y 坐标平移 `target_d` 到目标车道；(2) `alc_completed_flag_=true` 守卫下一帧不再设 -3.75 偏距；(3) `driving_mode` 切回 LCC；(4) 重新发布参考路径和道路边界 markers
26. **障碍物 Marker 持久化** — 障碍物 CUBE 标记从每帧重发改为仅 frame 0 发布一次（`frame_locked=true`, `lifetime=Duration::max()`）。消除 NUDGE 场景中因 10Hz 重发透明障碍物（α=0.5）导致的视觉闪烁，以及候选轨迹碰撞状态切换（cyan/orange）引起的连带闪烁
27. **Scenario 5: 4.2km S 弯高速赛道** — 新增 `init_scenario_5()`，从分段线性曲率剖面数值积分生成参考线。3 个弯道 + 缓和曲线过渡，8401 个参考点。车辆起点 (0,0,heading=0)，LCC 默认模式
28. **交互式换道** — 新增 `/lattice_test/lane_change_cmd` 订阅。`driving_mode_` 和 `target_lat_offset_` 从局部变量升级为成员变量，`lane_change_callback` 和 `timer_callback` 共享状态。换道中拒绝新指令，支持反复换道
29. **ALC 参考线平移改为法向量方式** — 从 `y += target_d` 改为 `x += target_d * (-sin(heading)), y += target_d * cos(heading)`。弯道上沿道路法向正确平移，向后兼容直线场景
30. **道路边界渲染改为法向偏移** — `publish_road_boundaries()` 中 `make_line`/`make_dashed_line` 改为法向偏移，弯道上白线/黄线跟随曲率

### Scenario 3 重构：真实变道（target_lat_offset）

**改动范围**: 8 个文件

**设计动机**: 旧 Scenario 3 使用预先弯曲的参考线（smoothstep y=0→3.5），车辆只需跟踪参考线即可"换道"。这不符合真实自动驾驶场景——现实中参考线通常是当前车道中心线，不存在预先计算好的期望变道路径。规划器必须根据目标车道偏移量自主生成变道轨迹。

**核心改动**: 新增 `target_lat_offset` 参数，贯穿整个规划管线：

```
PlanningTarget::set_target_lat_offset(-3.75)
  → LatticePlanner::LatticePlan(..., target_lat_offset)
    → Trajectory1dGenerator::GenerateTrajectoryBundles(..., target_lat_offset)
      → EndConditionSampler::SampleLatEndConditions(..., target_lat_offset)
    → TrajectoryEvaluator(..., target_lat_offset)
      → LatOffsetCost: (d - target)² / bound²  （原: d² / bound²）
```

**各文件改动详情**:

| 文件 | 改动 |
|------|------|
| `PlanningTarget.h` | 新增 `double target_lat_offset_` 成员，默认 0.0；新增 `target_lat_offset()` getter 和 `set_target_lat_offset()` setter |
| `end_condition_sampler.h/.cpp` | `SampleLatEndConditions()` 新增 `target_lat_offset` 参数。ALC 模式：end_d 候选 `{tgt, tgt*0.85, tgt*0.6, 0}`（比例自适应）。end_s 改为**时间制**：`current_speed × {7.5, 7.8, 8.2, 8.6}s`，**任意速度下换道时间恒定不变** |
| `trajectory1d_generator.h/.cpp` | `GenerateTrajectoryBundles()` 和 `GenerateLateralTrajectoryBundle()` 新增 `target_lat_offset` 参数，透传给 `EndConditionSampler` |
| `trajectory_evaluator.h/.cpp` | 构造函数新增 `target_lat_offset` 参数。`LatOffsetCost()` 改为 `(d - target)² / bound²`。新增**超调惩罚**：当 `deviation * target > 0`（即 d 超过 target 远离 0），代价 ×100，将五次多项式中间振荡压制在 ≤0.05m。ALC 权重：`w_lat_offset`=10.0, `w_lat_offset_bound_`=2.0 |
| `lattice_planner.h/.cpp` | `LatticePlan()` 新增 `target_lat_offset` 参数，透传给 `Trajectory1dGenerator` 和 `TrajectoryEvaluator` |
| `lattice_test_node.cpp` | `init_scenario_3()`: 参考线从弯曲（smoothstep y=0→3.5）改为直线 y=0（同 S1/S2），长度 500m。`timer_callback()`: S3 设置 `planning_target.set_target_lat_offset(-3.75)`，调用 `LatticePlan()` 时传入 `planning_target.target_lat_offset()` |

**效果对比**:

| | 旧方案 | 新方案 |
|------|--------|--------|
| 参考线 | 弯曲（y=0→3.5 smoothstep） | 直线 y=0（当前车道中心线） |
| 换道方式 | 跟踪预先弯曲的参考线，d≈0 即完成 | 规划器自主生成 d: 0→-3.75 轨迹 |
| ALC 候选数 | 6×9=54 | 4×4=16 |
| d 值范围 | ~0（跟踪参考线） | -3.75（到达目标车道） |
| 真实度 | 低：依赖人工构造的参考线 | 高：参考线仅表示当前车道，变道由规划器驱动 |

### 避障后回中机制（最终版）

**原理**：Cartesian→Frenet 转换自动计算横向偏差 d。LCC 模式通过 `w_lat_offset=4` + `w_lat_offset_bound_=0.3` 强制五次多项式从当前 d 单调收敛到 d=0。**不人工干预任何状态**——不清零 dd0/ddd0/theta_init/kappa_init，不锁定方向，无延迟。

**实现**（`timer_callback()`）：
1. **计算障碍物 max_s**: 遍历 `obstacles_[0].polygon_points`，用 `PathMatcher::GetPathFrenetCoordinate()` 求每个顶点的 Frenet s，取最大值
2. **检测通过**: `vehicle_state_.s0 > obs_max_s + 2.0`（预留 2m 缓冲）
3. **切换模式**: 设置 `post_obstacle_lcc_ = true`，`driving_mode = DrivingMode::LCC`。不做任何状态清零
4. **LCC 自动回中**: 横向采样 l∈[-0.3, 0.3]，smin=speed×4=88m，五次多项式在 ~100m 内平滑收敛到 d=0

---

## 驾驶模式系统（LCC / ALC / NUDGE）

### 架构

新增 `DrivingMode` 枚举（定义于 `include/Lattice_planner/PlanningTarget.h:8-13`），贯穿整个调用链：

```
lattice_test_node (每个场景确定模式, 计算 distance_to_obstacle)
  → LatticePlan(..., driving_mode, current_speed, distance_to_obstacle)
    → Trajectory1dGenerator::GenerateTrajectoryBundles(..., driving_mode, current_speed, distance_to_obstacle)
      → EndConditionSampler::SampleLatEndConditions(driving_mode, current_speed, distance_to_obstacle)
    → TrajectoryEvaluator(..., driving_mode)
      → Evaluate() 使用模式特定的评价权重
```

### 模式特定的横向采样参数

**修改位置**: `src/Lattice_planner/end_condition_sampler.cpp:37-67` — `SampleLatEndConditions()`

LCC 使用 `smin = speed × 4.0`。ALC 使用**时间制 end_s**：`end_s = speed × {7.5, 7.8, 8.2, 8.6}s`，任意速度下换道时间恒定 7.5–8.6s。NUDGE 使用**动态 smin**基于障碍物实际距离（详见下方公式）。

| 参数 | LCC（车道保持） | ALC（变道） | NUDGE（避让） |
|------|-----------------|-------------|---------------|
| **s 公式** | smin + [5, 10, 15, 20, 30] | `speed × {7.5, 7.8, 8.2, 8.6}s`（时间制）| smin_nudge + [0, 10, 15, 20, 30] |
| **smin** | speed×4 | —（直接按时间算 end_s）| max(safe, kinematic) 动态 |
| **换道耗时** | — | 7.5–8.6s（任意速度下恒定）| — |
| **l 值 (m)** | [-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3] | `{tgt, tgt*0.85, tgt*0.6, 0}`（比例自适应）| [-2.5, -2.0, -0.65, -0.6, -0.5, 0, 0.5, 0.6, 0.65, 2.0, 2.5] |
| **候选数量** | 5×7 = 35 | 4×4 = 16 | 5×11 = 55 |
| **用途** | 紧密围绕参考线，小幅度修正 | 固定时间制换道 + 超调惩罚 ×100 | 短距离快速偏移，±2.5 确保避开障碍物 |

> **NUDGE 动态 smin 公式**:
> ```
> safe_smin    = max(20.0, distance_to_obstacle - 10.0)   // 障碍物前 10m 完成偏离
> kinematic_smin = current_speed × 1.8                      // 运动学最小距离
> smin_nudge   = max(safe_smin, kinematic_smin)             // 取两者中较大值
> ```
> `distance_to_obstacle` 由 `lattice_test_node` 计算：遍历 `obstacles_[0].polygon_points`，用 `PathMatcher::GetPathFrenetCoordinate()` 求最小 Frenet s（障碍物前端），减去 `vehicle_state_.s0`。无障碍物时传 `INF`。

### 模式特定的轨迹评价权重

**修改位置**: `src/Lattice_planner/trajectory_evaluator.cpp:154-189` — `Evaluate()`

| 权重参数 | LCC | ALC | NUDGE | 说明 |
|----------|-----|-----|-------|------|
| `w_lat_offset` | **8.0** | **10.0** | **2.0** | 横向偏移惩罚。LCC 强居中，ALC 极强拉到目标车道，NUDGE 中等 |
| `w_opposite_side` | **8.0** | **1.0** | **1.0** | 跨目标线的额外惩罚。ALC 对称以允许自由接近目标 |
| `w_same_side` | **1.0** | **1.0** | **1.0** | 同侧偏移惩罚（所有模式统一） |
| `w_lat_comfort` | **2.0** | **1.0** | **1.0** | 横向舒适度。LCC 略高抑制振荡 |
| `w_lat_offset_bound` | **0.3** | **2.0** | **3.0** | 横向偏移归一化边界。ALC 收紧边界放大偏差代价 |
| **超调惩罚** | — | **×100** | — | `deviation * target > 0` 时触发，压制五次多项式中间振荡 ≤0.05m |

> **设计原理**: LCC 使用强居中权重确保车辆紧密跟踪参考线。ALC 使用极强目标权重 (lat_offset=10, bound=2.0) + **超调惩罚 ×100**（`deviation * target > 0` 时触发），将五次多项式振荡压制在 ≤0.05m。end_s 按时间制 `speed×{7.5,7.8,8.2,8.6}s`，任意速度下换道时间恒定。NUDGE 使用对称权重 + 宽边界(3.0)。

### LatOffsetCost 代价计算公式

**修改位置**: `src/Lattice_planner/trajectory_evaluator.cpp:337-365`

**原公式（已修复）**:
```
cost = sum_i(w_i * (d_i / bound)²) / sum_i(w_i * |d_i / bound|)
```
这是一个**自归一化比例**。w_i（same/opposite side 权重）同时出现在分子和分母中，
因此无论设为 1× 还是 8×，最终比例值都 ≈1.0，惩罚完全失效。
而且经过 d=0 的轨迹在 d≈0 处产生极小的 |d/bound|，反而拉低整体比例，
使评价器倾向于选择跨越中线的轨迹（例如从 d=0.38 选择 end_d=-0.3 而非 end_d=0.0）。

**新公式（target-aware + overshoot penalty）**:
```
dev = d_i - target_lat_offset
sq  = (dev / bound)²
if dev * target > 0:  sq *= 100   // overshoot: d beyond target (away from 0)
cost = (1/N) * sum_i( w_i * sq )
```
纯均值平方和，以 `target_lat_offset` 为参考中心。**超调惩罚**：当 `deviation` 与 `target` 同号（即 d 超越目标远离 0），平方代价 ×100，五次多项式中间振荡被压制在 ≤0.05m。`opposite_side` 判断基于 `(d - target)` 的符号变化：
- **LCC/NUDGE**: `target_lat_offset = 0` → 同旧公式，惩罚偏离参考线
- **ALC**: `target_lat_offset = -3.75`。end_d 候选 `{tgt, tgt*0.85, tgt*0.6, 0}`（比例自适应）。end_s = `speed × {7.5, 7.8, 8.2, 8.6}s`（时间制，任意速度恒定）。**超调惩罚 ×100** 压制五次多项式中间振荡 ≤0.05m

### 速度约束放宽（configs.yaml）

**修改位置**: `config/configs.yaml`

| 参数 | 原值 | 新值 | 原因 |
|------|------|------|------|
| `FLAGS_speed_upper_bound` | 5 | **40** | 允许 22 m/s 巡航 |
| `FLAGS_longitudinal_acceleration_lower_bound` | -1.0 | **-4.0** | 允许合理减速 |
| `FLAGS_longitudinal_acceleration_upper_bound` | 0.5 | **2.0** | 允许合理加速 |
| `FLAGS_longitudinal_jerk_lower_bound` | -1.0 | **-4.0** | 高速下加加速度放宽 |
| `FLAGS_longitudinal_jerk_upper_bound` | 1.2 | **4.0** | 高速下加加速度放宽 |
| `FLAGS_lateral_acceleration_bound` | 4.0 | **10.0** | 22 m/s 下 v²κ 可达较大值 |
| `FLAGS_kappa_bound` | 1.0 | **2.0** | 允许适度曲率 |

> **注意**: 巡航速度和初始速度直接在 `src/lattice_test_node.cpp` 各场景的 `target_speed_` 和 `vehicle_state_.v_init` 中设置（当前为 22.0 m/s），不在 configs.yaml 中。

### 纵向轨迹长度选择机制

纵向轨迹是**四次多项式** s(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴，拟合初始条件 [s₀, v₀, a₀] 和终点条件 [_, v_end, 0]，以时间 t 为自变量。

**轨迹长度的两个维度：**

| 维度 | 控制参数 | 位置 | 说明 |
|------|----------|------|------|
| **时间长度** | `FLAGS_trajectory_time_length` (8.0s) | `config/configs.yaml:12` | 规划时域上限，多项式以此时长生成速度剖面 |
| **空间分辨率** | `FLAGS_trajectory_space_resolution` (0.4m) | `config/configs.yaml:14` | 轨迹组合器按此弧长间隔离散化，决定轨迹点数 |
| **时间分辨率** | `FLAGS_trajectory_time_resolution` (0.1s) | `config/configs.yaml:13` | 轨迹组合器中的时间采样步长 |

**时间采样点生成**（`end_condition_sampler.cpp:86-101`）：
- 9 个时间采样点，从 `FLAGS_polynomial_minimal_param` (0.01s) 到 `FLAGS_trajectory_time_length` (8.0s)
- 每个时间点生成速度终点候选：`v_upper = min(FeasibleRegion::VUpper(t), ref_cruise_speed)` 到 `v_lower = FeasibleRegion::VLower(t)`
- 在 v_upper 和 v_lower 之间均匀采样中间速度点（数量由 `FLAGS_num_velocity_sample` 控制）

**空间覆盖范围估算**：
- 对每个 (t_end, v_end) 组合拟合四次多项式
- 轨迹终点 s ≈ s₀ + (v₀ + v_end)/2 × t_end
- 以 22 m/s 为例，v_upper 可达 ~38 m/s，t_end=8s → s_end ≈ 240m
- 参考线必须长于最大 s_end，否则 `trajectory_combiner.cpp:57-60` 的 `s > s_ref_max` 检查会截断轨迹

**如何修改：**
- 修改规划时域：编辑 `config/configs.yaml` 中 `FLAGS_trajectory_time_length`（无需重新编译）
- 修改速度采样数：编辑 `FLAGS_num_velocity_sample`（当前为 6，控制 v_upper/v_lower 之间的采样点数）
- 修改离散化精度：编辑 `FLAGS_trajectory_space_resolution` 或 `FLAGS_trajectory_time_resolution`
- **注意事项**：增大 `FLAGS_trajectory_time_length` 会增加每次规划的轨迹对数量（lon_bundle_size × lat_bundle_size），可能影响实时性。8.0s × 6 velocity samples × 2 bounds = ~108 条纵向轨迹束

### 场景更新

| 场景 | 模式 | 初始速度 | 目标速度 | 参考线长度 | 障碍物 |
|------|------|----------|----------|------------|--------|
| 1 | LCC | 22 m/s | 22 m/s | 500m | 无 |
| 2 | NUDGE | 22 m/s | 22 m/s | 500m | 4m×2m 矩形，位于 s=95m, y=-1.6（偏离中线 -1.6m） |
| 3 | ALC | 22 m/s | 22 m/s | 500m (直线) | 无。target_lat_offset=-3.75，规划器自主生成变道路径 |
| 5 | LCC (+ ALC) | 22 m/s | 22 m/s | 4200m (S 弯) | 无。默认 LCC，支持交互式换道 |

---

## 轨迹簇数量变化的说明

轨迹簇数量在每个规划周期会自然波动，原因如下：

1. **纵向轨迹束大小变化**: `SampleLonEndConditionsForCruising()` 根据 `v_upper = min(feasible_region_.VUpper(t), ref_cruise_speed)` 采样速度。随车速变化，可行速度范围不同。同时 `SampleLonEndConditionsForPathTimePoints()` 仅在 ST 图中存在障碍物时才产生候选轨迹。

2. **横向轨迹束大小按模式固定**: LCC=35, ALC=54, NUDGE=45。模式不变则数量不变。

3. **总配对数的波动**: 纵向束大小 (通常 44-50) × 横向束大小 (35/45/54)，总对数的微小波动是正常现象。

4. **有效轨迹数量**: `checked` 计数是经过纵向有效性过滤（`IsValidLongitudinalTrajectory`）和约束检查后实际评估的轨迹对数。`collected` 表示收集到的轨迹结果数（上限 200）。

---

## 所有可调参数汇总

### 横向采样参数（end_condition_sampler.cpp:37-67）

| 参数 | LCC | ALC | NUDGE | 类型 |
|------|-----|-----|-------|------|
| `smin` 公式 | speed×4 | —（时间制 end_s）| max(safe, kinematic) 动态 | 见动态公式 |
| `end_d_candidates` | [-0.3, 0.3] 7值 | `{tgt, tgt*0.85, tgt*0.6, 0}` 4值（比例自适应）| [-2.5, 2.5] 11值 | configs.yaml |
| `end_s_candidates` | 5值 | `speed × {7.5,7.8,8.2,8.6}s` 4值（时间制）| 5值 | configs.yaml |
| 候选总数 | 35 | 16 | 55 | — |

### 评价权重（trajectory_evaluator.cpp:158-181）

| 权重 | LCC | ALC | NUDGE | 类型 |
|------|-----|-----|-------|------|
| `LCC_w_lat_offset` 等 | 8.0 | 10.0 | 2.0 | configs.yaml |
| `LCC_w_opposite_side` 等 | 8.0 | 1.0 | 1.0 | configs.yaml |
| `LCC_w_same_side` 等 | 1.0 | 1.0 | 1.0 | configs.yaml |
| `LCC_w_lat_comfort` 等 | 2.0 | 1.0 | 1.0 | configs.yaml |
| `LCC_w_lat_offset_bound` 等 | 0.3 | 2.0 | 3.0 | configs.yaml |
| `ALC_overshoot_penalty` | — | 100.0 | — | configs.yaml |
| `preferred_lat_sign_tiebreaker` | 3.0 | 3.0 | 3.0 | configs.yaml |

### 约束参数（configs.yaml）

| 参数 | 值 | 用途 |
|------|-----|------|
| `FLAGS_speed_upper_bound` | 40 | ConstraintChecker 速度上限 |
| `FLAGS_speed_lower_bound` | -0.1 | ConstraintChecker 速度下限 |
| `FLAGS_longitudinal_acceleration_upper_bound` | 2.0 | ConstraintChecker 加速度上限 |
| `FLAGS_longitudinal_acceleration_lower_bound` | -4.0 | ConstraintChecker 加速度下限 |
| `FLAGS_longitudinal_jerk_upper_bound` | 4.0 | ConstraintChecker 加加速度上限 |
| `FLAGS_longitudinal_jerk_lower_bound` | -4.0 | ConstraintChecker 加加速度下限 |
| `FLAGS_lateral_acceleration_bound` | 10.0 | ConstraintChecker 横向加速度上限 |
| `FLAGS_kappa_bound` | 2.0 | ConstraintChecker 曲率上限 |
| `FLAGS_trajectory_time_length` | 8.0 | 最大预测时间 [s] |
| `FLAGS_trajectory_time_resolution` | 0.1 | 轨迹时间分辨率 |
| `FLAGS_lon_collision_buffer` | 2.0 | 静态障碍物纵向膨胀 (完整值) |
| `FLAGS_lat_collision_buffer` | 0.3 | 静态障碍物横向膨胀 (完整值) |
| `FLAGS_speed_lon_decision_horizon` | 250 | 纵向评价视界 [m]，需大于横向终点 s 值 |

### 基础配置参数（configs.yaml）

| 参数 | 值 | 用途 |
|------|-----|------|
| `FLAGS_vehicle_width` | 1.6 | 自车宽度 [m] |
| `FLAGS_vehicle_length` | 3.0 | 自车长度 [m] |
| `front_edge_to_center` | 2.0 | 车头到几何中心 [m] |
| `back_edge_to_center` | 1.0 | 车尾到几何中心 [m] |
| `FLAGS_default_reference_line_width` | 4.0 | 车道宽度 [m] |
| `FLAGS_weight_lon_objective` | 10.0 | 纵向目标权重 |
| `FLAGS_weight_lon_jerk` | 1.0 | 纵向加加速度权重 |
| `FLAGS_weight_lon_collision` | 5.0 | 纵向碰撞权重 |
| `FLAGS_weight_centripetal_acceleration` | 1.0 | 向心加速度权重 |
| `FLAGS_weight_lat_offset` | 0.3 | 横向偏移基础权重（实际被模式特定值覆盖） |
| `FLAGS_weight_lat_comfort` | 2.0 | 横向舒适度基础权重（实际被模式特定值覆盖） |
| `FLAGS_num_velocity_sample` | 6 | 速度采样数 |
| `FLAGS_numerical_epsilon` | 1e-7 | 数值 epsilon，防除零 |
| `FLAGS_prediction_total_time` | 5.0 | 障碍物预测总时间 [s] |
| `eval_time_interval` | 0.1 | ST 图时间分辨率 [s] |

---

## 障碍物位置修改指南

障碍物定义在 `src/scenario_manager.cpp` 的 `build_scenario_2()` 方法中。

**修改位置**: `src/scenario_manager.cpp` — `build_scenario_2()`

关键修改方式:

```cpp
// 障碍物中心位置 (行 163-164)
obs.centerpoint.position.x = 95.0;   // 沿参考线方向 (s)，单位 m
obs.centerpoint.position.y = 0.2;    // 横向偏移 (d)，单位 m，相对于参考线 y=0
```

| 修改内容 | 修改位置 | 行号 | 说明 |
|----------|----------|------|------|
| 纵向位置 (s) | `obs.centerpoint.position.x` | 163 | 沿参考线的距离。参考线 y=0 沿 x 方向，故 x 即为 s |
| 横向偏移 (d) | `obs.centerpoint.position.y` | 164 | 垂直参考线方向的偏移量。0.0=车道中心，正值=左侧 |
| 障碍物尺寸 | `obs.obstacle_length` / `obs.obstacle_width` | 156-157 | 默认 4.0m×2.0m |
| 多边形顶点 | `obs.polygon_points` / `Box2d` / `PerceptionBoundingBox` | 171-177 | 必须与 centerpoint 同步更新 |

**修改示例** — 将障碍物移到 s=60m, d=-0.3m（中心偏右）:
```cpp
// Line 163-164: 改为
obs.centerpoint.position.x = 60.0;
obs.centerpoint.position.y = -0.3;

// Line 171-177: 多边形和 Box2d 同步更新
obs.polygon_points = {
    Vec2d(60.0 - hl, -0.3 - hw), Vec2d(60.0 + hl, -0.3 - hw),
    Vec2d(60.0 + hl, -0.3 + hw), Vec2d(60.0 - hl, -0.3 + hw)
};
obs.SetPerceptionBoundingBox(
    Box2d(Vec2d(60.0, -0.3), 0.0, obs.obstacle_length, obs.obstacle_width));
obs.SetPerceptionPolygon(common::math::Polygon2d(obs.polygon_points));
```

> **注意**: 障碍物 s 位置应与 NUDGE 模式横向终点 s 值（93-103m at 22 m/s）匹配。障碍物放置过近（如 s=30m）时，车辆来不及在纵向上到达横向轨迹发散区，无法生成有效绕行轨迹。若需调整速度，`smin = speed × 4.0`，应确保障碍物 s 在终点 s 范围的 30%-80% 处。
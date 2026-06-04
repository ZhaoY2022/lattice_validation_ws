# Lattice Validation WS vs Planning with ROS2 Course — 详细对比报告

## 总览

| 维度 | 当前项目 (test02) | 参考项目 (test03) |
|------|-------------------|-------------------|
| 定位 | Apollo Lattice 算法离线验证框架 | ROS2 规划教学课程项目 |
| 代码规模 | ~8,000 行 C++ | ~3,500 行 C++ + Python |
| 包数量 | 2 (pnc_msg, lattice_standalone_test) | 3 (base_msgs, planning, data_plot) |
| 可执行文件 | 1 (lattice_test_node) | 5 (planning_process, pnc_map_server, global_path_server, car_move_cmd, obs_move_cmd) |

---

## 1. 架构与代码框架

### 1.1 模块化程度

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 架构风格 | 单体 (monolithic) | 模块化 (modular) |
| 包组织 | 核心算法 + ROS 胶水混合在同一节点 | 清晰分层: msg定义/规划/可视化 |
| 库组织 | 1个静态库 (lattice_core) + 1个可执行文件 | 7个共享库 + 5个可执行文件 |
| 模块边界 | lattice_planner, trajectory_evaluator 等通过头文件分离 | 每个模块独立 CMakeLists.txt, 共享库, 明确 public API |

**参考项目的模块拆分**:
```
planning/src/
├── common/              # config_reader (共享库), mathlibs (共享库)
├── pnc_map_creator/     # pnc_map_creator (共享库) + pnc_map_server (可执行)
├── global_planner/      # global_planner (共享库) + global_path_server (可执行)
├── reference_line/      # reference_line (共享库, OSQP平滑)
├── decision_center/     # decision_center (共享库, 障碍物决策)
├── local_planner/       # local_planner (共享库, 路径+速度+合并)
├── vehicle_info/        # vehicle_info (共享库, 主车+障碍车)
├── move_cmd/            # car_move_cmd (可执行) + obs_move_cmd (可执行)
├── planning_process/    # planning_process (可执行, 主循环)
└── test/                # osqp_test (可执行)
```

**差距**: 当前项目所有功能耦合在单个 `lattice_test_node.cpp` + `lattice_core` 静态库中, 无独立的仿真节点、地图服务、全局路径服务。

### 1.2 可执行文件角色

| 可执行文件 | 当前项目 | 参考项目 | 职责 |
|-----------|----------|----------|------|
| lattice_test_node | **有** | — | 规划+仿真+可视化全合一 |
| planning_process | — | **有** | 纯规划编排节点 (主循环 10Hz) |
| pnc_map_server | — | **有** | 地图服务 (service 模式, 返回 PNCMap) |
| global_path_server | — | **有** | 全局路径规划服务 (service 模式) |
| car_move_cmd | — | **有** | 主车运动仿真 (tf 广播) |
| obs_move_cmd | — | **有** | 障碍车运动仿真 (tf 广播) |

**差距**: 当前项目无独立仿真节点 — 车辆运动直接在 timer_callback 中"瞬移"; 无服务端架构 — 地图和全局路径直接在测试节点中硬编码生成。

### 1.3 设计模式

| 模式 | 当前项目 | 参考项目 |
|------|----------|----------|
| 策略模式 | 无 — 驾驶模式通过 if/switch 分支 | GlobalPlannerBase/PNCMapCreatorBase 纯虚接口 |
| 服务化初始化 | 无 — 所有数据在节点内直接生成 | map_server + global_path_server (ROS2 service) |
| 工厂模式 | 无 | PNCMapCreator 根据 map_type 枚举创建 Straight/STurn |
| 命名空间隔离 | 无 | /car, /obs_car, /planning 三组命名空间 |

---

## 2. 规划算法

### 2.1 算法范式

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 方法 | **Lattice 采样规划** | **多项式插值规划** |
| 搜索策略 | 纵/横向终点采样 → 笛卡尔积配对 → 代价排序 → 碰撞校验 | SL 决策点序列 → 五次多项式插值 → 坐标转换 |
| 时域 | 8s, 100ms 分辨率 (80 点) | 无固定时域, 基于弧长 (path_size=80 点) |
| 候选数 | 1750 对/周期 (50 lon × 35 lat) | 1 条路径/周期 |

### 2.2 Frenet 解耦

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 纵向采样 | **四次多项式** (巡航) / **五次多项式** (跟车/停车), 最多 50 个 | **无** (速度规划是 stub) |
| 横向采样 | **五次多项式** d(s), 最多 35 个, 按 LCC/ALC/NUDGE 模式分别采样 | **五次多项式** d(s), 按 SL 决策点逐段插值 |
| 2D 合成 | TrajectoryCombiner → 笛卡尔积 → Frenet→Cartesian | LocalPathPlanner → 逐段插值 → Frenet→Cartesian |
| 代价函数 | **六级加权**: 纵向目标偏差 + jerk + 碰撞 + 向心加速度 + 横向偏移 + 舒适度 | **无代价函数** (直接生成唯一路径) |

**关键差异**: 当前项目是真正的"规划器" (搜索-评估-选择), 产生候选轨迹簇; 参考项目是"插值器" (决策→插值→输出), 只产生一条路径。

### 2.3 障碍物处理

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 碰撞检测 | **AABB 逐点检测** (CollisionChecker), 对每条候选轨迹的每个点 | 无碰撞检测 — 仅决策层面做避障 |
| ST 图 | **PathTimeGraph** — 障碍物投影到 (s,t) 平面, 提取上下边界用于跟车/超车采样 | 无 |
| 障碍物预测 | **PredictionQuerier** — 查询障碍物未来轨迹 | 无 — 障碍物仅沿初始方向匀速移动 |
| 决策方式 | 隐式 (通过采样空间+代价函数自然避障) | 显式 (DecisionCenter 输出 LEFT_PASS/RIGHT_PASS/STOP) |

### 2.4 驾驶模式

| | 当前项目 | 参考项目 |
|------|----------|----------|
| LCC (车道保持) | **有** — 7 个 end_d × 5 个 end_s = 35 个横向候选 | 无显式模式 — 始终沿参考线 |
| ALC (自主变道) | **有** — 比例制 end_d + 时间制 end_s + 超调惩罚 | 无 |
| NUDGE (绕行) | **有** — 11 个 end_d × 动态 smin | 有 — DecisionCenter 决策 LEFT_PASS/RIGHT_PASS |

### 2.5 参考线平滑

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 方法 | 无平滑 — 直接使用原始参考点 | **OSQP 二次规划平滑** (penalty on curvature + deviation) |
| 工具 | — | OsqpEigen + Eigen3 |

### 2.6 多项式工具

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 五次多项式 | QuinticPolynomial (6阶方阵求解) | PolynomialCurve::quintic_polynomial (Vandermonde 6x6 求解) |
| 四次多项式 | QuarticPolynomial (5阶方阵求解) | 无 |
| 三次样条 | CubicSpline1D/2D | 无 |
| 实现方式 | 自研代码 | 自研代码 |

---

## 3. 仿真方案

### 3.1 车辆模型

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 可视化 | Marker CUBE (1.6m × 3.0m, 黄色) | **URDF/Xacro RobotModel** (3D 模型含底盘/车轮/相机/雷达) |
| 运动模拟 | "瞬移" — 直接更新 (x, y, theta, v) 到轨迹点 | **TF 插值** — car_move_cmd 沿轨迹点以恒定速度移动, 通过 tf 广播位置 |
| 车轮/关节 | 无 | 前轮转向 knuckle joint ±30°, 后轮 continuous rolling joint |
| 传感器模型 | 无 | camera (红色方块), radar (蓝色圆柱) — 仅为装饰 |

### 3.2 障碍车仿真

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 可视化 | Marker CUBE (橙色/红色) | URDF RobotModel (蓝色 box 3.0×1.6×1.2) |
| 运动 | 静止障碍物 (固定在场景中) | **obs_move_cmd 独立节点** — 3辆障碍车沿初始方向匀速移动, 通过 tf 广播 |
| 初始化 | 在 lattice_test_node 中硬编码创建 Obstacle 对象 | 在 planning_process 中通过 YAML 配置创建, ObsCar::Init 广播初始 tf |

### 3.3 独立仿真节点

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 仿真与规划分离 | **否** — 同一 timer_callback 中完成规划+状态更新+可视化 | **是** — move_cmd 独立节点, 可单独启动; 规划节点通过 tf 获取车辆位置 |
| 启动方式 | 单一 launch 文件 | 分离: planning_launch.py (规划) + move_cmd_launch.py (仿真) |
| tf 使用 | 单向广播 (map→base_link, 用于 RViz 相机) | 双向: 规划节点通过 tf 获取车辆位置, 仿真节点通过 tf 广播车辆位置 |

---

## 4. RViz 可视化

### 4.1 显示项对比

| 显示类型 | 当前项目 | 参考项目 |
|----------|----------|----------|
| Grid (地面网格) | 关闭 | 开启 |
| **RobotModel (3D 车辆)** | **无** ← 关键差距 | 主车 (黄色) + 3 辆障碍车 (蓝色) |
| TF (坐标轴) | 无 | 开启 (显示所有 frame 的坐标轴和名称) |
| Axes (世界坐标) | 无 | 开启 |
| Path (路径线) | 3 条: Reference(蓝), Optimal(绿), Candidates(灰) | 2 条: 参考线(黄, Billboard), 局部路径(绿, 粗线 Line) |
| Marker (全局路径) | 无 | 红色 LINE_STRIP, frame_locked |
| MarkerArray (地图) | 无 | pnc_map — 道路中线(虚线)+左右边界(实线) |
| **MarkerArray (规划器) ← 当前项目独创** | ego + optimal + candidates + obstacles + reference + history_* | — |

### 4.2 相机配置

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 相机类型 | Orbit (自由轨道) | **ThirdPersonFollower** (第三人称跟随) |
| Target Frame | base_link (动态跟随) | base_footprint / Fixed Frame |
| 距离 | 130m | 35m |
| 视角 | 俯视 (Pitch=0.615, Yaw=3.38) | 后方跟随 (Pitch=0.35, Yaw=3.25) |

### 4.3 命名空间可视化

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 命名空间组织 | viz_markers 的 namespace 字段 (ego, optimal, candidates, etc.) | ROS2 namespace (/car, /obs_car, /planning) + RobotModel/TF 分别显示 |
| 独立开关 | 可在 RViz 中单独开关每个 namespace 的 MarkerArray | 每个 RobotModel/Path/Marker 是独立 Display |

---

## 5. 消息系统

### 5.1 自定义消息

| | 当前项目 (pnc_msg) | 参考项目 (base_msgs) |
|------|---------------------|----------------------|
| 消息数量 | 2 msg | 11 msg + 2 srv |
| 轨迹点 | **TrajectoryPoint**.msg (13 字段, 控制接口) | **LocalPathPoint**.msg (16 字段, Frenet+Cartesian) |
| 轨迹 | **TrajPoints**.msg (header + 10业务flags + 40点数组) | **LocalTrajectory**.msg (header + LocalTrajectoryPoint[]) |
| 路径 | — | **LocalPath**.msg, **LocalPathPoint**.msg |
| 速度规划 | — | **LocalSpeeds**.msg, **LocalSpeedsPoint**.msg |
| 参考线 | — | **Referline**.msg, **ReferlinePoint**.msg |
| 地图 | — | **PNCMap**.msg (道路长度+宽度+Marker) |
| 障碍物 | — | **ObsInfo**.msg |
| 可视化聚合 | — | **PlotInfo**.msg |
| 服务 | — | **PNCMapService**.srv, **GlobalPathService**.srv |

**关键差异**:
- 当前项目: 消息面向**控制下游** (速度/加速度/曲率/pinch/jerk 等物理量)
- 参考项目: 消息面向**架构分层** (路径/速度/轨迹/地图/障碍物各有独立消息类型)

### 5.2 Topic 对比

| Topic | 当前项目 | 参考项目 |
|-------|----------|----------|
| 全局路径 | 无独立 topic | `/planning/global_path` (nav_msgs/Path) |
| 全局路径 RViz | 无 | `/planning/global_path_rviz` (Marker) |
| PNC 地图 | 无 | `/planning/pnc_map` (PNCMap) |
| PNC 地图 RViz | 无 | `/planning/pnc_map_markerarray` (MarkerArray) |
| 参考线 | `/lattice_test/reference_path` (nav_msgs/Path) | `/planning/reference_line` (nav_msgs/Path) |
| 局部路径 | — | `/planning/local_path` (nav_msgs/Path) |
| 局部轨迹 | `/lattice_trajectory` (TrajPoints, 控制接口) | `/planning/local_trajectory` (LocalTrajectory) |
| 候选轨迹簇 | `/lattice_test/candidate_trajectories` (nav_msgs/Path) | 无 |
| 最优轨迹 | `/lattice_test/optimal_trajectory` (nav_msgs/Path) | 无 |
| 可视化标记 | `/lattice_test/viz_markers` (MarkerArray) | 无 |

---

## 6. 配置管理

### 6.1 配置方式

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 配置文件 | **单个 YAML** (`configs.yaml`, ~110行) | **多个 YAML** (按场景: static_obs / onlane_obs / dynamic_obs) |
| 配置加载 | Config.cpp (自研 yaml-cpp 读取) | **ConfigReader** 类 (yaml-cpp, 通过 ament_index 定位 share 目录) |
| 结构体定义 | 单个全局 struct (Configs.h) | 分模块 struct: VehicleStruct, PNCMapStruct, DecisionStruct, 等 |
| 参数可调性 | **全部 YAML 化** — 采样数/权重/边界/惩罚系数 | 基本参数: 车辆尺寸/道路宽度/缓冲区/检测范围 |
| 环境变量 | `LATTICE_CONFIG_PATH` | 无 (固定在 `planning_static_obs_config.yaml`) |

**关键差异**: 当前项目的配置系统更**灵活** (所有算法参数均可 YAML 调参); 参考项目的配置系统更**模块化** (每个模块独立配置结构体, 但配置文件固定不可切换)。

### 6.2 Launch 参数

| | 当前项目 | 参考项目 |
|------|----------|----------|
| 运行时参数 | `scenario` (1/2/3), `persist_history` (bool), `history_frame_interval` (int) | 无 — 修改配置需重新编译或修改 YAML |
| 环境变量注入 | `SetEnvironmentVariable('LATTICE_CONFIG_PATH', ...)` | 无 |

---

## 7. 启动与编排

### 7.1 Launch 文件

| | 当前项目 | 参考项目 |
|------|----------|----------|
| Launch 文件数 | 1 (`run_scenario.launch.py`) | 2 (`planning_launch.py` + `move_cmd_launch.py`) |
| 命名空间 | 无 | `/car`, `/obs_car`, `/planning` |
| robot_state_publisher | 无 | 2 个 (分别发布主车和障碍车 URDF) |
| joint_state_publisher | 无 | 2 个 |
| xacro 处理 | 无 | Command + ParameterValue 动态转换 |

### 7.2 启动流程

**参考项目**:
```
planning_launch.py 启动:
  → robot_state_publisher (car, obs) ← URDF 模型
  → joint_state_publisher (car, obs)
  → pnc_map_server ← 地图服务启动
  → global_path_server ← 全局路径服务启动
  → planning_process ← 规划节点启动, 内部:
       → vehicle_spawn (tf 广播初始位置)
       → connect_server → map_request (获取地图)
       → connect_server → global_path_request (获取全局路径)
       → 10Hz timer → planning_callback

move_cmd_launch.py 启动:
  → car_move_cmd ← 订阅 /planning/local_trajectory, 广播 tf
  → obs_move_cmd ← 10Hz 广播 3 辆障碍车 tf
```

**当前项目**:
```
run_scenario.launch.py 启动:
  → lattice_test_node ← 100ms timer 内完成所有工作:
       → 硬编码生成参考线
       → 硬编码创建障碍物
       → 运行 LatticePlan
       → 更新车辆状态 (瞬移)
       → 发布所有可视化
       → 写 CSV 日志
  → rviz2
```

---

## 8. 参考项目缺失 (当前项目已有的独特功能)

以下功能是当前项目**有**而参考项目**没有**的, 应保留并考虑集成:

| 功能 | 说明 |
|------|------|
| **Lattice 采样规划** | 1750 候选/周期, 代价评估, 冲突校验 → 真正的搜索式规划器 |
| **ST 图障碍物处理** | 障碍物预测轨迹投影, 跟车/超车采样边界 |
| **三模式驾驶** | LCC/ALC/NUDGE 完整实现, 含模式特定采样和代价 |
| **全参数 YAML 化** | 所有算法参数可调, 无需重编译 |
| **候选轨迹簇可视化** | 50 条灰线 + 绿色最优, 可选持久化历史 |
| **代价函数体系** | 六级加权代价, 含横向超调惩罚 |
| **控制接口消息** | TrajPoints 含 speed/accel/curvature/pinch/jerk, 可直接对接下游控制 |
| **CSV 日志** | 每周期规划数据写入 CSV, 便于离线分析 |

---

## 9. 当前项目的关键差距 (需要改进)

按优先级排序:

### 9.1 高优先级 — 仿真独立化

| 差距 | 参考项目方案 | 改进建议 |
|------|-------------|----------|
| 规划与仿真耦合 | car_move_cmd + obs_move_cmd 独立节点 | **创建 lattice_simulator 节点**: 订阅最优轨迹, 通过 tf 广播车辆位置 |
| 无 URDF 车辆模型 | Xacro 3D 模型 (RobotModel) | **添加 URDF 模型**: 简单的 box 底盘 + 车轮, 通过 robot_state_publisher 发布 |
| "瞬移"无物理感 | tf 增量插值移动 | **改为增量运动**: 读取轨迹点速度, dt 积分更新位置 |

### 9.2 高优先级 — 模块化拆分

| 差距 | 参考项目方案 | 改进建议 |
|------|-------------|----------|
| 单一可执行文件 | 5 个可执行文件 + 7 个共享库 | **拆分 lattice_test_node**: 规划逻辑 → lattice_planner_node, 场景管理 → scenario_manager 库 |
| 无服务边界 | service 获取地图/全局路径 | **创建 MapService**: 通过 service 提供参考线/全局路径 |
| 库组织粗粒度 | 每模块独立 CMakeLists.txt + 共享库 | **按模块拆分 lattice_core**: 拆分 LatticePlanner / TrajectoryEvaluator / CollisionChecker 为独立库 |

### 9.3 中优先级 — 可视化增强

| 差距 | 参考项目方案 | 改进建议 |
|------|-------------|----------|
| 无 RobotModel | main_car 黄色底盘 + 车轮 + 传感器模型 | **添加 RobotModel Display** |
| 无道路边界显示 | pnc_map_markerarray (LINE_STRIP 左右边界+虚线中线) | **添加道路 Marker**: 参考线两侧 ±road_width |
| 无 TF 显示 | TF Display 显示所有 frame | **添加 TF Display**: 开启 map→base_link 可见性 |
| 相机是手动 Orbit | ThirdPersonFollower 自然地跟随 | **改为 ThirdPersonFollower** + 近距 (35m) |

### 9.4 中优先级 — 消息扩展

| 差距 | 参考项目方案 | 改进建议 |
|------|-------------|----------|
| 消息量少 | 11 msg + 2 srv | **扩展 pnc_msg**: 添加 LocalPath / LocalSpeeds / ObsInfo / PlotInfo |
| 无服务定义 | PNCMapService / GlobalPathService | **添加 service**: MapService / ScenarioService |
| 消息方向偏向控制 | 架构分层消息 | **保持现有控制接口 + 补充架构消息** |

### 9.5 低优先级 — 额外功能

| 差距 | 参考项目方案 | 改进建议 |
|------|-------------|----------|
| 无实时绘图 | data_plot (Python matplotlib, 目前是 stub) | **添加 Plot 节点**: 实时显示速度/加速度/横向偏差时域曲线 |
| 无 OSQP 平滑 | OSQP 参考线平滑 (curvature penalty) | **可选集成 OSQP**: 在参考线预处理前添加平滑步骤 |
| 配置文件不可按场景切换 | 多个 YAML 文件 | **支持多配置文件**: `--config static_obs.yaml` launch 参数 |

---

## 10. 改进路线图建议

### Phase 1: 仿真独立化 (1-2 天)

```
新增:
  src/lattice_simulator/
    ├── car_simulator.cpp/h      # 订阅最优轨迹, tf 广播车位置
    ├── scenario_manager.cpp/h   # 管理不同场景 (替代硬编码)
    └── CMakeLists.txt

修改:
  lattice_test_node.cpp → 删除车辆状态更新, 仅保留规划逻辑 + 可视化
  CMakeLists.txt       → 添加 lattice_simulator 可执行文件
  launch/              → 拆分为 planning_launch.py + simulator_launch.py
```

### Phase 2: 可视化升级 (1 天)

```
新增:
  urdf/main_car.xacro              # 简单底盘 + 车轮模型
  rviz/ → 添加 RobotModel, TF, Axes Display
  launch/ → 添加 robot_state_publisher, joint_state_publisher
```

### Phase 3: 模块化重构 (2-3 天)

```
拆分 lattice_core 为独立共享库:
  liblattice_planner.so
  libtrajectory_evaluator.so
  libcollision_checker.so
  libfrenet_converter.so

新增服务节点:
  reference_line_server (service 模式提供参考线)

新增消息:
  LocalPath.msg / LocalSpeeds.msg / ScenarioInfo.msg
```

### Phase 4: 功能增强 (可选)

```
- OSQP 参考线平滑
- 多场景配置文件切换
- data_plot 实时可视化曲线
- 障碍车运动仿真 (动态场景)
```

---

## 附录: 文件对照表

| 功能 | 当前项目路径 | 参考项目路径 |
|------|-------------|-------------|
| 主节点 | `src/lattice_test_node.cpp` | `src/planning_process/planning_process.cpp` |
| 核心规划器 | `src/Lattice_planner/lattice_planner.cpp` | `src/local_planner/local_path/local_path_planner.cpp` |
| 多项式 | `src/Polynomial/QuinticPolynomial.cpp` | `src/common/math/polynomial_curve.cpp` |
| Frenet 转换 | `src/Frenet/cartesian_frenet_conversion.cpp` | `src/common/math/curve.cpp` |
| 碰撞检测 | `src/Lattice_planner/collision_checker.cpp` | (无 — 仅决策层面) |
| 代价评估 | `src/Lattice_planner/trajectory_evaluator.cpp` | (无 — 直接插值) |
| 障碍物决策 | `src/Lattice_planner/end_condition_sampler.cpp` | `src/decision_center/decision_center.cpp` |
| 参考线 | `src/ReferenceLine/reference_point.cpp` | `src/reference_line/reference_line_creator.cpp` |
| 参考线平滑 | (无) | `src/reference_line/reference_line_smoother.cpp` (OSQP) |
| 配置 | `src/common/Config.cpp` + `config/configs.yaml` | `src/common/config_reader/config_reader.cpp` + `config/*.yaml` |
| 消息 | `src/pnc_msg/msg/TrajPoints.msg` | `src/base_msgs/msg/LocalTrajectory.msg` |
| 车辆模型 | (无 — Marker CUBE) | `urdf/main_car/car_base.xacro` |
| 仿真 | (耦合在 lattice_test_node.cpp) | `src/move_cmd/car_move_cmd.cpp` |
| RViz | `rviz/lattice_test.rviz` (Orbit) | `rviz/planning.rviz` (ThirdPersonFollower) |
| Launch | `launch/run_scenario.launch.py` | `launch/planning_launch.py` + `launch/move_cmd_launch.py` |

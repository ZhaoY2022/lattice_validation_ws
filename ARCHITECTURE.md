# Lattice 轨迹规划器 — 框架与算法详解

## 1. 项目概述

本项目是一个独立的 ROS2 Humble 验证工作空间，将 Apollo Lattice 轨迹规划算法提取为可离线运行的仿真测试框架。车辆状态通过"瞬移"到每帧最优轨迹点来模拟闭环控制，在 RViz2 中可视化所有候选轨迹。

**核心能力**: 给定参考线（当前车道中心线）、障碍物列表、驾驶模式，规划器自主生成最优无碰撞轨迹。

---

## 2. 框架结构

```
lattice_validation_ws/
├── src/lattice_standalone_test/
│   ├── config/configs.yaml              # 所有可调参数（无需重编译）
│   ├── include/Lattice_planner/         # 14 个头文件
│   │   ├── lattice_planner.h            # 主入口
│   │   ├── trajectory1d_generator.h     # 1D 轨迹生成（Frenet 解耦）
│   │   ├── end_condition_sampler.h      # 终点状态采样
│   │   ├── trajectory_evaluator.h       # 轨迹代价评估与排序
│   │   ├── trajectory_combiner.h        # 纵/横向 1D 轨迹合成为 2D
│   │   ├── constraint_checker.h         # 运动学约束校验
│   │   ├── collision_checker.h          # 碰撞检测
│   │   ├── path_time_graph.h            # ST 图构建
│   │   ├── prediction_querier.h         # 障碍物预测查询
│   │   ├── feasible_region.h            # 可行区域（s 上下界）
│   │   ├── PlanningTarget.h             # 规划目标定义
│   │   └── lattice_trajectory1d.h       # 1D 轨迹容器（外推处理）
│   ├── include/Polynomial/              # 多项式曲线
│   │   ├── curve1d.h                    # 基类
│   │   ├── quintic_polynomial_curve1d.h # 五次多项式（横向）
│   │   └── quartic_polynomial_curve1d.h # 四次多项式（巡航纵向）
│   ├── include/Frenet/                  # Cartesian ↔ Frenet 转换
│   ├── include/ReferenceLine/           # 参考线点（含 kappa/dkappa）
│   ├── include/Obstacle/                # 障碍物模型
│   ├── include/path/                    # 轨迹点数据结构
│   ├── include/common/                  # Vec2d, Box2d, Configs 等
│   └── src/                             # 与 include/ 镜像的 .cpp 文件
│       └── lattice_test_node.cpp        # ROS2 仿真节点
```

---

## 3. 算法流水线

每一规划周期（100ms）执行以下步骤：

```
Step 1: 参考线离散化
    ToDiscretizedReferenceLine(ref_points) → vector<PathPoint>

Step 2: 匹配点计算
    PathMatcher::MatchToPath() / FindProjectionPoint()
    → 找到自车在参考线上的投影点，获取 Frenet 基准

Step 3: Cartesian → Frenet 转换
    CartesianFrenetConverter::cartesian_to_frenet()
    → 输出 init_s = [s0, s_dot, s_ddot], init_d = [d0, d_prime, d_double_prime]

Step 4: 构建 ST 图
    PathTimeGraph(obstacles, ref, s0, s0+30m, 0, 8s, init_d)
    → 障碍物在 ST 平面的投影，用于跟车/超车采样

Step 5: 1D 轨迹生成（Frenet 解耦）
    Trajectory1dGenerator.GenerateTrajectoryBundles()
    ├── 纵向 (s-t): 四次/五次多项式 → lon_trajectory_bundle
    └── 横向 (d-s): 五次多项式     → lat_trajectory_bundle

Step 6: 2D 轨迹合成
    TrajectoryCombiner.Combine(lon, lat, ref) → DiscretizedTrajectory
    → 对每个 (s_i, d_i) 用参考线点转换为 Cartesian (x, y, theta, kappa)

Step 7: 轨迹评估与排序
    TrajectoryEvaluator → 优先队列（代价升序）
    ├── 纵向代价: 目标速度偏差, 加加速度, 碰撞风险
    ├── 横向代价: 偏离目标车道, 舒适度(横向加加速度)
    └── 综合代价: 加权求和

Step 8: 约束校验 + 碰撞检测
    ConstraintChecker.ValidTrajectory() + CollisionChecker.InCollision()
    → 按代价升序检查，取第一个同时通过两者校验的轨迹

Step 9: 车辆状态更新
    trajectory[1] → (x, y, theta, v) → 更新 vehicle_state_
```

---

## 4. 关键算法

### 4.1 Frenet 坐标解耦

Lattice 的核心思想是将二维轨迹规划分解为两个一维问题：

| 维度 | 自变量 | 多项式 | 物理意义 |
|------|--------|--------|----------|
| 纵向 s(t) | 时间 t | 四次（巡航）/五次（跟车/停车）| 速度规划 v-t |
| 横向 d(s) | 纵向距离 s | 五次 | 路径规划 d-s |

纵横向独立采样后，通过笛卡尔积配对，再用 `TrajectoryCombiner` 合成。

### 4.2 多项式拟合

**横向 — 五次多项式 d(s) = a₀ + a₁s + a₂s² + a₃s³ + a₄s⁴ + a₅s⁵**

6 个系数由 6 个边界条件确定：
- 起点: [d₀, d'₀, d''₀]
- 终点: [d_end, 0, 0]（零速度、零加速度到达目标横向位置）

**纵向巡航 — 四次多项式 s(t) = b₀ + b₁t + b₂t² + b₃t³ + b₄t⁴**

5 个系数：起点 [s₀, v₀, a₀]，终点 [v_end, 0]。终点 s 不固定（根据时间自动确定）。

**纵向跟车/停车 — 五次多项式**：终点 [s_end, v_end, 0] 全部固定。

### 4.3 终点采样策略

#### 横向终点采样 (end_condition_sampler.cpp)

按驾驶模式分别定义 end_d 和 end_s 候选集：

| 模式 | end_d 候选 | end_s 候选 | 总数 |
|------|-----------|-----------|------|
| **LCC** | [-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3] 7 值 | speed×4 + {5,10,15,20,30} 5 值 | 35 |
| **ALC** | `{tgt, tgt×0.85, tgt×0.6, 0}` 4 值（比例自适应）| `speed × {7.5, 7.8, 8.2, 8.6}s` 4 值（时间制）| 16 |
| **NUDGE** | [-2.5, -2.0, -0.65, -0.6, -0.5, 0, 0.5, 0.6, 0.65, 2.0, 2.5] 11 值 | smin_nudge + {0,10,15,20,30} 5 值（动态）| 55 |

**ALC 关键设计**:
- end_d 用比例 `tgt × k`（k∈[0, 0.85]），确保所有候选在 0 与 target 之间，杜绝端点超调
- end_s 用时间制 `speed × T`，使换道时间在任何速度下恒定于 7.5-8.6s

**NUDGE 动态 smin**:
```
safe_smin    = max(20.0, distance_to_obstacle - 10.0)
kinematic_smin = speed × 1.8
smin_nudge   = max(safe_smin, kinematic_smin)
```

#### 纵向终点采样

- **巡航**: 固定时间采样点 {0.01, 1, 2, 3, 4, 5, 6, 7, 8}s，每个时间点采样 v_lower, v_upper, 和中间 2-4 个速度点
- **跟车/超车**: ST 图障碍物边界采样点
- **停车**: 固定时间点 + 终点 s = max(s₀, stop_point)

### 4.4 代价函数

```
total_cost =   lon_objective_cost × 10.0    // 纵向目标速度偏差
             + lon_jerk_cost × 1.0           // 纵向加加速度
             + lon_collision_cost × 5.0      // 纵向碰撞风险
             + centripetal_acc_cost × 1.0    // 向心加速度
             + lat_offset_cost × w_lat_offset   // 横向偏离目标
             + lat_comfort_cost × w_lat_comfort // 横向舒适度
```

#### 横向偏移代价 LatOffsetCost（核心）

```
cost = (1/N) × Σᵢ wᵢ × sqᵢ

devᵢ = dᵢ - target_lat_offset
sqᵢ  = (devᵢ / bound)²

if devᵢ × target > 0:  sqᵢ ×= ALC_overshoot_penalty (default 100)  ← YAML 可配

wᵢ = w_opposite_side   if devᵢ × dev_start < 0  (跨目标线)
   = w_same_side        otherwise
```

#### 模式特定权重

| 权重参数 | LCC | ALC | NUDGE | 含义 |
|----------|-----|-----|-------|------|
| w_lat_offset | 8.0 | 10.0 | 2.0 | 横向偏移惩罚 |
| w_opposite_side | 8.0 | 1.0 | 1.0 | 跨目标线惩罚 |
| w_same_side | 1.0 | 1.0 | 1.0 | 同侧惩罚 |
| w_lat_comfort | 2.0 | 1.0 | 1.0 | 舒适度惩罚 |
| w_lat_offset_bound | 0.3 | 2.0 | 3.0 | 归一化边界 |
| **超调惩罚** | — | **×100** | — | d 超出 target 时触发 |

### 4.5 ST 图与障碍物处理

`PathTimeGraph` 将障碍物投影到 (s, t) 平面：
- 每个障碍物的预测轨迹 → 多边形 ST 区域
- `GetObstacleSurroundingPoints()` 提取 ST 上下边界
- 上边界用于**跟车采样**（在障碍物后方 s-lower 取点）
- 下边界用于**超车采样**（在障碍物前方 s-upper 取点）

### 4.6 碰撞检测

`CollisionChecker` 对合成后的 2D 轨迹逐点检查：
- 自车 bounding box (3.0m × 1.6m) 与每个障碍物的 polygon 做 AABB 判定
- 缓冲区: 纵向 2.0m, 横向 0.3m

### 4.7 约束校验

`ConstraintChecker` 检查每条轨迹的运动学可行性：
- 速度: [-0.1, 40] m/s
- 纵向加速度: [-4.0, 2.0] m/s²
- 纵向加加速度: [-4.0, 4.0] m/s³
- 横向加速度: ≤ 10.0 m/s²
- 曲率: ≤ 2.0 m⁻¹

---

## 5. 参数汇总

**所有参数均可通过 `config/configs.yaml` 修改，修改后无需重编译，直接 `ros2 launch` 生效。**

### 5.1 基础参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FLAGS_trajectory_time_length | 8.0 s | 规划总时域 |
| FLAGS_trajectory_time_resolution | 0.1 s | 采样步长 |
| FLAGS_num_velocity_sample | 6 | 每个时间点的速度采样数 |
| FLAGS_min_velocity_sample_gap | 1.0 m/s | 速度采样最小间隔 |
| FLAGS_longitudinal_acceleration_lower_bound | -4.0 m/s² | 最大减速度 |
| FLAGS_longitudinal_acceleration_upper_bound | 2.0 m/s² | 最大加速度 |
| FLAGS_longitudinal_jerk_lower_bound | -4.0 m/s³ | 最大负加加速度 |
| FLAGS_longitudinal_jerk_upper_bound | 4.0 m/s³ | 最大正加加速度 |
| FLAGS_lateral_acceleration_bound | 10.0 m/s² | 横向加速度上限 |
| FLAGS_kappa_bound | 2.0 | 曲率上限 |
| FLAGS_speed_lon_decision_horizon | 250 m | 纵向评价视界 |
| FLAGS_default_lon_buffer | 5.0 m | 纵向缓冲（跟车/超车） |
| FLAGS_num_sample_follow_per_timestamp | 3 | 每时间戳跟车采样数 |

### 5.2 横向采样参数（LCC / ALC / NUDGE）

| 参数 | LCC | ALC | NUDGE | 说明 |
|------|-----|-----|-------|------|
| `*_end_d_candidates` / `*_end_d_fractions` | `[-0.3..0.3]` 7 值 | `[1.0, 0.85, 0.6, 0.0]` 比例制 | `[-2.5..2.5]` 11 值 | end_d 候选集 |
| `LCC_smin_multiplier` / `ALC_end_s_times` / NUDGE 动态 | `speed×4` | `speed×{7.5,7.8,8.2,8.6}` 时间制 | `max(safe, kin)` 动态 | smin / end_s 计算方式 |
| `*_end_s_deltas` / `*_end_s_times` | `[5,10,15,20,30]` 5 值 | `[7.5,7.8,8.2,8.6]` 4 值 | `[0,10,15,20,30]` 5 值 | end_s 候选生成 |
| `NUDGE_kinematic_smin_multiplier` | — | — | `1.8` | 运动学 smin 系数 |
| `NUDGE_safe_smin_min` | — | — | `20.0` m | 安全距离下限 |
| `NUDGE_safe_smin_offset` | — | — | `-10.0` m | 障碍物前方偏移 |
| **候选总数** | **35** | **16** | **55** | — |

### 5.3 评价权重（LCC / ALC / NUDGE）

| 参数 | LCC | ALC | NUDGE | 含义 |
|------|-----|-----|-------|------|
| `*_w_lat_offset` | 8.0 | 10.0 | 2.0 | 横向偏移惩罚 |
| `*_w_opposite_side` | 8.0 | 1.0 | 1.0 | 跨目标线惩罚 |
| `*_w_same_side` | 1.0 | 1.0 | 1.0 | 同侧惩罚 |
| `*_w_lat_comfort` | 2.0 | 1.0 | 1.0 | 舒适度惩罚 |
| `*_w_lat_offset_bound` | 0.3 | 2.0 | 3.0 | 归一化边界 |
| `ALC_overshoot_penalty` | — | **100.0** | — | 超调惩罚倍率 |
| `preferred_lat_sign_tiebreaker` | 3.0 | 3.0 | 3.0 | 横向偏好 tiebreaker |

> **所有参数均在 `config/configs.yaml` 中定义，修改后无需重编译。**

---

## 6. 驾驶模式

| 模式 | 枚举值 | 场景 | end_d 范围 | 代价特点 |
|------|--------|------|-----------|----------|
| **LCC** | Lane Centering Control | 车道保持 | [-0.3, 0.3] | 强居中 (w_lat_offset=8, bound=0.3) |
| **ALC** | Auto Lane Change | 自主变道 | `{tgt, tgt×0.85, tgt×0.6, 0}` | 强拉到 target (w_lat_offset=10, bound=2) + 超调惩罚 ×100 |
| **NUDGE** | Obstacle Nudge | 障碍物绕行 | [-2.5, 2.5] | 对称权重，宽边界 (bound=3) |

### ALC 模式设计要点

1. **比例制 end_d**: `{tgt, tgt×0.85, tgt×0.6, 0}` — 所有候选在 0 与 target 之间，任意偏距值均无端点超调
2. **时间制 end_s**: `speed × {7.5, 7.8, 8.2, 8.6}s` — 任意速度下换道时间恒定
3. **超调惩罚 ×100**: `deviation × target > 0` 时触发，五次多项式中间振荡被压制在 ≤0.05m
4. **单点修改**: 只需改 `set_target_lat_offset(X)`，所有候选自动按比例适配

---

## 7. 仿真节点 — 数据流

```
lattice_test_node.cpp (timer_callback, 100ms)
    │
    ├─ 更新自车状态 (x, y, theta, v)
    ├─ 构建障碍物列表 (Obstacle 对象)
    ├─ 设置 PlanningTarget (巡航速度, 目标偏移)
    │
    ├─ LatticePlan()  ← 核心调用
    │   ├─ [输入] InitialConditions: x, y, v, a, theta, kappa
    │   ├─ [输入] PlanningTarget: cruise_speed, target_lat_offset
    │   ├─ [输入] obstacles, accumulated_s, reference_points
    │   ├─ [输入] driving_mode, current_speed, distance_to_obstacle
    │   ├─ [输出] DiscretizedTrajectory (最优轨迹)
    │   └─ [输出] all_trajectories (所有候选, 用于 RViz 可视化)
    │
    ├─ 提取 trajectory[1] → 更新 vehicle_state_
    ├─ 发布 RViz MarkerArray (参考线/最优/候选/障碍物/自车)
    └─ 写入 CSV 日志 (/tmp/lattice_trajectory_log_*.csv)
```

---

## 8. 关键修改记录

相对于原始 Apollo Lattice 的主要改动：

1. **全参数 YAML 化**: 所有模式特定参数（采样分布、评价权重、超调惩罚、tiebreaker）迁移到 `configs.yaml`，修改参数无需重编译
2. **时间制 end_s (ALC)**: 从距离制改为 `speed × {7.5,7.8,8.2,8.6}s`（YAML 可配），任意速度下换道时间恒定
3. **比例制 end_d (ALC)**: 从固定数组改为 `tgt × {1.0,0.85,0.6,0.0}`（YAML 可配），任意偏距自动适配且无端点超调
4. **超调惩罚**: LatOffsetCost 中新增 `ALC_overshoot_penalty`（默认 ×100，YAML 可配），压制五次多项式中间振荡 ≤0.05m
5. **模式特定权重**: 代价权重按 LCC/ALC/NUDGE 分别在 YAML 中设置，各自独立调优
6. **target_lat_offset 贯穿**: 从 PlanningTarget → LatticePlan → EndConditionSampler → TrajectoryEvaluator 全链路
7. **条件化状态继承**: 仅在 NUDGE 模式下继承上一周期 d0，其他模式自然收敛
8. **NUDGE 动态 smin**: 基于障碍物实际距离计算，参数在 YAML 中可配
9. **约束放宽**: 速度/加速度/加加速度约束从城市道路值放宽到高速公路值 (22 m/s)

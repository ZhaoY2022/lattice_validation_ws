float32 time        # [s] 相对于规划局部时间的相对时间
float32 length      # [m] 轨迹点的相对长度
float32 x           # [m] 轨迹点位置X 【GROUND系】
float32 y           # [m] 轨迹点位置Y 【GROUND系】
float32 vx          # [m/s] 轨迹点速度Vx 【GROUND系】
float32 vy          # [m/s] 轨迹点速度Vy 【GROUND系】
float32 acc_x       # [m/s^2] 轨迹点加速度Ax 【GROUND系】
float32 acc_y       # [m/s^2] 轨迹点加速度Ay 【GROUND系】
float32 heading     # [rad] 轨迹点的航向角 【GROUND系】
float32 curvature   # [m^-1] 轨迹点的曲率 【GROUND系】
float32 pinch       # [m^-2] 曲率变化率 pinch=d(curvature)/ds 【GROUND系】
float32 jerk        # [m/s^3] 加速度变化率 jerk=d(acc)/dt 【GROUND系】
float32 reserved    # 预留

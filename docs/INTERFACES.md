# 公共接口与坐标约定

动态心理扩展：`/multirobot/hunav/compute_agents_v2`使用
`arena_multi_hunav_msgs/ComputeMultiAgentsV2`，沿用v1共同快照、epoch/step/dt契约，
新增恰好覆盖所有行人的BehaviorIntent和响应力分项MotionDiagnostic。
意图只覆盖目标驱动力；目标ID不授权核心重新查询真值生成心理方向。
REGULAR/CURIOUS/SCARED枚举为0/1/2，`Agent.behavior.type`继续保持兼容值1。
心理话题schema_version=2在事务提交后发布，noop仍为版本1。
详见 [DYNAMIC_PSYCHOLOGY.md](DYNAMIC_PSYCHOLOGY.md)。

平台只有 `/clock` 和 `/map` 是跨机器人共享数据。`/tf`、`/tf_static` 使用共享总线，
但所有机器人 frame 都带 `robot_i/` 前缀。其余控制、状态和 action 都在机器人命名空间内。

| 名称 | 类型 | 发布/服务方 | 约束 |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Isaac | 必须只有一个发布者 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map server | 30 × 23 m 场景的公共静态地图 |
| `/robot_i/odom` | `nav_msgs/msg/Odometry` | Isaac | 真值里程计，frame 为 `robot_i/odom`、`robot_i/base_link` |
| `/robot_i/lidar_normalized` | `sensor_msgs/msg/LaserScan` | Isaac 规范雷达 | REP-117 语义，10 Hz 仿真时间 |
| `/robot_i/joint_states` | `sensor_msgs/msg/JointState` | Isaac | 逐机器人关节状态 |
| `/robot_i/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 逐机 Nav2 | 目标 frame 为 `map` |
| `/robot_i/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | 逐机 Nav2 | 目标 frame 为 `map` |
| `/robot_i/cmd_vel_external` | `geometry_msgs/msg/Twist` | 外部算法 | 只在 `external` 模式接收 |
| `/robot_i/cmd_vel` | `geometry_msgs/msg/Twist` | 命令守护 | 最终底盘命令，必须只有一个发布者 |
| `/robot_i/emergency_stop` | `std_srvs/srv/SetBool` | 命令守护 | 锁存停车；解除后需要新命令 |
| `/multirobot/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 命令守护 | 逐机安全状态 |

内部 Nav2 链为 `controller_server -> cmd_vel_raw -> velocity_smoother -> cmd_vel_nav ->
command_guard -> cmd_vel`。恢复行为也写入 `cmd_vel_nav`。命令守护把输出限制在
`|v| <= 1.0 m/s`、`|w| <= 1.2 rad/s`，丢弃非平面分量，并按墙钟检查命令、雷达和
同伴 odometry 的接收间隔；雷达与同伴状态还按仿真时间戳检查。时钟回退会锁存停车，
需要重启本轮实验。

TF 固定为：

```text
map -> robot_i/odom -> robot_i/base_link -> robot_i/传感器和轮子
```

Isaac 发布 `map -> robot_i/odom` 恒等变换和 `robot_i/odom -> robot_i/base_link` 真值变换；
出生位置已经进入真值位姿。逐机 `robot_state_publisher` 只发布机器人内部变换。

`arena_peer_costmap/PeerObstacleLayer` 只进入 local costmap。它使用其他机器人的真值
odometry，按 `0.48 × 0.44 m` 旋转矩形标记占据，排除自身，每轮清除旧位置。同伴状态
过期时保留最后占据并把 costmap 标为非 current；命令守护同时停车。该接口明确属于
理想感知，不提供优先级、轨迹预测或协调决策。
# 动态心理 V2 接口增量

V2模型使用 `/multirobot/hunav/compute_agents_v3`（ComputeMultiAgentsV3）。
BehaviorIntentV3保留0/1/2，新增SURPRISED=3、THREATENING=4；Agent.behavior.type仍为1。
每个行人选择DYNAMIC_SOCIAL=0或STATIC_OBSTACLE=1，附带可选desired_yaw、max_yaw_rate、
yaw_stop_speed。核心只在积分前后均停稳时执行限速身体转向，不直接改写位置或目标队列。
MotionDiagnosticV3分开记录robot_social_force、robot_static_force、robot_near_force和实际yaw。
旧v1/v2服务与消息保持不变。schema3心理日志增加profile、surprise、phase、事件与预算状态，
以及input_stamp_ns/dt，避免把输入机器人观测与积分后人位置的时间混为一谈。
参数/数据约定见 [PSYCHOLOGY_V2_DATA.md](PSYCHOLOGY_V2_DATA.md)。


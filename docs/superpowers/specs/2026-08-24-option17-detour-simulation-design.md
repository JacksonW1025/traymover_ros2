# 选项17动态障碍物绕行仿真设计

## 状态

设计已由用户确认，当前文档用于实现前的范围冻结。第一阶段只实现仿真原型，不迁移完整真机工作流，也不要求编译整个工作区。

## 目标

在 `detour` 分支的 `scripts/traymover.sh` 中新增第 17 个启动选项，启动一个可重复的视频演示场景：Traymover 仿真底盘导航时遇到静态地图之外的动态箱体，先由安全层停车；如果安全停车持续约 8 秒，才让全局代价地图接收该障碍物并由 Nav2 真正重新规划，从箱体旁边绕行到目标点。

验收必须能够观察到全局路径从当前位置到目标位置发生改变，而不是预设一个固定的左移、右移或侧向动作。

## 非目标

第一阶段不包含：

- 真机串口、STM32 EStop 反馈或真实底盘里程计；
- FAST-LIO、PCD、NDT 或真实建图定位；
- RealSense 深度相机仿真；
- 行人识别、障碍物跟踪、预测或多目标最优避障；
- 自定义 Nav2 全局规划器；
- 整个仓库的完整构建和 Humble 到 Jazzy 的全面迁移。

仿真使用 Gazebo Sim、ROS 2 Jazzy 的 `ros_gz_sim`/`ros_gz_bridge`，真实硬件 URDF 保持可用。

## 已确认的行为决策

1. 绕行必须由 Nav2 全局规划器产生新路径。
2. 计时从“导航仍有运动意图、最终速度被安全层压为零、且前方存在障碍物”开始。
3. 障碍物在 8 秒内消失时，计时复位，不污染全局地图，机器人沿原逻辑继续。
4. 只依赖仿真 LiDAR 即可完成第一版；深度相机不是演示前置条件。
5. 场景包含静态地图内的基础墙体/障碍物，以及静态地图外的动态箱体。
6. 第 17 项一键启动 Gazebo、仿真机器人、传感器、Nav2 和 RViz，并询问是否启用绕行。
7. 仿真使用独立模型和仿真底盘，不启动真实串口。
8. 第 17 项默认使用固定起点和目标以保证视频可复现，同时保留 RViz 点目标调试入口。

## 总体架构

```text
Gazebo Sim
  ├─ 差速驱动：消费最终 /cmd_vel，发布 /odom 和 odom→base_link TF
  └─ LiDAR：发布 /scan
        ├─ local_costmap obstacle_layer
        ├─ collision_monitor：/cmd_vel_nav → /cmd_vel
        └─ detour_supervisor
              └─ 仅在安全停车持续约 8 秒后发布 /scan_global
                    └─ global_costmap obstacle_layer
                          └─ Nav2 周期性全局规划
                                └─ /plan 改变 → controller → /cmd_vel_nav
```

局部代价地图和 `collision_monitor` 始终直接使用 `/scan`，因此动态箱体出现后可以立即停车。全局代价地图只观察 `detour_supervisor` 输出的 `/scan_global`；在等待阶段没有动态箱体输入，从而强制实现“先停车、后绕行”。

现有恢复行为树中的周期性 `ComputePathToPose` 保留使用，规划器仍为现有 GridBased/NavFn 配置。动态障碍物进入全局代价地图后，下一次规划周期将重新生成路径。

## `detour_supervisor` 设计

新增一个小型 Python ROS 2 节点。它不发布速度，也不调用导航动作，只负责安全停车状态判断和激光数据门控。

### 输入

- `/scan` (`sensor_msgs/msg/LaserScan`)：前方障碍检测和门控数据源；
- `/cmd_vel_nav` (`geometry_msgs/msg/Twist`)：Nav2 的运动意图；
- `/cmd_vel` (`geometry_msgs/msg/Twist`)：经过 `collision_monitor` 后的最终速度；
- 可选 `/traymover_estop/state` (`std_msgs/msg/Bool`)：已有仿真/安全状态时作为额外 OR 条件，不强制依赖深度相机。

### 输出

- `/scan_global` (`sensor_msgs/msg/LaserScan`)：仅在绕行阶段及清理窗口转发；
- `/traymover_detour/state` (`std_msgs/msg/String`)：`NORMAL`、`STOP_WAITING`、`DETOUR_ACTIVE`、`CLEARING`；
- `/traymover_detour/active` (`std_msgs/msg/Bool`)：便于 RViz/命令行观察。

### 状态规则

- `NORMAL`：默认不发布 `/scan_global`。只有导航速度非零、最终速度接近零、且前方 LiDAR 障碍满足距离/角度阈值时进入 `STOP_WAITING`。
- `STOP_WAITING`：使用 ROS 仿真时间计时。障碍物消失、最终速度恢复或导航意图消失时回到 `NORMAL`。持续时间达到 `hold_time_sec` 且障碍物仍在时进入 `DETOUR_ACTIVE`。
- `DETOUR_ACTIVE`：按原始扫描频率转发 `/scan_global`，使全局代价地图标记和清除动态障碍物；不干涉 `/cmd_vel`。观察到障碍清除后进入 `CLEARING`。
- `CLEARING`：继续转发一个短清理窗口，使全局代价地图通过 raytracing 清除旧标记，随后回到 `NORMAL`。

默认参数为 `hold_time_sec=8.0`、前方停止距离约 `0.8--0.9 m`，并允许 launch 参数覆盖。`enable_detour=false` 时节点不转发 `/scan_global`，其余停车链路不变。

安全边界：节点没有消息或自身退出时，最坏结果是全局障碍物不被标记；`collision_monitor` 仍然负责局部停车，不允许该节点绕过安全层直接驱动机器人。

## 仿真模型和场景

新增 `traymover_robot_description/urdf/traymover_sim.urdf.xacro`，沿用当前底盘几何尺寸、`base_link`、轮子和 `laser` frame，同时补充 Gazebo Sim 所需的差速驱动和 LiDAR 插件。硬件用的 `traymover.urdf.xacro` 不直接加入仿真插件。

新增 `traymover_robot_sim`（`ament_python`）包，包含：

- `traymover_detour_sim.launch.py`；
- Gazebo 世界文件；
- 动态箱体模型；
- 2D 静态演示地图；
- 仿真专用 Nav2/collision monitor 参数；
- RViz 配置；
- `detour_supervisor` 和固定目标发送器。

世界中包含矩形房间/走廊、基础墙体和若干静态障碍物。动态箱体不写入静态地图，可通过 launch 参数在导航启动后延迟生成，以模拟运行中的障碍物出现。场景几何会保留至少一条足够宽的旁路，并通过不对称的静态几何让规划器自然选择可行方向。

定位使用 AMCL + 静态地图；不启动 FAST-LIO、NDT、PCD 或串口底盘。Gazebo 差速驱动直接消费最终 `/cmd_vel`，桥接 `/scan`、`/odom`、`/clock` 和所需 TF。

## Nav2 配置边界

仿真参数文件与选项16分开维护：

- local costmap 继续观察 `/scan`；
- global costmap 增加 `obstacle_layer`，只观察 `/scan_global`；
- 保留静态层和 inflation layer；
- 继续使用现有周期性规划行为树；
- 保留选项16的 collision monitor 多边形和最终 `/cmd_vel` 安全链路；
- `use_sim_time=true`。

当 `enable_detour=false` 时，全局动态障碍层没有输入，因此同一场景表现为遇障停车、不绕行。

## 启动脚本行为

`scripts/traymover.sh` 改动范围仅限菜单和新增函数：

- 菜单加入 `17) Simulation: Dynamic obstacle stop and detour`；
- 第 17 项询问是否启用 8 秒后绕行，默认启用；
- 可提供 RViz 启动选项；
- 直接启动 `traymover_robot_sim` 的仿真 launch；
- 不要求 FAST-LIO PCD，不检查真实串口，不启动真实 LiDAR/RealSense；
- 现有 1--16 项行为不改变。

动态箱体生成、延迟时间、自动发送目标等调试选项通过 launch 参数提供，不增加不必要的菜单分支。

## 测试和验收

实现后先执行最小范围验证：

- `bash -n scripts/traymover.sh`；
- Python 源文件语法检查；
- xacro 展开检查；
- launch 参数解析/`--show-args` 检查；
- `detour_supervisor` 状态机单元测试；
- 仿真相关包的定向构建，不执行全仓库构建。

视频验收分为两条路径：

1. `enable_detour=true`：箱体出现后停车约 8 秒，全局 `/plan` 改变，机器人绕开箱体到达目标。
2. `enable_detour=false`：箱体出现后停车，路径不改变，机器人保持安全停车。

额外验证：在 8 秒内移除箱体，机器人不进入绕行状态并恢复原路径。

## 已知限制和风险

- Thor 主机是 ROS 2 Jazzy，而仓库原始开发环境是 Humble；实现按本机 Gazebo Sim/ROS Jazzy 接口编写，并在缺少依赖时给出明确诊断。
- Gazebo Sim 插件和 `ros_gz_bridge` 的具体消息桥接名称需要在本机做最小运行验证。
- 当前硬件 EStop 的串口反馈不在选项17仿真范围内；仿真安全停车由 collision monitor 和 supervisor 的等价状态完成。
- 全局障碍层只在持续停车后接收扫描，适合演示“等待后绕行”，不是通用动态障碍物跟踪算法。


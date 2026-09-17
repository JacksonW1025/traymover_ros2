# NDT 源码摘录与干扰敏感性补充

本文档补充《Traymover 干扰因素敏感的定位算法技术报告》的实机源码核对结果。核对对象为 Jetson 工作区当前实现，未修改 `lidar_localization_ros2`、FAST-LIO 或 `ndt_omp_ros2` 核心源码。

## 1. 前置核对

- 核心包存在：`src/traymover_robot_slam/lidar_localization_ros2`、`src/traymover_robot_slam/FAST_LIO`、`src/traymover_robot_slam/ndt_omp_ros2`。
- 回归测试：`colcon test --packages-select traymover_robot_nav --pytest-args -k test_initialpose_is_only_ndt_guess_until_valid_alignment --event-handlers console_direct+` 通过；实际 CTest 运行该包 4 个 pytest 入口，`test_nav_simple_config.py` 19 项通过。
- 汇总测试：`colcon test-result --verbose` 输出 `Summary: 42 tests, 0 errors, 0 failures, 0 skipped`。
- 现场 ROS 日志未包含当前实现的 `NDT aligned:` fitness 统计行；旧日志中仍可见早期 `Seeded map→odom from initialpose` 行，因此未作为 A/B/C/D/E 定位实验数据使用。

## 2. 源码摘录

### 2.1 参数入口与干扰门限

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:98-137`

摘录要点：节点声明 `score_threshold`、`ndt_resolution`、`voxel_leaf_size`、`scan_min/max_range`、`far_point_boost_*`、`use_odom`、`max_pose_jump_*`、`ndt_align_interval_s`、`map_odom_smoothing` 与 `max_map_odom_update_*`。

这些参数把干扰响应从 NDT 核心源码外提到 ROS 参数层。地图鬼影、远距墙体稀疏、U 型转弯、TF stale 与控制器冲击分别落到 score、点云裁剪/加权、jump gate、对齐节流和平滑门限上。

### 2.2 TF 高频重广播

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:233-240`、`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:265-273`

摘录要点：激活时创建 `tf_rebroadcast_timer_`；`rebroadcastMapOdomTF()` 只在 `have_good_pose_` 为真时把 `last_good_map_to_odom_` 以 `now()` 时间戳重新广播。

这降低了 Jetson 算力波动和 NDT 低频对齐造成的 TF 过期风险。它不接受未验证初值，因此不会把 `/initialpose` 或失败匹配扩散给 Nav2。

### 2.3 NDT_OMP 初始化

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:441-486`

摘录要点：`registration_method == "NDT_OMP"` 时创建 `pclomp::NormalDistributionsTransform`，设置 `ndt_step_size`、`ndt_resolution`、`transform_epsilon`、线程数和最大迭代次数；随后配置体素滤波 `voxel_leaf_size`。

NDT 分辨率决定收敛盆地和地图几何离散尺度。体素过细会增加 CPU 压力，过粗会抹掉柱、墙角等约束；因此它对开阔大厅和长通道的几何退化非常敏感。

### 2.4 `/initialpose` 仅作为 NDT 初始猜测

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:489-518`

摘录要点：收到 `/initialpose` 后设置 `initialpose_recieved_`，但同时清空 `have_good_pose_`、置 `ndt_bootstrap_pending_ = true`、重置 `last_align_stamp_s_ = -1.0e9`，并记录 “Initialpose accepted as NDT guess only; waiting for NDT alignment before publishing map->odom”。

这阻断了人为初值误差直接播种 `map→odom` 的路径。机场现场手点初值可能存在 0.5 m 级位置误差或 yaw 偏差，必须等待 NDT 通过收敛、fitness 与 jump gate 后才能成为全局 TF。

### 2.5 FAST-LIO odom 先验

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:549-600`

摘录要点：`use_odom_` 为假时直接返回；为真时用连续 `/odom` 位姿差 `body_delta` 推进当前 map-frame 估计，而不是依赖空 twist 字段。

这一段对应开阔场地“车动位姿不动”的失效模式。NDT 对初值收敛盆地敏感，FAST-LIO 先验让 init guess 随车移动，降低纯 NDT 在大位移和弱特征区域冻结的概率。

### 2.6 对齐节流与等待状态

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:647-699`

摘录要点：`cloudReceived()` 要求已收地图和初值；失败状态文案区分 “keeping last good TF” 与 “waiting for valid NDT before publishing map->odom”；只有 `ndt_align_interval_s_` 到期才进入 NDT re-align，并在 align 前更新 `last_align_stamp_s_`。

节流避免每帧 NDT 抢占 Jetson CPU，失败时继续广播最后一次好 TF。对动态遮挡、点云突刺和短时算力抖动，这比直接发布失败结果更稳。

### 2.7 点云体素滤波、距离裁剪与远点加权

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:729-752`

摘录要点：实时源点云先经 `voxel_grid_filter_`，再按 `scan_min_range_ < r < scan_max_range_` 保留；当 `r >= far_point_boost_min_range_` 时按 `far_point_boost_factor_` 复制远点。

这直接面对几何退化干扰。远墙和长边界对 yaw/横向约束关键，但远点稀疏；保守复制远点能提升约束权重，同时仍受最大距离和体素滤波限制，避免远距噪声主导配准。

### 2.8 收敛与 fitness 判定

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:774-797`

摘录要点：调用 `registration_->align(*output_cloud, init_guess)` 后读取 `hasConverged()` 与 `getFitnessScore()`；未收敛或 `fitness_score > score_threshold_` 都拒绝，并广播最后一次好 TF或继续等待有效 NDT。

地图鬼影、动态遮挡和初值错误通常表现为 fitness 升高或不收敛。该门限过松会接受错误地图锚点，过严会长期 stale TF；报告主文档中的 `score_threshold: 1.0` 取舍正对应这一敏感性。

### 2.9 pose jump gate

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:827-843`

摘录要点：已有好姿态且不在 bootstrap 时，计算 `init_guess.inverse() * final_transformation`，若平移超过 `max_pose_jump_translation_` 或 yaw 超过 `max_pose_jump_rotation_`，记录 “NDT jump ... keeping last good TF” 并拒绝。

重复墙面、相似立柱和 U 型急转都可能让 NDT 锁到邻近局部最优。jump gate 抑制明显误锁，但 yaw 门限太小也会拒绝急转后的合法校正，因此 U 型转弯需要单独实测。

### 2.10 map→odom 二级跳变门限

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:845-885`

摘录要点：先由 `map→base_link * (odom→base_link)^-1` 计算候选 `map→odom`；若启用 `max_map_odom_update_translation_` 或 `max_map_odom_update_rotation_`，再比较新旧 map→odom，超限时记录 “NDT map→odom jump ... keeping last good TF” 并拒绝。

这一层保护下游控制器和 Nav2 costmap，尤其适合三维局部规划或对 TF 阶跃更敏感的模式。它对地图污染导致的系统性误校正敏感，能挡住大跳但不能修复污染地图本身。

### 2.11 map→odom 平滑

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:888-907`

摘录要点：若已有好 TF，则使用 `map_odom_smoothing_` 对平移线性插值、旋转 SLERP；默认含义是新 NDT 结果只按权重逐步施加。

平滑降低 NDT 偶发小偏差对控制器的冲击。副作用是地图真实漂移校正会滞后，因此在动态遮挡频繁或高速转弯时需要与 jump gate 和 odom 先验共同调参。

### 2.12 接受结果、发布诊断扫描和日志

**位置**：`src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp:909-946`

摘录要点：只有通过所有门限后才写入 `last_good_map_to_odom_`、置 `have_good_pose_ = true`、清除 `ndt_bootstrap_pending_`、发布 `/pcl_pose` 与 `ndt_aligned_scan`，并输出 `NDT aligned: fitness=... map→odom=...`。

这说明 RViz 的 `ndt_aligned_scan` 是“已接受匹配”的诊断视图，不是所有尝试的可视化。若后续现场日志没有这类行，就无法计算真实 `fitness_median`、`fitness_p90` 和拒配比例。

## 3. 旧日志排除说明

`/home/wheeltec/.ros/log/lidar_localization_node_*.log` 中存在 2026-04-30 的旧运行记录，部分文件包含 “Seeded map→odom from initialpose; next cloud will refine with NDT”。该行为已被当前源码和测试替换为“initialpose 仅作 NDT guess”。旧日志没有当前 `NDT aligned:` 统计行，也没有按 A/B/C/D/E 标记的实验变量，因此不纳入 `localization_experiment_metrics.csv` 的实车数据。

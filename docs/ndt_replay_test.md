# NDT 定位日志与回放测试

测试工具迁自 `/home/wheeltec/traymover_ros2_paramchanged`。
`scripts/traymover.sh` 选项 10 已自动接入诊断日志。
源仓库的调参差异保存于
[`ndt_replay_candidates.yaml`](../src/traymover_robot_nav/config/ndt_replay_candidates.yaml)，
全文为注释，不能作为运行参数文件使用；待当前仓库实际运行日志分析后再决定是否覆盖。

2026-09-18 按用户要求，将 `config/localization.yaml` 的 `score_threshold`
从 `1.0` 调为 `40.0`，用于观察 `w_f_c.pcd` 的匹配结果；其他参数保持原值。
这是临时接纳门限实验，尚未验证定位质量。所有使用该共享配置的定位入口均受影响，
包括选项 10 和独立回放；已运行的节点需重新启动后加载。原始基线为 `1.0`。

## 地图降采样与重新初始化

选项 10 使用的 `config/localization.yaml` 已启用 `map_voxel_leaf_size: 0.15`。
定位节点加载地图后进行一次 15 cm 体素质心降采样，再检查 NDT 体素密度、
设置配准目标并发布 `/initial_map`。这样同时减少 NDT 目标搜索与 RViz 传输开销。
它不修改原始 PCD、地图坐标或二维地图生成流程；实时扫描仍用原来的
`voxel_leaf_size: 0.15`。将 `map_voxel_leaf_size` 设为 `0.0` 可禁用地图降采样。

本机对 `w_f_c.pcd` 的离线实测：

| 地图体素尺寸 | 点数 | 剩余点中满足 0.5 m NDT 体素至少 6 点条件的比例 |
| --- | ---: | ---: |
| 原始地图 | 22,697,715 | 98.2%（实车日志） |
| 0.15 m（采用） | 5,449,904 | 90.7% |
| 0.20 m（对照） | 3,329,042 | 77.5% |

采用 0.15 m 后点数减少约 76%，同时保留较多 NDT 协方差估计所需的点。
降采样本身约耗时 2.7 秒（本机离线单次测量）；这不是导航配准耗时或精度验证。
每次启动可在 `nav_stack.log` 中检查 `Map downsample:` 和 `NDT map density`。
其他地图仍会运行已有的自适应分辨率检查，不能假定同一降采样尺寸适用于所有地图。

选项 10 启动后，可重复使用 RViz **2D Pose Estimate** 设置机器人位置和朝向。
新的 `/initialpose` 会清除上一轮定位有效状态、odom 增量基线和已捕获的平面锁，
等待时间戳不早于该初始位姿的新扫描，不再重新使用旧的成功扫描。
重新匹配成功之前不会继续广播旧解；已有 TF 缓存仍可能短暂保留旧数据。
早于新初始位姿时间戳的排队里程计也会被忽略，避免将旧运动叠加到新位姿。

本次从源仓库迁入并启用：

- `max_bootstrap_pose_jump_translation: 2.5`：首次/重新初始化时相对新猜测的平移门限。
- `max_bootstrap_pose_jump_rotation: 0.60`：对应的三维旋转角门限，单位 rad。

新结果仍需通过收敛、fitness 和上述门限；成功后直接建立新的 `map→odom`，
不与旧定位结果混合，然后恢复原有的跟踪跳变检查与平滑。
重置请求若在 NDT 计算期间到达，要等当前单线程回调结束后处理。
这里只重置 NDT 全局定位，不会重置 FAST-LIO 内部里程计；若 FAST-LIO 本身已发散，
不能把重新 Init 当作已恢复整个定位链的证明。

已在隔离 ROS 域中验证安装后的定位节点：实际 `w_f_c.pcd` 加载/激活成功，
读取到 `map_voxel_leaf_size=0.15`、`score_threshold=40.0` 和 `2.5/0.60`
初始化门限，并收到连续两次 `/initialpose`。该验证没有接入硬件扫描或运动。
配准接受、拒绝、旧数据过滤和平面锁兼容性由生产回调的自动化回归测试覆盖；
降采样后的实车定位精度仍需下一轮运行日志确认。

## 选项 10：实车导航自动采集

正常运行 `./scripts/traymover.sh` 并选择 **10** 即可，无需另外启动回放脚本。
`run_nav_logged.sh` 在导航启动前启动 `ndt_diagnostics.py` 和
`fastlio_diagnostics.py`，监听 `/point_cloud_localization`、`/point_cloud_raw`、
`/imu/data_raw`、`/odom`、`/initialpose`、`/pcl_pose`、`/tf` 和 `/rosout`。
这是被动日志采集，不回放 bag，也不修改定位参数。

每次运行保存到 `logs/nav/<UTC时间>_option10.<唯一后缀>/`，菜单会打印完整路径。
可通过环境变量 `TRAYMOVER_NAV_LOG_ROOT` 修改根目录。主要文件：

- `ndt/`：NDT 事件、ROS 日志、CSV 指标。
- `fastlio/`：FAST-LIO 输入、里程计、TF 和消息间断指标。
- `processes/nav_stack.log`：导航终端完整 stdout/stderr，包含算法直接打印的信息。
- `processes/*diagnostics.log`：采集器自身报错。
- `parameters/`：启动后重试读取的实际参数；`.stderr` 保留读取失败原因。
- `run_info.txt`、`map.yaml`、`software_*.txt`：本次地图路径、投影参考和软件版本信息。
- `ros/`：本次启动的 ROS 日志。

诊断器使用实时时钟（`--use-sim-time false`），不依赖 `/clock`。
导航进程退出或关闭导航终端时会停止本次采集；菜单 **0** 也会清理采集进程。
菜单 `q` 仅退出菜单，和原有导航进程一样，采集可继续运行。
重新启动导航前先用 **0** 停止上一轮，避免重复导航节点混入日志。

## 独立回放测试

从工作区根目录运行，使用同一场地的 PCD 地图和含 `/point_cloud_raw`、
`/imu/data_raw` 的 ROS 2 bag：

```bash
# 仅定位：点云预处理 + FAST-LIO + NDT + RViz + 诊断采集
./scripts/test_nav_replay.sh --mode static /absolute/map.pcd /absolute/bag_dir

# 连续导航链测试：额外生成临时二维地图并启动 Nav2
./scripts/test_nav_replay.sh --mode continuous /absolute/map.pcd /absolute/bag_dir 0
```

需要已构建的 ROS 2 Humble 工作区和图形桌面；脚本自动加载 ROS 和本仓库
`install/setup.bash`。本次未迁移定位算法、FAST-LIO 算法或调参 launch。
`continuous` 采用当前菜单选项 10 的地面参考和 0.17–2.20 m 投影高度，
源仓库的 0.10 m 下限只留在候选注释中。

启动后不会立即播放 bag。在原终端输入 `once` 开始一次回放，收到点云后
在导航 RViz 中用 **2D Pose Estimate** 给定初始位姿。可用 `pause` 暂停，
设置初始位姿后 `resume`。`stop` 停止播放，`quit` 或 Ctrl+C 结束本次会话。
`play` 循环播放；`restart` 重启循环播放。循环和 restart 会使 bag 时间回退，
不会重置 FAST-LIO/NDT 内部状态；做参数对比应退出并重新启动整个会话，优先用 `once`。
`static` 指仅定位模式，不会自动截取静止片段；可传起始偏移秒数，再手动暂停。

脚本不启动底盘、雷达或 IMU 驱动。请在独立回放 ROS 图中运行：例如在没有其他节点的
`ROS_DOMAIN_ID` 下启动脚本；连续模式的 Nav2 仍可在收到导航目标后发布 `/cmd_vel`。
回放只播放点云、IMU 和 `/tf_static`，监视 RViz 的相机窗没有图像属于预期。

## 日志与后续参数判断

默认每次会话写入 `~/.ros/traymover_nav_replay/<UTC时间_PID_模式>/`。
可用 `TRAYMOVER_REPLAY_LOG_ROOT` 指定目录。

| 文件 | 用途 |
| --- | --- |
| `run_info.txt`、`bag_info.txt` | 地图、bag、模式、起始偏移和 bag 元信息 |
| `software_commit.txt`、`software_status.txt` | 软件版本及未提交变更状态 |
| `parameters/*.yaml`、`*.stderr` | 启动后实际节点参数快照；失败原因单独保存 |
| `ndt/ndt_events.log`、`ndt_metrics.csv` | 初始位姿、NDT 输出、TF、生命周期及消息年龄 |
| `ndt/ndt_rosout.log` | 定位节点 ROS 日志 |
| `fastlio/fastlio_events.log`、`fastlio_metrics.csv` | 点云/IMU/里程计接收速率和间断 |
| `processes/*.log`、`ros/`、`rosbag.log` | launch、算法 stdout/stderr 和播放器日志 |

诊断周期使用单调时钟，即使回放暂停或尚未产生 `/clock` 也继续记录。
`TRAYMOVER_NDT_LOG_INTERVAL_SEC`（默认 5）、
`TRAYMOVER_FASTLIO_LOG_INTERVAL_SEC`（默认 2）可调整采样周期。
`TRAYMOVER_REPLAY_EXTERNAL_CONTROL=1` 可恢复源脚本的独立控制终端。
两个诊断终端仅显示日志；没有终端模拟器时可直接读取上述文件。

分析顺序：先确认 LiDAR/IMU 连续性和 FAST-LIO 里程计，再核对 NDT fitness、
收敛/跳变拒绝原因及 `map→odom` 更新。诊断 `ACTIVE` 仅表示曾收到相应数据，
不能单独证明定位准确或当前 TF 新鲜；应结合消息年龄和轨迹检查。
不要仅因没有 NDT 输出就放宽评分阈值。

另外迁入两项只读工具，用于区分 bag 时间问题和在线传输问题：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/rosbag2_timing_audit.py /absolute/bag_dir --topics /point_cloud_raw /imu/data_raw
python3 scripts/ros2_raw_topic_rate.py /point_cloud_raw --qos sensor_data
```

源仓库针对其他运行环境的调试结论未作为本仓库已验证事实。当前迁移仅完成
无硬件检查和回归测试；实际地图/bag 回放、RViz 交互与定位质量仍需运行日志验证。

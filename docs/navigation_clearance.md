# 通道居中与转弯余量

## 适用范围

本次调整覆盖所有现有二维 Nav2 参数文件：菜单 10、15（两种加速配置）、
16、18，以及 `traymover_nav2` 的旧版二维导航。无需重新生成地图，
重启相应导航进程后加载参数。

菜单 14 使用 CMU 三维局部路径基元，没有本次调整的 Navfn 全局代价地图；
菜单 19 是独立的引导绕障测试，不使用 Nav2 全局路径。这两个实验模式
不具有本次加入的通道居中偏好。

## 修改与理由

| 参数 | 原值 | 新值 | 作用 |
| --- | --- | --- | --- |
| 全局/局部 `inflation_radius` | 0.35 m | 1.20 m | 扩大障碍物的软代价范围，让搜索提前远离墙边和内角 |
| 全局/局部 `cost_scaling_factor` | 3.0 | 2.0 | 让代价随距离下降更缓，在常见通道中保留朝中心递减的梯度 |
| RPP `lookahead_dist` | 1.40 m | 0.50 m | 未启用速度缩放时的前视距离 |
| RPP `min_lookahead_dist` | 1.10 m | 0.45 m | 低速时不再远远瞄准弯后的路径点 |
| RPP `max_lookahead_dist` | 2.60 m | 0.65 m | 加速模式仍限制提前切弯 |
| RPP `lookahead_time` | 2.80 s | 1.50 s | 缩短随速度增长的前视距离 |

旧版 MPPI 的 `ObstaclesCritic` 膨胀参数也与局部代价地图同步，避免
将栅格代价换算成错误的障碍物距离。

沿用 Navfn 的代价搜索，没有新增插件或依赖。RPP 仍在朝向误差超过
0.35 rad（约 20°）时停止平移并转向；短前视距离让控制器更贴近已规划
的通道转弯。大角度转向条件并非每个弯都一定触发，这也不保证生成严格
的“直行—原地转 90°—直行”折线路径。

1.20 m 是软代价影响范围，不是额外扩大 1.20 m 的车体，也不是禁行半径。
车体仍为 0.80 × 0.55 m。Navfn 按二维中心位置搜索，不保证矩形车体在
任何朝向都有足够转动空间；其半对角线约 0.49 m，窄通道尤其需要检查
实际转弯时的车角余量。宽于约 2.4 m 的大区域也未必存在唯一中心线。

## 验证与边界

配置回归覆盖所有二维模式的软代价梯度、窄通道非致命代价、前视距离
以及旧版 MPPI 的距离换算一致性。离线规划回归调用本机安装的 Navfn
算法，比较原参数与新参数在合成通道中的路径。

本机六套参数的离线规划回归共 24 项通过，结果如下（距离均为路径中心
到占用栅格边缘，不能直接当作车壳余量）：

| 合成场景 | 原参数 | 新参数 |
| --- | --- | --- |
| 左直角弯最小障碍距离 | 0.326 m | 0.779 m |
| 右直角弯最小障碍距离 | 0.359 m | 0.778 m |
| 直通道中段平均中心线偏差 | 0.300 m | 0.0031 m |

0.95 m 宽直通道仍能规划通过。测试使用真实 Navfn 搜索，栅格膨胀采用
Nav2 的距离代价公式；不启动机器人、传感器或 ROS 控制节点。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select traymover_robot_nav traymover_nav2
colcon test --packages-select traymover_robot_nav
colcon test-result --verbose
```

本次验证时，工作区原有定位测试仍要求 `score_threshold == 40.0`，而
用户当前 `localization.yaml` 为 `60.0`，该项失败与路径调参无关，保留
用户配置。新增配置回归 12 项及规划回归 24 项均通过。

离线测试不能替代带定位误差、传感器杂点和实际底盘响应的行驶验证。
短前视距离可能增加转向修正频率；速度配置、曲率减速和现有停障逻辑均
保留。软代价不会消除杂点，也不能修复 TF 超时或定位跳变。现有
`20260918T155413Z_option10.fVh66g` 日志中存在 `Unable to transform robot
pose into global plan's frame` 和 NDT 跳变，因此有路径但停驶时仍需区分
障碍停机与定位失效。

参考：[Nav2 官方调参指南：Inflation Potential Fields](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/)。
参数语义另与本机 `navigation2-humble` 的 RPP 和 MPPI 源码核对。

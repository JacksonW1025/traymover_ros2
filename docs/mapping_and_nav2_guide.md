# Traymover 建图与 Nav2 导航分工文档

本文档按两部分组织：

- `Codex 完成`：我已经替你做掉的基础检查与当前结论
- `人工完成`：必须由你在机器人现场完成的操作，包括键盘控制、地图保存、RViz 点位

目标是先完成 `gmapping` 建图，再用保存好的地图做一次 `Nav2` 导航测试。

## Codex 完成

这一部分是我已经在当前工作区里完成的基础检查。它们不依赖你手动操作机器人。

### 1. 工作区与包检查

我已经确认下面两个包能被当前环境正确找到：

- `turn_on_traymover_robot`
- `traymover_nav2`

对应安装目录分别是：

- `/home/wheeltec/traymover_ros2/install/turn_on_traymover_robot`
- `/home/wheeltec/traymover_ros2/install/traymover_nav2`

这说明当前工作区至少已经完成过一次构建，`source install/setup.bash` 后可以解析这两个包。

### 2. 启动文件检查

我已经验证以下两个启动文件可以正常解析参数：

- `ros2 launch turn_on_traymover_robot traymover_gmapping.launch.py --show-args`
- `ros2 launch traymover_nav2 traymover_nav2.launch.py --show-args`

这说明：

- `gmapping` 启动链路在当前安装环境里可被 ROS 2 正常识别
- `Nav2` 启动链路在当前安装环境里可被 ROS 2 正常识别

### 3. 建图前关键配置检查

我已经核对了 [src/turn_on_traymover_robot/config/traymover_robot.yaml](../src/turn_on_traymover_robot/config/traymover_robot.yaml)，当前关键参数是：

```yaml
usart_port_name: "/dev/ttyCH341USB0"
serial_baud_rate: 115200
odom_source_mode: "stm32_feedback"
```

这意味着当前源码配置符合本项目 `gmapping` 的前置要求：

- 底盘默认串口是 `/dev/ttyCH341USB0`
- 建图使用的里程计模式已经是 `stm32_feedback`

`gmapping` 在这个项目里明确依赖 `stm32_feedback` 和 `/odom_combined`，这一项目前是正确的。

### 4. 导航默认地图路径检查

我已经确认 `traymover_nav2.launch.py` 默认加载的地图路径是：

```text
/home/wheeltec/traymover_ros2/install/traymover_nav2/share/traymover_nav2/map/TRAYMOVER.yaml
```

这意味着你建图后，最好直接使用项目自带的保存命令：

```bash
ros2 launch traymover_nav2 save_map.launch.py
```

这样后面启动导航时不需要额外手动传 `map:=...`。

### 5. 当前地图文件状态检查

我已经检查过两个默认地图目录：

- `install/traymover_nav2/share/traymover_nav2/map`
- `src/traymover_robot_nav2/map`

当前都只有 `README.md`，还没有：

- `TRAYMOVER.yaml`
- `TRAYMOVER.pgm`

结论很明确：你现在还没有完成地图保存，所以暂时不能直接做默认路径的 Nav2 导航。

### 6. 这一轮我没有替你做的事

以下几件事必须你在机器人现场操作，我这边不能直接替代：

- 驾驶机器人走图
- 在现场观察地图质量
- 执行地图保存命令
- 在 RViz 中点击 `2D Pose Estimate`
- 在 RViz 中点击 `2D Goal Pose`

换句话说，本轮分工就是：

- 我负责把链路、配置、默认路径、启动入口先核对清楚
- 你负责真正和机器人交互的部分

## 人工完成

这一部分是你需要实际执行的步骤。建议严格按顺序来。

### 1. 每个终端先做环境准备

每开一个新终端，都先执行：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果你刚修改过代码但还没重新构建，再补一次：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. 建图阶段

这一阶段由你完成实际驾驶和现场观察。

#### 2.1 终端 1：启动建图主流程

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turn_on_traymover_robot traymover_gmapping.launch.py
```

这个启动会自动带起：

- 底盘驱动
- IMU 驱动
- 雷达驱动
- `pointcloud_to_laserscan`
- EKF
- `slam_gmapping`

#### 2.2 终端 2：打开 RViz 看建图效果

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/traymover_robot_description/rviz/traymover.rviz
```

你要重点看四件事：

- `Fixed Frame` 是不是 `map`
- 能不能看到机器人模型
- 能不能看到 `/scan`
- `/map` 是否随着机器人移动逐步长出来

#### 2.3 终端 3：你亲自键盘控制机器人走图

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run traymover_robot_keyboard traymover_keyboard
```

常用按键：

- `i`：前进
- `,`：后退
- `j`：左转
- `l`：右转
- `k` 或空格：停止

推荐走图方式：

1. 先在起点附近慢速转一圈
2. 沿房间或走廊边界慢速走一圈
3. 再补中间区域
4. 尽量回到走过的位置形成闭环

走图时尽量做到：

- 速度不要太快
- 转弯不要太急
- 避免剧烈加减速
- 尽量减少地面打滑

#### 2.4 什么时候算地图可以保存

你在 RViz 中看到下面这些现象后，再进入保存步骤：

- 地图轮廓已经比较完整
- 墙体没有明显双层重影
- 机器人位置和激光轮廓能对上地图
- 回到旧区域时地图没有明显撕裂

如果仍出现这些情况，先不要保存：

- 墙体重复成两层
- 机器人静止时地图持续漂
- 一转弯地图就扭曲
- 激光和墙体长期对不齐

### 3. 地图保存阶段

这个动作按你的要求由你来做。

#### 3.1 终端 4：你手动保存地图

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch traymover_nav2 save_map.launch.py
```

当前项目会把地图保存到两个位置：

- `install/traymover_nav2/share/traymover_nav2/map/TRAYMOVER.yaml`
- `src/traymover_robot_nav2/map/TRAYMOVER.yaml`

以及对应的图片文件，通常是 `TRAYMOVER.pgm`。

#### 3.2 保存后你自己确认文件是否生成

```bash
ls -l install/traymover_nav2/share/traymover_nav2/map/TRAYMOVER.*
ls -l src/traymover_robot_nav2/map/TRAYMOVER.*
```

你应该能看到至少：

- `TRAYMOVER.yaml`
- `TRAYMOVER.pgm`

### 4. 从建图切换到导航

地图保存完成后，先把建图阶段所有进程停掉：

- `traymover_gmapping.launch.py`
- 键盘遥控
- 建图 RViz

不要在建图还在运行时直接叠加启动导航。

### 5. 导航阶段

这一阶段由你负责启动后的人机交互动作，也就是你说的“导航点击”。

#### 5.1 终端 1：启动 Nav2

如果你刚刚用的是默认保存方式，直接执行：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch traymover_nav2 traymover_nav2.launch.py
```

如果你后面想换地图，再用：

```bash
ros2 launch traymover_nav2 traymover_nav2.launch.py \
    map:=/绝对路径/你的地图.yaml
```

#### 5.2 终端 2：打开导航 RViz

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/traymover_robot_nav2/rviz/traymover_nav2.rviz
```

#### 5.3 你手动完成 RViz 两次点击

第一步，点击：

```text
2D Pose Estimate
```

然后在地图上点机器人当前位置，并拖动箭头设置朝向。

第二步，点击：

```text
2D Goal Pose
```

然后在地图上点目标位置，并拖动箭头设置目标朝向。

这两步都由你来完成，我这边不替你操作。

#### 5.4 你如何判断导航是否启动成功

发目标后，正常现象通常是：

- 机器人先转向，再开始前进
- 能看到机器人沿路径移动
- 最后在目标点附近停下

如果机器人完全不动，先检查你自己刚才是否已经：

1. 先点了 `2D Pose Estimate`
2. 再点了 `2D Goal Pose`
3. 没把目标点打在墙里或障碍物里

### 6. 最简执行清单

如果你只想照着跑一遍，按这个顺序做就可以。

#### 6.1 你执行建图

终端 1：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turn_on_traymover_robot traymover_gmapping.launch.py
```

终端 2：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/traymover_robot_description/rviz/traymover.rviz
```

终端 3：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run traymover_robot_keyboard traymover_keyboard
```

你负责：

- 开车走图
- 看 RViz 地图质量
- 决定何时结束建图

#### 6.2 你执行地图保存

终端 4：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch traymover_nav2 save_map.launch.py
```

#### 6.3 你执行导航测试

先停掉建图阶段所有进程，再重新开：

终端 1：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch traymover_nav2 traymover_nav2.launch.py
```

终端 2：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/traymover_robot_nav2/rviz/traymover_nav2.rviz
```

然后由你在 RViz 中完成：

1. `2D Pose Estimate`
2. `2D Goal Pose`

## 什么时候再叫 Codex

如果你在人工操作阶段遇到问题，可以直接把现象告诉我，我继续接手排查。最有用的信息通常是：

- 哪一步出问题了
- 终端报错原文
- RViz 里你看到的现象
- 机器人是否有动作

最常见的几类问题包括：

- `gmapping` 起不来
- 地图不长出来
- 地图保存后没有生成 `TRAYMOVER.yaml`
- Nav2 启动后地图正常，但点目标机器人不动
- 机器人会动，但定位飘或者路径很怪

你只要把现象贴给我，我就继续帮你拆。当前这份文档已经按“我做基础检查，你做现场操作”的分工整理好了。

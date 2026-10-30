# nav_sim

面向动态障碍物导航的 ROS 2 / Gazebo Classic 地面机器人仿真平台。提供 AGV、激光雷达、RGB-D 相机和沿路径运动的障碍物，通过标准 ROS 2 接口接入导航栈。

仿真包本身可以独立启动；`scripts/start_all.sh` 是当前工作区中联动 `hanmole_navigation` / Nav2 的宿主机启动入口。

## 当前状态（2026-10-30）

| 项目 | 当前配置 |
|---|---|
| 使用环境 | ROS 2 Humble、Gazebo Classic 11；当前 Docker 镜像为 `nav-sim-full:humble`，容器为 `nav_sim_full` |
| AGV | `libplanar_drive_plugin.so` 平面全向驱动，接收机体坐标系 `geometry_msgs/Twist` 的 `vx / vy / wz`；四轮为固定关节 |
| 车体尺寸 | 长 × 宽 × 高为 `0.40 × 0.30 × 0.25 m`；轴距 `0.32 m`，轮距 `0.26 m`，轮半径 `0.10 m` |
| 里程计 | 来自 Gazebo 模型的实际位姿和速度；`/odom` 和 `odom -> base_footprint` TF 的目标发布频率为 `50 Hz`，底盘控制按物理步更新 |
| 停车保护 | `0.5 s` 单调时钟指令超时；NaN/Inf 指令立即清空目标速度；仿真重置清空指令并恢复发布时间 |
| 激光雷达 | `/scan`，配置频率 `10 Hz`，360 个采样点，量程 `0.12–30 m` |
| 相机 | RGB 与深度图配置为 `640 × 480 / 5 Hz`；已移除额外的 aligned-depth 相机 |
| 常用场景 | 联动脚本默认使用 `generated_env_ground_14.world`：40 个静态障碍物、14 个动态障碍物 |
| 导航联动默认值 | `robot_version=v0_3`、`nav_mode=wheel_odom`，使用 `config/v0_3/nav2_params_sim.yaml`；启用 boost、接近目标减速和 Humble Twist 适配器；intent 默认关闭 |
| 默认模型驱动 | `agv.urdf.xacro` 仅加载新平面驱动；旧 `libmecanum_drive_plugin.so` 保留构建供其他模型使用 |
| 显示 | 联动脚本默认打开 Gazebo GUI 与导航 RViz；`--headless` 关闭这两个窗口 |
| 进程管理 | 启动前清理旧进程；独立的 `scripts/stop_all.sh` 支持预览、正常退出和超时强制结束 |

默认 `v0_3` 使用完整速度链：`cmd_vel_nav`（Twist）→ 类型适配 → boost / mux → approach limiter → 类型适配 → velocity smoother → collision monitor → `/cmd_vel`（Twist）。显式选择 `v0_2` 会绕过 boost 和 approach limiter，仅加载相同 YAML 并不代表这些节点已启用。

### 底盘运动方式

旧模型通过普通圆柱轮碰撞体执行麦克纳姆轮速，实际横移与 Nav2 的 `Omni` 运动模型不符。默认 AGV 现将车轮固定为刚性底盘，保留车体、轮子和传感器的碰撞几何与质量，轮子及脚轮使用低摩擦系数。每个物理步开始时，将机体 `vx / vy` 按当前 yaw 转为世界速度，并设置角速度 `wz`；物理步结束时保留碰撞求解得到的 `x / y / yaw`，仅恢复出生高度、清零 roll/pitch 和相应速度。因此墙体仍可阻挡运动，odom 来自实际模型状态。

这是面向平坦地面导航的运动近似：轮子不独立转动，不模拟麦克纳姆滚子、轮胎牵引或打滑，也不适用于坡道、台阶和三维底盘动力学验证。速度指令应持续发布；超过 `0.5 s` 未收到有效指令时，下一物理步目标速度归零，仿真暂停期间的时间也计入超时。

## 环境与构建

独立运行需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11，以及 `xacro`、`robot_state_publisher`、`vision_msgs`、`cv_bridge`、TF2 和 Eigen。使用 RViz 时还需安装 `rviz2`。

在已安装 ROS 2 Humble 的环境中补齐依赖：

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-vision-msgs ros-humble-cv-bridge \
  ros-humble-tf2-geometry-msgs ros-humble-rviz2 libeigen3-dev
```

当前容器将宿主机的 `~/nav_sim_ws/src` 挂载到 `/workspace/src`。在容器内构建仿真包：

```bash
# 宿主机：进入已运行的容器
docker exec -it nav_sim_full bash

# 以下在容器内执行
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --symlink-install --packages-select nav_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

联动导航时，还需要同一工作区中的 `hanmole_navigation`、对应构建产物，以及：

- `/workspace/src/hanmole_navigation/maps/generated_env_ground_static.yaml`
- `/workspace/src/hanmole_navigation/maps/square_dynamic.yaml`（使用 `square_dynamic.world` 时）
- `/workspace/src/hanmole_navigation/config/v0_3/nav2_params_sim.yaml`
- `/workspace/src/hanmole_navigation/rviz/navigation.rviz`（打开导航 RViz 时）

## 快速使用：仿真与导航一起启动

本节所有脚本命令均在**宿主机、nav_sim 仓库根目录**执行。容器 `nav_sim_full` 需要事先处于运行状态。

```bash
cd ~/nav_sim_ws/src/nav_sim

# 默认：14 个动态障碍物、Gazebo GUI、导航 RViz，intent 关闭
./scripts/start_all.sh start

# 查看节点、话题和日志末尾
./scripts/start_all.sh status

# 关闭仿真与导航进程，保留容器运行
./scripts/stop_all.sh
```

常用启动方式：

```bash
# 先构建 nav_sim 和 hanmole_navigation，再启动
./scripts/start_all.sh start --build

# 关闭 Gazebo GUI 和 RViz
./scripts/start_all.sh start --headless

# 开启 intent 桥接与 IntentCostCritic / HeadingCritic
./scripts/start_all.sh start --intent

# 指定出生点和导航目标；yaw 单位为弧度
./scripts/start_all.sh start --spawn -11.0,-10.1 --goal 7.0,5.0,0.0

# 切换到方形场景；脚本自动匹配出生点和地图
./scripts/start_all.sh start --world square_dynamic.world

# 其他场景需要明确提供地图；--map 路径是容器内路径
./scripts/start_all.sh start --world corridor_dynamic_9.world \
  --spawn 0.0,0.0 --map /workspace/maps/corridor.yaml
```

启动顺序为：清理旧进程、刷新 Xauthority、启动 Gazebo、等待深度图话题、启动导航、等待 AMCL、关闭 AMCL 的 TF 广播并发布静态 `map -> odom`、按需打开 RViz、等待目标网关 ready，最后按需发送导航目标。

静态 `map -> odom` 为单位变换，依赖当前地图使用 Gazebo 世界坐标、仿真里程计直接输出世界位姿这一约定。

## 全部脚本与使用方法

仓库包含两个 Shell 脚本、一个 Python launch 入口和一个底盘实测脚本：

| 文件 | 执行位置 | 作用 | 基本用法 |
|---|---|---|---|
| [`scripts/start_all.sh`](scripts/start_all.sh) | 宿主机 | 一键启动 Gazebo + 导航，查询状态、发送初始目标；`stop` 与启动前清理均复用停止脚本 | `./scripts/start_all.sh start`、`./scripts/start_all.sh status`、`./scripts/start_all.sh stop` |
| [`scripts/stop_all.sh`](scripts/stop_all.sh) | 宿主机 | 关闭指定容器中的 Gazebo、RViz、ROS 2 命令/daemon、仿真、导航、定位、intent 节点及其子进程 | `./scripts/stop_all.sh`；预览：`./scripts/stop_all.sh --dry-run` |
| [`launch/sim.launch.py`](launch/sim.launch.py) | 已 source ROS 与工作区的环境，当前使用容器内环境 | 独立启动 Gazebo、AGV、robot_state_publisher，以及可选 RViz / ground-truth 节点 | `ros2 launch nav_sim sim.launch.py world:=square_dynamic.world` |
| [`test/planar_drive_test.py`](test/planar_drive_test.py) | 已构建并 source 工作区的 ROS/Gazebo 环境 | 启动独立空场景与墙体，实测底盘运动、碰撞、停车、无效指令和仿真重置，保存真值、odom、TF 和检查结果 | `ros2 run nav_sim planar_drive_test.py --output-dir /tmp/planar-test --disable-sensors` |

### start_all.sh

```bash
./scripts/start_all.sh --help
./scripts/start_all.sh start [选项]
./scripts/start_all.sh status
./scripts/start_all.sh stop
```

| 启动选项 | 作用 / 默认值 |
|---|---|
| `--world NAME` | `worlds/` 下的场景文件，可省略 `.world`；默认 `generated_env_ground_14.world` |
| `--spawn X,Y` | 出生点。generated 场景默认 `-11.0,-10.1`；`square_dynamic.world` 默认 `0.0,5.0` |
| `--map NAME\|PATH` | 地图 YAML；文件名相对 `hanmole_navigation/maps`，完整路径按容器内路径解释 |
| `--goal X,Y[,YAW]` | ready 后发送一次 `NavigateToPose` 目标；yaw 默认 `0.0`，单位为弧度 |
| `--robot-version V` | `v0_1 / v0_2 / v0_3`，默认 `v0_3`；`v0_2` 不启用 boost / approach limiter |
| `--nav-mode M` | `wheel_odom / ekf_odom`，默认 `wheel_odom` |
| `--gui`、`--rviz`、`--desktop` | 开启相应窗口；`--desktop` 同时开启两者，为默认状态 |
| `--headless` | 同时关闭 Gazebo GUI 和 RViz；相机仍由 gzserver 渲染 |
| `--intent`、`--no-intent` | 开启 / 关闭 intent；默认关闭 |
| `--timeout SEC` | 每个就绪阶段的等待次数，默认 150；每轮还包含 ROS 查询耗时，因此不是严格的总时限 |
| `--no-kill` | 跳过启动前的进程清理 |
| `--build` | 启动前在容器内构建 `hanmole_navigation` 与 `nav_sim` |

环境变量：

```bash
# 覆盖容器和 ROS domain；默认分别是 nav_sim_full、42
CONTAINER=nav_sim_full NAV_SIM_DOMAIN=42 ./scripts/start_all.sh start
```

联动脚本固定使用 `GAZEBO_MASTER_URI=http://127.0.0.1:11355`。与其他 Gazebo 实例并行运行时，各实例应使用不同的 master 端口，避免 GUI 连接到旧世界。`NAV_SIM_DOMAIN` 会显式设置容器中启动进程的 `ROS_DOMAIN_ID`。

### stop_all.sh

```bash
# 先查看将被关闭的进程，不发送停止信号
./scripts/stop_all.sh --dry-run

# 关闭当前仿真与导航栈
./scripts/stop_all.sh

# 指定容器，给正常退出留出 10 秒
./scripts/stop_all.sh --container nav_sim_full --timeout 10

# 与启动脚本使用同一个 CONTAINER 环境变量
CONTAINER=nav_sim_full ./scripts/stop_all.sh

# 等效停止入口
./scripts/start_all.sh stop
```

脚本先发送 `SIGTERM`，默认等待最多 5 秒，再对残留和重启的相关进程发送 `SIGKILL`。匹配包括 ROS 2 launch/run/监控命令、ROS daemon、Gazebo、RViz、Nav2 组合/独立节点、robot_state_publisher、静态 TF、ground-truth、定位，以及 `hanmole_navigation` 的网关、状态、目标目录、速度链和 intent 节点，并清理其子进程。已经停止的容器或没有匹配进程时正常退出。

停止范围是所选容器内的相关进程；容器本身、宿主机进程和其他容器继续运行。该操作会停止该容器中的所有 ROS 2 命令及匹配节点，不按 ROS domain 进一步区分。

### sim.launch.py：仅启动仿真

在容器内执行：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=42
export GAZEBO_MASTER_URI=http://127.0.0.1:11355

# 与联动脚本使用同一个场景和出生点，但单独启动仿真
ros2 launch nav_sim sim.launch.py \
  world:=generated_env_ground_14.world \
  spawn_x:=-11.0 spawn_y:=-10.1 gui:=true rviz:=true

# 只关闭 Gazebo GUI；RViz 默认关闭
ros2 launch nav_sim sim.launch.py gui:=false

# 查看完整参数
ros2 launch nav_sim sim.launch.py --show-args
```

| launch 参数 | 默认值 | 作用 |
|---|---|---|
| `world` | `square_dynamic.world` | `worlds/` 下的场景文件名；与联动脚本的默认场景不同 |
| `spawn_x` / `spawn_y` | `0.0` / `0.0` | AGV 出生坐标 |
| `gui` | `true` | 是否启动 gzclient |
| `rviz` | `false` | 是否打开 `nav_sim/rviz/nav_sim.rviz` |
| `use_sim_time` | `true` | robot_state_publisher 使用仿真时间 |
| `publish_ground_truth` | `false` | 是否启动 ground_truth_publisher；需要另行提供 `/model_states` |

该入口只启动仿真相关节点。相机即使在 `gui:=false` 时也需要可用的渲染环境；当前容器使用宿主机 X11/Xauthority。

### planar_drive_test.py：实测底盘

先构建并 source 工作区，再选择空闲的 ROS domain 和 Gazebo master 端口。该脚本自行启动和清理测试 Gazebo，勿与同 domain 的其他速度发布者同时使用。输出目录必须为空：

```bash
export ROS_DOMAIN_ID=84
export GAZEBO_MASTER_URI=http://127.0.0.1:11357

# 快速验证：关闭临时测试模型的传感器，保留全部碰撞体和惯性
ros2 run nav_sim planar_drive_test.py \
  --output-dir /tmp/planar-drive-core --disable-sensors

# 完整模型：需要有效 DISPLAY，可使用私有 Xvfb 显示
ros2 run nav_sim planar_drive_test.py \
  --output-dir /tmp/planar-drive-sensors
```

测试覆盖前进、双向横移、正反转向、转向后横移、斜向运动、正面及侧面撞墙、零速停车、指令断流、NaN/Inf 和仿真重置。完整模型还订阅并检查激光、RGB 和深度图输出。日志与 `results.json`、`odom_samples.csv`、`truth_samples.csv`、`tf_samples.csv` 保存到指定目录；任一检查失败均返回非零退出码。

Gazebo `ModelStates` 没有时间戳，测试记录其最新接收的 odom 时间；该话题在仿真时间重置后存在发布计时不回退的问题，因此重置后的独立真值改用带时间戳的 `GetEntityState` 服务。

2026-10-29 实测：完整传感器模型的 102 项检查全部通过。`0.2 m/s` 双向横移约 4 秒，位移误差约 `2 mm`；转向后横移正常；墙体可阻挡底盘，测试采样中高度、roll/pitch 无偏移；指令断流约 `0.48 s` 后停止。该空场景测试的仿真实时倍率中位数约 `1.00`，不代表完整导航场景的负载。

## 场景清单

| 场景文件 | 动态障碍物 | 说明 |
|---|---:|---|
| `generated_env_ground_14.world` | 14 | 当前联动默认场景；40 个静态障碍物，5 个动态圆柱和 9 个动态方块 |
| `generated_env_ground.world` | 80 | 完整的地面生成场景；40 个动态圆柱和 40 个动态方块，计算负载更高 |
| `square_dynamic.world` | 3 | 独立 launch 默认场景；方形围墙、静态方块、3 个人物 |
| `square_dense_dynamic_14.world` | 14 | 更密集的方形人物场景 |
| `corridor_dynamic_9.world` | 9 | 走廊人物场景 |
| `floorplan1_dynamic_16.world` | 16 | 平面布局人物场景 |
| `generated_env.world` | 80（原始插件） | 保留的原始场景，引用 `libobstaclePathPlugin.so`；需先迁移为本包插件才能按当前地面仿真方式使用 |

自定义场景放在 `worlds/` 下。动态模型使用本包的 `libobstacle_path_plugin.so`，例如：

```xml
<plugin name="obstacle_motion" filename="libobstacle_path_plugin.so">
  <loop>1</loop>
  <velocity>0.6</velocity>
  <path>
    <waypoint>-6 6 0</waypoint>
    <waypoint> 2 6 0</waypoint>
  </path>
</plugin>
```

## ROS 2 接口与当前限制

下表的相机话题已在当前容器运行态核对；Gazebo 相机插件实际使用 sensor 名称作为前缀，接入方应使用这些话题名。

| 话题 / TF | 类型 / 含义 |
|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist`，AGV 速度输入 |
| `/odom` | `nav_msgs/msg/Odometry`，Gazebo 世界位姿和机器人坐标系速度 |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/camera_depth/depth/image_raw` | `sensor_msgs/msg/Image`，深度图，也是当前 detector 使用的 aligned-depth 输入 |
| `/camera_depth/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera_color/image_raw` | `sensor_msgs/msg/Image`，RGB 图 |
| `/camera_color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/clock` | `rosgraph_msgs/msg/Clock`，仿真时间 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage`；主要链路为 `odom -> base_footprint -> base_link -> lidar_link / camera_link` |
| `/dynamic_obstacles/ground_truth` | 可选的 `vision_msgs/msg/Detection2DArray` |
| `/dynamic_obstacles/ground_truth/poses` | 可选的 `geometry_msgs/msg/PoseArray` |

当前 world 没有配置 Gazebo model-state 发布插件，运行态也没有 `/model_states`。`publish_ground_truth:=true` 只会启动订阅该话题的转换节点；要得到真值输出，还需为 Gazebo 配置对应状态插件和 `/model_states` 话题。该节点默认仅排除 `agv` 和 `ground_plane`，也会包含静态模型，按需设置 `exclude_models`。

日志位于容器内：

| 文件 | 内容 |
|---|---|
| `/tmp/sim.log` | Gazebo、AGV 生成、仿真插件 |
| `/tmp/nav.log` | Nav2、导航网关、intent 等导航节点 |
| `/tmp/static_tf.log` | 静态 `map -> odom` 发布 |
| `/tmp/rviz.log` | 导航 RViz |
| `/tmp/goal.log` | 可选导航目标及反馈 |

查看运行状态可使用 `./scripts/start_all.sh status`。在容器内手动查询话题时，先执行 `export ROS_DOMAIN_ID=42`，保持与启动配置一致。

### 检查速度与 boost 判定

修复多类型冲突后，默认配置下 `/cmd_vel` 只有 `geometry_msgs/msg/Twist`：RViz 的里程计显示订阅 `/odom`，关闭 fine-tune 时网关不创建额外的 TwistStamped 发布器。不要在 Humble 默认速度链中直接启用向 `/cmd_vel` 发布 TwistStamped 的 fine-tune；该功能仍采用原有的 stamped 接口。

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /cmd_vel
# 其他终端分别观察：
ros2 topic echo /cmd_vel_boost_source/status
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_nav_fast
ros2 topic echo /odom --field twist.twist
```

boost 原有门槛保持不变：导航指令 `vx >= 0.60 m/s`、`|vy| <= 0.10 m/s`、`|wz| <= 0.15 rad/s`，雷达接收时间不超过 `0.30 s`，前方 `3 m`、半宽由 footprint 推导的矩形走廊没有有效障碍点。门槛检查的是**导航指令，不是实际 odom 速度**，没有额外的起步等待时间。空扫描或完全无有效量程的扫描不允许 boost。

`/cmd_vel_boost_source/status` 每条输入指令发布一次：`boost_allowed` 表示上述门槛满足；否则为 `below_start_speed`、`lateral_motion`、`turning`、`scan_stale`、`scan_invalid` 或 `corridor_blocked`；非有限速度为 `invalid_command`。这不是实际车速或“已经达到高速”的标志。启用整形时，未满足 boost 条件也会发布受限速的 fast 源，所以 mux 的 `nav_fast` 不能单独证明 boost 已触发。

boost 的第一条输出从零开始，随后按 `boost_max_accel_mps2=1.0` 和输出低通逐步加速；指令断流超过 `command_timeout=0.25 s` 或仿真时钟回退时重置起步状态。最终速度还经过 Nav2 smoother（x 轴加速度上限 `1.5 m/s²`）。没有降低原来的最高速度 `1.20 m/s`，也没有加入额外的底盘电机动力学。接近目标限速节点使用独立 TF 监听线程，避免 Humble 的带 timeout 查询因未启用独立线程而失败。

## License

MIT.

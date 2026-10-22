# Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress

采用 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress。Humble与Fortress是官方默认配对，安装 `ros-humble-ros-gz` 可取得对应集成包。[官方版本配对](https://gazebosim.org/docs/fortress/ros_installation/)

本包先验证理想调度算法：Python推进三机运动和地图观测，Gazebo提供仿真时间与三维显示，不依赖ArduPilot/PX4、相机识别、真实无线链路或旋翼动力学。三机用不同显示高度区分，RSSI仍按二维模型计算。主机固定飞行5个往返、第一轮返程动态移箱、第二轮起重规划。

## 1. 安装环境

先按 [ROS 2 Humble 官方 Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置ROS软件源并安装Humble。该版本的Deb包面向Ubuntu 22.04；系统Python为3.10。随后运行：

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
ros2 --help
ign gazebo --versions
```

Fortress使用 `ign gazebo` 命令。本包的插件名称与消息类型按Fortress编写，请使用上述配对环境。

## 2. 放入工作空间并构建

将本交付目录（包含 `package.xml`、`setup.py`、`ideal_sim/`、`launch/`、`worlds/`）放到 `~/relay_ws/src/warehouse_relay_ideal/`。例如从Windows目录复制或解压后执行：

```bash
mkdir -p ~/relay_ws/src
# 将交付目录复制为 ~/relay_ws/src/warehouse_relay_ideal
cd ~/relay_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select warehouse_relay_ideal
source install/setup.bash
ros2 pkg executables warehouse_relay_ideal
```

若从未初始化rosdep，先执行一次 `sudo rosdep init`。不用安装旧版 `requirements.txt` 中的pymavlink来运行理想实验。

## 3. 随机场景

```bash
cd ~/relay_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch warehouse_relay_ideal ideal.launch.py mode:=random seed:=42 output_dir:=outputs/random42
```

启动文件同时运行Gazebo、`/clock`桥接和调度节点。五轮结束后节点保存JSON/CSV/HTML，Gazebo保留最终场景供观察，退出时按Ctrl+C。

世界包含12个货箱、网格线、蓝色主机与两个绿色系中继，不需要联网下载模型。视觉上移除的货箱移到地板下方，Python环境真值则真正删除对应占用单元。请通过 `/swarm/remove_cells` 移箱，直接在Gazebo界面删模型不会同步算法地图。

## 4. 手动窗口场景

终端A：

```bash
source /opt/ros/humble/setup.bash
source ~/relay_ws/install/setup.bash
ros2 launch warehouse_relay_ideal ideal.launch.py mode:=window window_start_s:=2.0 window_end_s:=12.0 output_dir:=outputs/window_manual
```

终端B预先准备，观察窗口事件：

```bash
source /opt/ros/humble/setup.bash
source ~/relay_ws/install/setup.bash
ros2 topic echo /swarm/events
```

看到 `window_open` 后，在终端C提交一次移除任务：

```bash
source /opt/ros/humble/setup.bash
source ~/relay_ws/install/setup.bash
ros2 topic pub --once /swarm/remove_cells std_msgs/msg/String \
  "{data: '{\"id\":\"manual-1\",\"cells\":[[1,1],[2,2],[3,4]]}'}"
```

货箱编号为1基。默认窗口为首轮返程开始运动后第2–12秒。查看 `removal_command` 的 `accepted` 字段；发送成功不等于业务接受。非法数量、重复坐标、窗口外、剩余返程来不及观测的货箱会被拒绝。窗口到期不会自动移箱，未接受任务会被标记为未完成该场景。

## 5. 时钟、话题与显示

```text
Gazebo /clock → ros_gz_bridge → ROS use_sim_time
                                 ↓
                       ideal_sim 理想算法核心
                     ↙             ↓              ↘
             /swarm/state    /swarm/main/path    /swarm/events
                     ↓
       Fortress /world/warehouse/set_pose_vector
```

`/swarm/state` 是JSON字符串，含轮次、方向、窗口、三机位置、路由、已知地图和已移除区块（显示信息）；模型接口只获得 `SchedulingState`，不获得该显示消息中的环境真值。`/swarm/main/path` 是 `nav_msgs/Path`，可用RViz以 `map` 为固定坐标系观察轨迹。

时钟桥使用单向Gazebo→ROS，Humble桥支持 `rosgraph_msgs/Clock` 与 `ignition.msgs.Clock`。[ros_gz Humble 接口](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md)

三维位姿由独立工作线程通过 `ign service` 调用Fortress的 `set_pose_vector`，只保留最新待发送帧，显示延迟不阻塞算法。该服务在Fortress的UserCommands系统中实现。[Fortress UserCommands 源码](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/user_commands/UserCommands.cc)

Gazebo暂停时算法时间和操作窗口同步暂停；恢复后继续。仿真时钟重置时节点停止推进并要求重启实验，避免将新时钟混入原五轮记录。

## 6. 检查与排错

```bash
ros2 topic echo /clock --once
ros2 topic echo /swarm/state --once
ign service -l
```

- 无 `/clock`：检查Gazebo是否启动/暂停、桥接是否运行；不要另起一个竞争的时钟发布者。
- 没有 `/world/warehouse/set_pose_vector`：检查Fortress版本与世界的UserCommands插件是否加载。
- GUI启动失败：先确认Ubuntu图形/OpenGL环境。可分别运行 `ign gazebo -s -r <warehouse.sdf绝对路径>`、时钟桥和调度节点，做无GUI验证。
- 算法日志正常而三维不动：查看节点的“Gazebo显示未同步”警告。服务返回代表命令入队，本实验不把显示命令当飞控执行确认。

无Gazebo时也可通过系统时间单独测试ROS消息入口：

```bash
ros2 run warehouse_relay_ideal ideal_scheduler --ros-args \
  -p use_sim_time:=false -p gazebo_view:=false -p mode:=window
```

上述命令用于诊断ROS层；纯Python完整实验可直接运行 `python3 demo.py`。

## 7. AI扩展与验证状态

`model_plugin` 启动参数可指定Python工厂，例如：

```bash
ros2 launch warehouse_relay_ideal ideal.launch.py \
  model_plugin:=ideal_sim.example_model:build_model
```

该例子仅演示手写候选评分。真实模型输入、风险约束、错误回退及训练数据接口见 [IDEAL_SIM.md](IDEAL_SIM.md)。

当前交付已在Windows Python运行理想核心、自动测试和两种场景的离线产物；ROS节点、launch和SDF做静态检查。**尚未在Ubuntu 22.04实际启动ROS 2/Gazebo**，不能将其称为已验证三维联调或飞控闭环。Ubuntu侧验收应记录：10个单程、230m主机航程、首轮返程移除与地图更新、第二轮对照、命令拒绝/接受、暂停恢复和最终返场。

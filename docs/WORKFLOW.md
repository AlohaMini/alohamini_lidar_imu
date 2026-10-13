# AlohaMini C1、建图、导航和本机 RViz 流程

本项目把运行职责分成两个 Docker 容器：

- 树莓派的 `alohamini_nav2`：连接 C1 和底盘，运行 C1 驱动、底盘驱动、SLAM 和 Nav2。
- 本机 PC 的 `alohamini_rviz_humble`：运行 RViz，通过 ROS 2 网络查看树莓派发布的地图、雷达、TF、路径和相机。

两台机器必须在同一局域网，使用 host 网络，并设置相同的 `ROS_DOMAIN_ID=5` 和
`ROS_LOCALHOST_ONLY=0`。容器的创建、依赖安装和 `/dev/lidar_c1` udev 别名配置见
[INSTALL.md](INSTALL.md)。

## 1. 启动两个 Docker 容器

### 1.1 树莓派容器

在树莓派宿主机执行：

```bash
docker start alohamini_nav2
docker exec -it alohamini_nav2 bash
```

进入容器后，每次新开终端都执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

确认两个串口都在容器中可见：

```bash
ls -l /dev/lidar_c1 /dev/ttyUSB0 /dev/ttyACM0
```

如果没有配置 C1 的 udev 别名，把下面命令中的 `/dev/lidar_c1` 换成实际的
`/dev/ttyUSB0`。底盘串口仍使用 `serial_port` 参数（默认 `/dev/ttyACM0`）。

### 1.2 本机 PC RViz 容器

PC 首次使用先拉取镜像，并按 [INSTALL.md](INSTALL.md) 创建容器：

```bash
docker pull docker.io/osrf/ros:humble-desktop
docker start alohamini_rviz_humble
```

然后在 PC 终端进入容器：

```bash
docker exec -it alohamini_rviz_humble bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

PC 容器需要挂载 X11 才能显示 RViz；首次创建时执行 `xhost +local:root`。本机只
查看树莓派数据时并不依赖 PC 的 `/dev`，但本项目按安装文档保留 `/dev:/dev`，这样
同一个容器也可以直接测试连接到 PC 的 C1。

## 2. 单独测试 C1

雷达插在运行驱动的那台机器上。通常先在树莓派容器中测试：

```bash
ros2 launch alohamini_rplidar_c1 test_rplidar_c1.launch.py \
  serial_port:=/dev/lidar_c1
```

另开一个树莓派或 PC 容器终端确认数据：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

测试节点应持续报告 `frame=laser_frame`、点数、有效点数、频率、角度和距离范围。
独立测试完成后按 `Ctrl+C` 停止测试节点，再启动完整建图，确保雷达串口只有一个驱动使用。

需要直接在 PC 测试 C1 时，必须把雷达插入 PC，并使用安装文档中带 `/dev` 映射的
`alohamini_rviz_humble` 容器，在该容器内执行同一个 launch 命令。

### 在 RViz 中查看原始 `/scan`

启动 RViz：

```bash
ros2 launch alohamini_bringup rviz.launch.py
```

单独测试没有完整建图 TF 时，在 RViz 中手动设置：

1. `Global Options -> Fixed Frame` 设置为 `laser_frame`。
2. `LaserScan -> Topic` 设置为 `/scan`。
3. 暂时关闭 `Map`、`RobotModel` 和 `Scan Sector` 显示。

### 独立测试IMU

将 IMU 插入运行驱动的树莓派或 PC，按 [IMU 说明](../ros2_ws/src/alohamini_imu/README.md)
安装 `python3-serial` 并编译 `alohamini_imu`。在相应 Humble 容器中执行，设备路径以实物为准：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 launch alohamini_imu test_imu.launch.py serial_port:=/dev/ttyUSB0
```

启动时读取实际量程和输出速率，每秒显示加速度、角速度、姿态、频率和数据年龄；
新运动帧中断时测试会报错结束。另开终端可执行 `ros2 topic echo /imu/data_raw --once`。
先检查静止重力模长约 9.81 m/s²、角速度接近零，再手动转动验证轴向和正负号。
本入口只读 IMU，融合到建图／导航的工作另见 [融合方案](IMU_FUSION_PLAN.md)。

## 3. 在树莓派启动 SLAM 建图

建图命令应在树莓派的 `alohamini_nav2` 容器内执行。推荐使用带键盘控制和地图保存
快捷键的会话脚本：

```bash
ros2 run alohamini_bringup alohamini_mapping_session \
  --serial-port /dev/ttyACM0 \
  --lidar-serial-port /dev/lidar_c1 \
  --map /root/ws/maps/alohamini_map \
  --enable-head-camera
```

不需要相机时，删除 `--enable-head-camera`。如果使用其他底盘串口或关闭相机，按实际
设备修改参数。

会话脚本所在终端负责键盘控制和保存地图：

```text
w             前进
a / d         原地左转 / 原地右转
s             后退
z / x         左移 / 右移
r / f         提高 / 降低速度
Space 或 0    发送一次零速 /cmd_vel
Shift+S       保存地图并继续建图
Shift+X       保存地图，停止建图并退出
q             不保存，停止建图并退出
Ctrl+C        不保存，停止建图并退出
```

地图保存为：

```text
/root/ws/maps/alohamini_map.yaml
/root/ws/maps/alohamini_map.pgm
```

由于 `/root/ws` 是宿主机工作区挂载，树莓派宿主机对应目录为：

```text
~/alohamini_lidar_imu/ros2_ws/maps/
```

## 4. 本机 PC 用 RViz 查看建图

建图进程启动后，在 PC 的 `alohamini_rviz_humble` 容器中另开终端：

```bash
docker exec -it alohamini_rviz_humble bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch alohamini_bringup rviz.launch.py
```

默认 RViz 配置已经加载：

- `Fixed Frame`：`map`
- 地图：`/map`
- 雷达：`/scan_filtered`
- 机器人模型：`/robot_description`
- TF：`/tf` 和 `/tf_static`
- 建图期间可打开 `Odometry` 查看 `/odom`
- 启用相机后可打开 `Head Camera` 查看 `/head_camera/image_raw/compressed`

如果 RViz 没有数据，先在 PC 容器内确认 DDS 发现和话题：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan_filtered --once
ros2 topic echo /map --once
```

建图时的运动键仍在树莓派会话脚本终端中输入；PC 的 RViz 负责实时观察扫描、地图和
机器人 TF。

### 4.1 查看头部摄像头

树莓派上的建图会话需带 `--enable-head-camera`；直接使用建图或导航 launch 时，
传入 `enable_head_camera:=true`。这会同时启动头部相机，无需另开相机进程。

在 PC 的 RViz 中：

1. 在左侧 `Displays` 中找到并勾选 `Head Camera`，点击左侧箭头展开。
2. 将 `Topic` 设为 `/head_camera/image_raw/compressed`。
3. 展开 `Topic`，将 `Reliability Policy` 设为 **`Best Effort`**。
4. `Transport Hint` 使用 `compressed`，上方的 `Head Camera` 图像窗口即可显示机器人视角。

项目提供的 RViz 配置已保存这些设置。使用自己保存的配置时，也按上述步骤检查。

![头部摄像头](./image/6.png)

相机默认设备为树莓派上的 `/dev/am_camera_forward`，默认输出为 `640×480`、10 FPS。
若设备路径不同，建图会话通过 `--head-camera-device /dev/videoN` 指定，直接启动
建图或导航 launch 时使用 `head_camera_device:=/dev/videoN`。

只查看相机时，可在树莓派容器中单独启动：

```bash
ros2 launch alohamini_bringup head_camera.launch.py device:=/dev/am_camera_forward
```

然后在 PC 容器中按本节前面的命令启动 RViz。相机已随建图或导航启动时，不再重复
运行这条命令。若没有图像，可在 PC 容器中确认发布者和消息：

```bash
ros2 topic info /head_camera/image_raw/compressed --verbose
ros2 topic echo /head_camera/image_raw/compressed --field header --once \
  --qos-reliability best_effort
```

## 5. 在树莓派启动 Nav2 导航

先退出建图会话（`Shift+X` 保存并退出），再在树莓派的 `alohamini_nav2` 容器中执行：

```bash
ros2 launch alohamini_bringup navigation_ros2_control.launch.py \
  serial_port:=/dev/ttyACM0 \
  lidar_serial_port:=/dev/lidar_c1 \
  map:=/root/ws/maps/alohamini_map.yaml \
  enable_head_camera:=true
```

如果不需要相机，将 `enable_head_camera:=true` 改为 `false`。

导航启动后，在 PC RViz 中按下面顺序操作：

1. `Global Options -> Fixed Frame` 保持为 `map`。
2. 确认 `Map` 显示 `/map`，`LaserScan` 显示 `/scan_filtered`。
3. 点击工具栏 `2D Pose Estimate`，在地图上点击机器人实际位置并拖动箭头设置朝向；
   等待粒子云收敛到机器人附近。
4. 确认机器人模型、雷达扫描和地图墙体基本重合。
5. 点击工具栏 `Nav2 Goal` 或 `2D Goal Pose`，在可通行区域点击目标点并拖动箭头设置
   到达朝向。
6. 第一次先发送约 `0.5` 到 `1` 米的短距离目标，观察 global plan、local plan 和
   机器人运动是否正常，再发送更远的目标。

导航过程中可以在 PC RViz 打开 `Global Costmap`、`Local Costmap` 和 `Odometry`，用于
检查避障和定位；树莓派容器继续运行底盘、C1 和 Nav2 节点。

## 6. 再次使用 PC RViz 容器

容器已经创建并安装依赖后，每次只需：

```bash
docker start alohamini_rviz_humble
docker exec -it alohamini_rviz_humble bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch alohamini_bringup rviz.launch.py
```

树莓派容器重启后的进入方式：

```bash
docker start alohamini_nav2
docker exec -it alohamini_nav2 bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
```

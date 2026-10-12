# AlohaMini 雷达与导航

本项目使用树莓派直接连接思岚 RPLIDAR C1，通过 ROS 2 发布激光数据，并提供
底盘控制、SLAM 建图、Nav2 导航和 RViz 启动入口。

## 目录

```text
alohamini_lidar_imu/
├── ros2_ws/src/
│   ├── alohamini_rplidar_c1/      # C1 ROS 2 驱动、完整内置 Slamtec SDK、扫描测试
│   ├── alohamini_description/     # 机器人模型和 laser_frame TF
│   ├── alohamini_base_control/    # ros2_control 底盘驱动
│   ├── alohamini_nav_bridge/      # 可选 ZMQ 底盘桥
│   ├── alohamini_bringup/         # 传感器、SLAM、Nav2、RViz 启动入口
│   └── third_party/geometry2/     # TF2 0.25.24 源码依赖
└── docs/
    ├── INSTALL.md                 # Docker 和 ROS 2 依赖
    └── WORKFLOW.md                # C1 测试、建图、导航和 RViz 流程
```

运行环境统一使用 ROS 2 Humble Docker：树莓派容器负责 C1、底盘、建图和导航，本机 PC
容器负责 RViz 可视化。树莓派 Docker、PC RViz 容器和依赖安装见
[docs/INSTALL.md](docs/INSTALL.md)；从 C1 测试到 PC 查看建图、设置导航起点和发送目标点
的完整顺序见 [docs/WORKFLOW.md](docs/WORKFLOW.md)。

树莓派和 PC 都需要编译整个工作区，每次运行前依次 source ROS 2 和工作区的
`setup.bash`。工作区依赖随项目一起编译，无需额外下载雷达 SDK 或 TF2 源码。

## 话题和默认硬件参数

- C1 原始话题：`/scan`
- 建图 / 导航话题：`/scan_filtered`
- C1 串口默认：`/dev/ttyUSB0`（推荐配置 udev 固定别名 `/dev/lidar_c1`）
- C1 波特率：`460800`
- 扫描模式：`Standard`
- frame：`laser_frame`
- ROS Domain ID：`5`

C1 驱动和测试说明见 [alohamini_rplidar_c1/README.md](ros2_ws/src/alohamini_rplidar_c1/README.md)。
树莓派 USB 固定别名配置见 [安装文档](docs/INSTALL.md#配置-devlidar_c1-udev-别名)。
雷达安装位置和姿态见 [alohamini_nav.urdf](ros2_ws/src/alohamini_description/urdf/alohamini_nav.urdf)。
默认安装方式为 C1 正装、出线口朝车尾，建图和导航默认使用前方 180° 扫描区域。

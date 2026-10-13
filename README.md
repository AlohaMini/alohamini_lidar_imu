# AlohaMini 雷达与导航

本项目使用树莓派直接连接思岚 RPLIDAR C1，通过 ROS 2 发布激光数据，并提供
底盘控制、SLAM 建图、Nav2 导航和 RViz 启动入口。
幻尔十轴 IMU 的 USB 独立读取与验证入口已提供，见
[IMU 使用说明](ros2_ws/src/alohamini_imu/README.md)。

## 目录

```text
alohamini_lidar_imu/
├── ros2_ws/src/
│   ├── alohamini_rplidar_c1/      # C1 ROS 2 驱动、完整内置 Slamtec SDK、扫描测试
│   ├── alohamini_imu/             # 幻尔十轴 USB IMU 读取、单位换算和数据验证
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

## IMU 当前安装方向与轴向约定

幻尔十轴 IMU 通过 USB 连接树莓派，发布 `/imu/data_raw`，消息坐标系为 `imu_link`。
当前安装方式已通过抬起左侧、右侧、前端及水平正反转验证，输出轴向对应机器人
**X 向前、Y 向左、Z 向上**。这里的前、后、左、右均以机器人自身朝向为准，
不是观察者面对机器人时的左右。

| 轴 | 正方向 | 绕该轴转动时，角速度为正的动作 |
|---|---|---|
| X | 机器人前方 | 左侧抬高、右侧降低，即正 roll |
| Y | 机器人左方 | 前端下俯、后端抬高，即正 pitch |
| Z | 机器人上方 | 从机器人上方往下看，逆时针转动，即正 yaw |

三轴转动均遵循右手定则：右手拇指指向轴的正方向，四指弯曲方向为正转动。
日志中 `acc=[ax, ay, az]` 的单位为 m/s²，`gyro=[gx, gy, gz]` 的单位为 rad/s，
`wz` 是 `gz` 换算成 °/s 的结果，`rpy=[roll, pitch, yaw]` 的单位为度。
在本次接近水平的测试姿态下，绕对应轴正转时，相应姿态角增大；yaw 跨过 ±180°
时显示值会回绕，不代表实际转向改变。

2026-10-13 实测确认的动作与读数关系如下：

| 验证动作 | 转动过程中的角速度 | 倾斜保持静止时的加速度与姿态 |
|---|---|---|
| 抬起左侧，再放平 | 抬起时 `gx > 0`，放平时 `gx < 0` | `ay` 变为正值，roll 为正 |
| 抬起右侧，再放平 | 抬起时 `gx < 0`，放平时 `gx > 0` | `ay` 变为负值，roll 为负 |
| 抬起前端，再放平 | 抬起时 `gy < 0`，放平时 `gy > 0` | `ax` 变为正值，pitch 为负 |
| 水平逆时针转动，再顺时针转回 | 逆时针 `gz > 0`，顺时针 `gz < 0` | yaw 随转动增大、减小 |

加速度读数包含重力对应的比力：水平静止时 `az` 应接近 **+9.81 m/s²**，
`ax`、`ay` 接近零；轻微倾斜会让 X、Y 轴出现重力分量。因此，倾斜后保持静止仍有
非零加速度读数是正常现象，不能将其直接当作机器人平移加速度。

保持当前安装方向时，无需为了轴向符号额外反号或交换 XYZ。本次验证确认的是方向，
尚不代表零偏、安装角和协方差已完成标定；驱动也不会自动发布安装 TF。
重新安装或旋转模块后，应重新验证上述对应关系，并配置实际安装变换。

# 幻尔十轴 IMU 独立读取与验证

本包参考用户提供的 `10轴IMU惯导模块` 本地通信协议和树莓派 ROS 2 `imu_usb` 示例，
提取 USB 数据读取功能，使用原生 ROS 2 Humble 节点。源码和协议的版本核对见
[资料对照](../../../docs/research/imu-local-manual-comparison.md)。运行不依赖 Downloads 目录。

支持 **CH340X、`55 55 ID LEN DATA SUM` 协议**，默认打开 `/dev/imu_usb`、115200 波特率。
先通过 `55 AF` 读命令确认实际加速度量程、陀螺仪量程、回传速率和上报开关，再进行 SI
换算。读回失败就退出，不猜测量程。该示例只发送读取请求，不修改量程、频率、算法、
标定或 Flash，不启动底盘、雷达、TF、EKF 或导航。

## 编译

在项目的 ROS 2 Humble 容器内：

```bash
apt-get update
apt-get install -y python3-serial
source /opt/ros/humble/setup.bash
cd /root/ws
colcon build --symlink-install --packages-select alohamini_imu
source install/setup.bash
```

IMU 插在树莓派时，在树莓派容器运行；插在 PC 时，也可以在 PC Humble 容器独立测试。
确保设备在该容器的 `/dev` 中可见。USB 已为模块供电，不重复接排针电源。
已有 `/dev/imu_usb` 别名可直接使用；没有时按插拔前后的设备变化确定串口，避免误选 C1。
不要直接照搬仅匹配 CH340 VID/PID 的规则：多个同类 USB 设备会争用同一个别名。

## 一键读取与验证（与 C1 测试入口相同用法）

```bash
ros2 launch alohamini_imu test_imu.launch.py \
  serial_port:=/dev/imu_usb
```

未配置别名时，例如实际设备是 `/dev/ttyUSB1`：

```bash
ros2 launch alohamini_imu test_imu.launch.py \
  serial_port:=/dev/ttyUSB1 serial_baudrate:=115200
```

启动日志先显示读回的量程、频率和上报掩码；随后约每秒显示：

```text
IMU verified: acc=4G gyro=1024dps report=100Hz flags=0x1f; frame=imu_link
imu ok: frame=imu_link rate=99.8 Hz age=4.2 ms acc=[0.010, -0.020, 9.801] m/s^2 |a|=9.801 gyro=[0.0001, -0.0002, 0.0003] rad/s wz=0.02 deg/s rpy=[0.1, -0.1, 12.3] deg rejected=0
```

这是格式示例，不是实物测量结果。初次报告可能只覆盖部分采样周期。
`imu ok` 表示消息格式、时间戳和新鲜度通过检查，不表示已经完成零偏标定或保证定位精度。
`Ctrl+C` 结束。配置失败、持续无有效运动帧、串口异常或检查节点失败时，测试 launch
会结束整个测试会话。节点故障退出码为 1；ROS launch 自身的退出码不等同于子节点退出码。

另开一个加载过 ROS 环境的终端：

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data_raw --once
ros2 topic echo /imu/temperature --once
```

只运行驱动：

```bash
ros2 launch alohamini_imu imu.launch.py serial_port:=/dev/imu_usb
```

检查一个已经存在的 IMU 话题：

```bash
ros2 run alohamini_imu imu_data_test --ros-args \
  -p imu_topic:=/imu/data_raw -p frame_id:=imu_link
```

## 实物验证顺序

1. **静止放置**：先让模块上电静止至少 2～3 秒，按本地教程完成必要标定。观察三轴角速度
   是否接近零；含重力的加速度模长应接近 9.81 m/s²。Z 朝上水平放置时，主要应为正 Z 加速度。
   连续观察 30～60 秒记录波动，不把固定的示例协方差当成实测精度。
2. **分别转动三轴**：核对相应角速度分量和正负号。按右手定则绕朝上的 Z 轴逆时针转动时
   应为正角速度。姿态来自模块自身算法，初始 yaw 不要求为零，也尚未验证其世界参考方向。
3. **慢转并回到起点**：观察正反转、姿态连续性和停止后的零偏；跨 ±180° 时显示的欧拉角
   可以跳边界，这不等于四元数跳变。
4. **停止数据／拔线**：应出现明确错误并结束会话，不应保持旧数值却持续显示新时间戳。

本包发布加速度、角速度、可用的四元数及陀螺仪温度。磁场和气压帧可以存在于串口流，
本阶段不额外发布它们。`/imu/data_raw` 名字沿用厂商习惯，但有有效四元数时也携带姿态。
姿态缺失、异常或超过 `orientation_timeout_sec` 时设 `orientation_covariance[0]=-1`。

## 时序、参数与排查

运动数据只由新收到且通过校验的 `0x03` 帧触发发布。运动帧没有硬件时间戳，消息使用
主机接收后的 ROS 时间；如果一次读取积压了多条运动帧，只保留最后一条并统计丢弃数量，
避免把积压样本作为一批同时采集的数据。该策略不是硬件时钟同步，接入融合前仍需测量延迟。
四元数单独检查新鲜度并归一化；本包不进行安装坐标变换或 ENU/NED 转换。

| launch 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_port` | `/dev/imu_usb` | 所连主机上的实际设备或固定别名 |
| `serial_baudrate` | `115200` | 仅设置主机串口，需与模块当前配置一致 |
| `frame_id` | `imu_link` | 消息坐标系名称，不会自动产生 TF |
| `imu_topic` | `/imu/data_raw` | `sensor_msgs/msg/Imu`，SensorDataQoS |
| `temperature_topic` | `/imu/temperature` | `sensor_msgs/msg/Temperature`，摄氏度 |
| `data_timeout_sec` | `2.0` | 连续无有效运动数据后报错退出 |
| `orientation_timeout_sec` | `0.2` | 超过此时间未更新的姿态标为不可用 |
| `startup_timeout_sec` | `10.0` | 检查节点等待首条合格消息的时间，仅测试 launch |
| `report_period_sec` | `1.0` | 检查节点输出周期，仅测试 launch |
| `angular_velocity_variance` | `0.01` | 三轴角速度对角方差，单位 `(rad/s)²`，待标定 |
| `linear_acceleration_variance` | `0.04` | 三轴加速度对角方差，单位 `(m/s²)²`，待标定 |
| `orientation_variance` | `0.01` | 姿态对角方差，单位 `rad²`，待标定 |

- 无法打开串口：核对设备路径、容器设备映射、用户串口权限以及上位机是否占用。
- 配置读回超时：核对波特率和模块版本。此驱动不支持旧 `55 51/52/53` 的 WIT 协议。
- 设备主动上报或运动帧被关闭：用本地上位机开启，保存后再测试。本示例不替你改变配置。
- 校验／长度错误不断增加：检查协议、波特率、接线、USB 供电；日志累计计数可辅助定位。
- `superseded_motion_frames` 增长：说明一次轮询收到了多条运动帧，应检查主机负载和串口积压。
- 低于配置的发布频率：先比较有效帧、积压计数与主机负载；不要只看厂商上位机的频率选项。

## 自动验证

```bash
colcon test --packages-select alohamini_imu --event-handlers console_direct+
colcon test-result --verbose
```

测试覆盖协议分包／粘包／错误恢复、已知物理量换算、姿态失效、配置读回和真实 ROS 2
节点连接伪终端串口的链路。伪终端不会连接实物，不能代替上述实物验收。

## 树莓派实测记录（2026-10-13）

代码已部署到树莓派 `/home/pi5/alohamini_lidar_imu/ros2_ws/src/alohamini_imu`，
在 `alohamini_nav2` 容器中编译并连接 `/dev/ttyUSB0` 验证。重新运行：

```bash
ssh pi5@192.168.10.135
docker exec -it alohamini_nav2 bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5
ros2 launch alohamini_imu test_imu.launch.py serial_port:=/dev/ttyUSB0
```

| 检查项 | 实测结果 |
|---|---|
| 配置读回 | 115200 波特率；4G；1024 dps；100 Hz；上报掩码 `0x1f` |
| 连续采集 | 20.01 秒，1997 条消息，平均 99.76 Hz |
| 时间戳 | 严格递增，最大相邻间隔 12.04 ms |
| 校验与格式 | 无校验、长度或串口积压告警，检查节点拒收为 0 |
| 加速度模长 | 均值 9.908 m/s²，范围 9.885～9.927 m/s² |
| Z 轴角速度 | 均值约 0.000055 rad/s |
| 四元数与温度 | 1997 条消息均有有效姿态；温度均值 32.84 °C |
| 退出 | 中断后两个节点正常结束，串口已释放 |

容器内原始记录为 `/root/ws/log/imu_validation_20261013.log` 和同名 `.json`，
对应树莓派项目下的 `ros2_ws/log/`。本地 Humble 容器的 13 项自动测试也已通过，
覆盖断流和中断退出。树莓派容器已安装 pyserial 3.5；当时已有其他 apt 任务占用锁，
因此通过离线 wheel 安装到容器内 root 的 Python 用户目录。

这次实测确认了读取链路、单位换算和退出行为；未进行人工三轴转动、安装方向确认、
零偏／比例因子标定或导航融合。加速度模长与标准重力仍有偏差，接入融合前应继续标定。
JSON 中的消息年龄只表示 ROS 发布时间到订阅接收时间，不代表传感器到主机的全部延迟。

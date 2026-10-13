# 二维导航 IMU 融合：官方资料核实

核实日期：2026-10-13。适用 ROS 2 Humble；源码按对应 `humble` / `humble-devel` 分支核实。本文是方案依据，不表示项目已实现或完成实车验证。

**硬件资料修订：** 用户随后提供的本地资料与先前网上资料不是同一套硬件／协议描述。
当前方案以本地 CH340X、115200／100 Hz、`55 55` 变长报文、Humble `imu_usb` 为准。
差异与勘误见[本地教程对照](imu-local-manual-comparison.md)，源码接入事项见[本地驱动核对](imu-local-driver-review.md)。
下文 ROS 融合原理仍适用；末尾 WIT 示例部分仅保留为来源辨别记录，不再作为当前模块的接入依据。

## 推荐融合范围

对平面全向底盘，建议第一阶段融合轮速里程计的 `vx、vy` 与 IMU 陀螺仪的车体 `wz`，设置 `two_d_mode: true`。这是根据官方状态变量与平面模式能力作出的项目方案选择，并非唯一配置。`two_d_mode` 约束高度、横滚、俯仰等三维变量，不禁止横向速度；全向车不能照抄差速车“`vy` 恒为 0”的约束。[1][2]

若轮式里程计的位置、姿态、速度都由同一组编码器计算，不应将这些重复来源全部当作独立观测融合。官方配置教程建议此时优先使用速度。[1]

以下配置顺序为 `x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az`；示例假定 IMU 的 Z 轴与车体朝上一致。若侧装，应先依据完整外参和原始轴定义选择、转换有效角速度分量，不能机械地只取原始 `angular_velocity.z`。[1][3]

```yaml
two_d_mode: true
world_frame: odom
odom_frame: odom
base_link_frame: base_footprint
publish_tf: true
odom0: /wheel/odom
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,
               false, false, false,
               false, false, false]
imu0: /imu/data_raw
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: false
```

这是参数片段；完整节点 YAML、输出重映射、频率、噪声参数、安装外参仍需结合项目落实。配置向量按输入传感器坐标定义；`Odometry.twist` 所在坐标由 `child_frame_id` 指定，不能仅靠修改 frame 名称来旋转数据。[1][3]

## IMU 数据规范

- `sensor_msgs/msg/Imu` 角速度单位是 `rad/s`，加速度是 `m/s²`；若模块输出 `°/s` 和 `g`，需转换。不能将度数直接填入四元数或角速度。[4]
- ROS 车体坐标是 X 前、Y 左、Z 上，右手系；逆时针转动时 `wz` 为正。`imu_link` 描述实际传感器坐标；固定 TF 描述它相对底盘的真实安装位置和旋转。[3][5][6]
- 安装旋转与 ENU/NED 世界参考系转换是两件事。`robot_localization` 要求符合 ENU 约定的数据；若以后融合模块姿态，必须确认其四元数/欧拉角参考系、旋转顺序和零点，不能认为补一个安装 TF 就自动纠正所有 NED 姿态。[3][6]
- 没有姿态估计时，设置 `orientation_covariance[0] = -1`。协方差全 0 表示未知，**不是无噪声**；用于融合的角速度应有经数据或规格得到的合理非零方差。[4][6]
- Humble EKF 源码会把过小的测量方差钳制到很小的正值；因此不能期待“填 0”让 EKF 自动判断未知噪声。方差是标准差的平方，单位也需随数据转换。[7]
- 正放静止、输出仍含重力的加速度应约为 `az = +9.81 m/s²`。第一阶段不融合加速度；以后若开启重力去除，需先确认固件是否已去重力及姿态来源是否可靠，避免去除两次。[2][3][6]
- 六轴 IMU 的加速度可提供重力方向，但没有磁力计等绝对航向参考时，绕 Z 的方向没有绝对基准。不能仅因模块输出了 yaw 就把它当作不会漂移的航向。[6]

## 数据实际进入哪些组件

| 组件 | 核实到的输入/职责 | 对方案的影响 |
|---|---|---|
| `robot_localization` EKF | 可融合里程计、IMU 等；`world_frame=odom` 时输出 `odom → base_link_frame`，默认里程计话题为 `odometry/filtered`。[2] | 负责短时间连续运动估计，融合输出可重映射为项目 `/odom`。 |
| `slam_toolbox` Humble | README 输入列表为 LaserScan 与 `odom_frame → base_frame` TF；实现创建激光订阅和 TF 姿态查询，没有直接 IMU 订阅。[8] | 通过 EKF 改进后的里程计 TF 间接使用 IMU；加 IMU 话题参数不会自动形成激光惯性 SLAM。 |
| AMCL Humble | `getOdomPose()` 在激光时间戳从 TF 求底盘在 odom 中的姿态；发布 `map → odom`。[9] | 改进其运动预测靠里程计 TF，不能只修改某个 `/odom` 话题配置。 |
| AMCL 运动模型 | Humble 插件表与源码均包含 `nav2_amcl::OmniMotionModel`；默认是差速模型。[9][10] | 可横移底盘应评估采用全向模型，并据运动实测调整误差参数。 |
| Nav2 controller_server | 使用 `OdomSubscriber`，该订阅器读 `odom_topic` 并提取 `vx、vy、wz`。[11] | Nav2 速度反馈应指向融合后的 `/odom`；保留横向速度。 |
| Nav2 velocity_smoother | 只有 `feedback=CLOSED_LOOP` 时通过 `odom_topic` 创建里程计速度反馈；`OPEN_LOOP` 使用自己的输出反馈。[12] | 并非所有配置下的平滑器都会直接消费融合里程计。 |

标准 TF 分工如下：

```text
map --(建图时 slam_toolbox / 定位时 AMCL，二选一)--> odom
odom --(EKF，唯一发布者)--> base_footprint
base_footprint --(URDF / 固定 TF)--> base_link / imu_link / laser_frame
```

上图最后一行表示固定传感器链路，实际父子级可按项目 URDF 保留。REP-105 将 `odom` 定义为连续、允许长期漂移的局部参考；`map` 用于长期全局定位，允许校正跳变。官方明确要求：EKF 接管 `odom → base` 后关闭底盘驱动的同一 TF 广播，避免同一边有多个发布者。[3][13]

## 不能期待 EKF 自动解决的事项

- Humble `robot_localization` 的状态向量只有 15 维位置、姿态、速度、加速度，没有陀螺仪/加速度计 bias 状态。不能承诺仅接入 EKF 就能在线自动估计和消除 IMU 零偏；静止标定、温漂验证仍需由模块、驱动或预处理实现。[14]
- 只融合轮速与角速度没有绝对位置/航向观测，长期仍会漂移；地图坐标中的纠偏由激光匹配、回环或定位承担。高质量短时间 odom 并不等于长期无漂移。[1][13]
- `sensor_timeout` 的作用是超时后预测，不是将禁用的轮速 `wz` 自动开启。若第一阶段只使用 IMU `wz`，IMU 断流后的停止、报警、人工切换纯轮速模式需要另行设计。[2]

## 另一套网上资料：仅作版本对照

以下只描述先前读取的幻尔网页 `IMU Module` 及附录 `wit_ros2_imu` 示例。
**用户本地资料已经证实采用不同 USB 芯片、默认参数、协议和 ROS 包，以下事实不能套用到当前本地模块。**
其中 CP2102／9600／Foxy／WIT 的区别不是修改几个参数就能消除的，尤其不能复用 WIT 解析器。

- 手册列出陀螺仪、加速度计、磁力计和气压计；Type-C 连接通过 CP2102 转 USB 串口。这里的“十轴”不意味着十个姿态自由度，也不表示自带 GPS；GPS 是外接扩展。[15]
- 手册默认配置为 **9600 bps、10 Hz**，波特率可设 4800–921600，输出频率可设 0.2–200 Hz；数据量大、串口带宽不足时会自动降低实际输出频率，官方建议高频时提高到约 115200。故“先用当前波特率连接，再将模块和驱动一起配置为 115200、50–100 Hz”是合理起点，最终频率必须实测。[15]
- 官方列出的可输出字段包括模块时间、加速度、角速度、磁场、姿态角、四元数、气压和高度。平面导航第一阶段只融合角速度，气压计无需进入二维 EKF。[15]
- 手册支持六轴/九轴姿态算法，九轴引入磁力计，并声明其 Z 角采用 ENU、角度为零时模块 Y 指北；上位机车模也是 Y 为前。接入 ROS 时应按实物坐标标识和安装外参验证，不能照搬车模方向。PC 软件示例选择 `JY901B`；这不是对用户实机型号的独立鉴定。[15]
- 模块陀螺仪自动标定默认开启，存在静止阈值和稳定时间；验收需包含持续慢转，确认没有被静止判定抑制真实角速度。磁力计姿态应在带电底盘环境下检查干扰后再决定是否融合。[15]
- 官方 ROS 2 教程基于 **Ubuntu 20.04 / Foxy**，提供 `wit_ros2_imu`、`/imu/data_raw` 和 Python 串口示例；未据此认定 Humble 已受验证。[16][17]

已下载并只读检查官方附件 `ros2_imu.zip`。[18] 归档 SHA256：`cf02cc6d8c01f01facb8d54fbbbb85a8a357d260bb546ea02781e9f1423dbecf`。其中 `wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py` 第 29–78 行的解析规则为：11 字节帧，首字节 `0x55`，`0x51` 为加速度、`0x52` 为角速度、`0x53` 为欧拉角、`0x54` 为磁场；前 10 字节之和取低 8 位与第 11 字节校验。解析代码按有符号 16 位值换算，加速度量程 ±16g、角速度量程 ±2000°/s、角度量程 ±180°。这些是**所检查示例采用的协议**；正式驱动仍须用实机帧验证。[18]

此附件不适合原样用于 EKF，已核实的问题包括：[18]

1. `handle_serial_data()`（驱动第 45、51 行）已把加速度转换为 `m/s²`、角速度转换为 `rad/s`；`imu_data()`（第 182–192 行）又乘量程比例并对角速度再次转弧度，形成重复换算。对该版本必须修正后才能使用数值。
2. `serial.Serial()`（驱动第 153 行）硬编码 `/dev/imu_usb` 和 9600，读取 `port`/`baud` 参数的代码在第 142–143 行被注释。因此只改 launch 参数不足以改变连接配置。
3. launch 第 8–9、19 行使用旧 `node_executable`、`node_name` 写法；包元数据没有完整声明 `sensor_msgs`、`pyserial`、`numpy` 等实际依赖，需要适配 Humble。
4. 没有填测量协方差；姿态虽然存在却发在 `data_raw`。需要明确姿态是否有效及其协方差，不应靠话题名称猜测字段内容。
5. 收到有效 `0x53` 角度帧才触发整条 IMU 消息发布；若只让模块输出角速度帧，原示例不会发布。改为只用角速度时，发布触发及时间戳逻辑也要相应改造。
6. 串口异常后退出读取线程，没有重连流程；接入项目时需补充断流状态和恢复处理。

上述缺陷仅针对该网上 ZIP。用户本地 `imu_usb` 已有正确的单位换算、Humble 参数和协方差，
当前建议修整本地包的数据新鲜度、配置确认和 TF 设置后复用，详见[本地驱动核对](imu-local-driver-review.md)。
此次只审阅，没有安装驱动、执行厂商主程序或改动实机配置。

## 官方来源

1. [robot_localization Humble：Configuring robot_localization](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst)
2. [robot_localization Humble：State estimation nodes](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/state_estimation_nodes.rst)
3. [robot_localization Humble：Preparing sensor data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)
4. [ROS 2 Humble：sensor_msgs/msg/Imu.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)
5. [REP-103：单位与坐标系规范](https://www.ros.org/reps/rep-0103.html)
6. [REP-145：IMU 驱动约定](https://www.ros.org/reps/rep-0145.html)
7. [robot_localization Humble：EKF 测量方差处理](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ekf.cpp)
8. [slam_toolbox Humble README](https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md)、[订阅与 TF 实现](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)
9. [Nav2 Humble：AMCL 节点](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp)
10. [AMCL 运动模型插件表](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/plugins.xml)、[OmniMotionModel 实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/motion_model/omni_motion_model.cpp)
11. [Nav2 Humble：controller_server](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)、[OdomSubscriber](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp)
12. [Nav2 Humble：velocity_smoother](https://github.com/ros-navigation/navigation2/blob/humble/nav2_velocity_smoother/src/velocity_smoother.cpp)、[OdomSmoother](https://github.com/ros-navigation/navigation2/blob/humble/nav2_util/src/odometry_utils.cpp)
13. [REP-105：坐标系及其发布职责](https://www.ros.org/reps/rep-0105.html)
14. [robot_localization Humble：状态变量定义](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/include/robot_localization/filter_common.hpp)
15. [幻尔 IMU Module：User Manual](https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/1.User_Manual.html)
16. [幻尔 IMU：ROS Application（含 ROS 2）](https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/3.IMU_Application_Instructions-ROS_Application.html)
17. [幻尔 IMU：树莓派 USB/Python 例程](https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/2.IMU_Application_Instructions-Basic_Application.html#raspberry-pi-usb-application-routines)
18. [幻尔官方附件入口](https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/Appendix.html) → [Program Files](https://drive.google.com/drive/folders/1-9gHCo9N8SOaOJli2f6GnFgSXFPrG3Dr) → [6.ROS2 Application Program](https://drive.google.com/drive/folders/1AWsB-WSwWK3zLfkPwECs9csat4eT4c1l) → [ros2_imu.zip](https://drive.google.com/file/d/1nPopb8Opc5lc3DUoel-dN-ZwzcQlPJ7H/view)。源码路径：`wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py`、`wit_ros2_imu/launch/rviz_and_imu.launch.py`、`wit_ros2_imu/package.xml`。

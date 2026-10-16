# AlohaMini ROS 2 安装

项目运行在 ROS 2 Humble Docker 环境中。推荐的部署方式是：树莓派运行带 `/dev` 映射的
ROS2 容器，负责 C1、底盘、建图和导航；本机 PC 运行 `osrf/ros:humble-desktop`
容器，负责通过同一 ROS Domain 的网络接收话题并运行 RViz。

下面的命令分为树莓派宿主机和本机 PC 两部分。树莓派使用 ARM64 镜像，PC 使用
Humble Desktop 镜像；两台机器需要在同一网络，并使用相同的 `ROS_DOMAIN_ID=5`。

## 树莓派宿主机安装 Docker

在树莓派宿主机执行：

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"
newgrp docker
docker version
docker run --rm hello-world
```

如果系统已经安装 Docker，只需确认 `docker version` 和 `docker run --rm hello-world`
可以成功执行即可。

## 树莓派拉取 ROS2 镜像

树莓派只需要 ROS2 基础镜像；项目内的 C1 SDK 和 ROS2 节点会在工作区中编译：

```bash
docker pull --platform linux/arm64/v8 docker.io/library/ros:humble-ros-base-jammy
```

如果树莓派无法直接访问 Docker Hub，可以在可联网的电脑上准备 ARM64 镜像：

```bash
docker pull --platform linux/arm64/v8 docker.io/library/ros:humble-ros-base-jammy
docker save -o ros_humble_base_jammy_arm64.tar \
  docker.io/library/ros:humble-ros-base-jammy
scp ros_humble_base_jammy_arm64.tar <pi_user>@<pi_ip>:~/
```

再在树莓派导入：

```bash
docker load -i ~/ros_humble_base_jammy_arm64.tar
```

## USB 串口

配置别名，必须要在创建容器之前执行，否则在容器内将会无法查询到配置别名之后的雷达和imu
先在运行 Docker 的宿主机确认设备：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
lsusb
```

驱动和 launch 的默认串口是 `/dev/ttyUSB0`。这个编号会随 USB 插拔顺序变化，建议给
C1 配置固定别名 `/dev/lidar_c1`，再在 launch 中使用该别名。

### 配置 `/dev/lidar_c1` udev 别名

先只连接 C1，确认当前设备节点和 USB 信息：

```bash
ls -l /dev/ttyUSB*
udevadm info --query=all --name=/dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
```

仓库已经提供常见 C1 CP210x 转换器的规则模板。在运行 Docker 的宿主机上：
`ros2_ws/src/alohamini_rplidar_c1/udev/99-alohamini-rplidar-c1.rules`。在树莓派宿主机执行：

```bash
sudo cp ~/alohamini_lidar_imu/ros2_ws/src/alohamini_rplidar_c1/udev/99-alohamini-rplidar-c1.rules \
  /etc/udev/rules.d/99-alohamini-rplidar-c1.rules
```

多个相同 USB 转串口设备之间串线。先读取 C1 序列号：

```bash
udevadm info --attribute-walk --name=/dev/ttyUSB0 \
  | awk -F'"' '/ATTRS{serial}/{print $2; exit}'
```

编辑刚才复制的规则文件，把 `<c1_serial>` 替换成实际输出的序列号。规则中的完整行应为：

```text
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", ATTRS{serial}=="从 udevadm 输出复制的序列号", MODE:="0660", GROUP="dialout", SYMLINK+="lidar_c1"
```

如果 `udevadm` 没有输出 `ATTRS{serial}`，不要使用仅按 `10c4:ea60` 匹配的规则，
否则其他 CP210x 设备可能占用 `/dev/lidar_c1`；此时应使用硬件提供的唯一序列号，
或临时继续使用 `/dev/ttyUSB0`。

修改规则后重新加载并触发：

```bash
sudo usermod -aG dialout "$USER"  
# 当前用户尚未在 dialout 组时执行，重新登录后生效
sudo udevadm control --reload-rules
sudo udevadm trigger
# 查询雷达别名是否生效
ls -l /dev/lidar_c1
```

确认别名指向 C1 后，单独测试和完整 bringup 都传入同一个参数：

```bash
ros2 launch alohamini_rplidar_c1 test_rplidar_c1.launch.py \
  serial_port:=/dev/lidar_c1
ros2 launch alohamini_bringup mapping_ros2_control.launch.py \
  serial_port:=/dev/ttyACM0 lidar_serial_port:=/dev/lidar_c1
```

如果尚未安装 udev 规则，仍可直接使用默认的 `/dev/ttyUSB0`。

### 配置 `/dev/imu` udev 别名

只连接imu，用以下命令查询信息

```bash
udevadm info --query=all --name=/dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
```

假如查询出来的内容为：

```bash
E: ID_MODEL_ID=7523
E: ID_VENDOR_ID=1a86
```

在 `/etc/udev/rules.d/99-alohamini-rplidar-c1.rules`追加一行，绑定imu规则

```bash
sudo nano /etc/udev/rules.d/99-alohamini-rplidar-c1.rules
```

粘贴以下内容，idVendor与idProduct替换成查询到的数字，将第一步ID_MODEL_ID值填入idProduct，将ID_VENDOR_ID填入idVendor

```bash
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="idVendor", ATTRS{idProduct}=="idProduct", MODE:="0660", GROUP="dialout", SYMLINK+="imu"
```

修改规则后重新加载并触发：

```bash
sudo usermod -aG dialout "$USER"  
# 当前用户尚未在 dialout 组时执行，重新登录后生效
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 创建树莓派 ROS2 容器

在树莓派宿主机的项目目录执行。`/dev:/dev` 让容器访问 C1 和底盘串口，
`--net=host` 让本机 PC 的 RViz 可以通过 DDS 接收话题：

```bash
cd ~/alohamini_lidar_imu
docker rm -f alohamini_nav2 2>/dev/null || true

docker run -dit \
  --platform linux/arm64/v8 \
  --net=host \
  --ipc=host \
  --privileged \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -e ROS_DOMAIN_ID=5 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$HOME/alohamini_lidar_imu/ros2_ws:/root/ws" \
  --name alohamini_nav2 \
  docker.io/library/ros:humble-ros-base-jammy \
  bash
```

进入容器：

```bash
docker exec -it alohamini_nav2 bash
```

需要头部摄像头时，在树莓派容器内确认设备可见：

```bash
ls -l /dev/am_camera_forward
```

若使用其他设备路径，在启动命令中指定对应相机参数，查看步骤见
[WORKFLOW.md](WORKFLOW.md#41-查看头部摄像头)。

## 在树莓派容器内安装 ROS2 依赖并构建

```bash
apt-get update
apt install -y --no-install-recommends \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-collision-monitor ros-humble-nav2-lifecycle-manager \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-realtime-tools ros-humble-tf2-ros ros-humble-tf2-tools \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-image-transport-plugins \
  ros-humble-rmw-fastrtps-cpp libopencv-dev \
  ros-humble-ament-cmake-google-benchmark ros-humble-ament-lint-common \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-gen-version-h \
  python3-colcon-common-extensions python3-zmq python3-serial iproute2 iputils-ping net-tools \
  ros-humble-cv-bridge \
  ros-humble-transmission-interface ros-humble-pluginlib \
  ros-humble-tf2-geometry-msgs ros-humble-imu-tools ros-humble-diagnostic-updater \
  ros-humble-rosbag2 python3-rosdep

```

构建工作空间：

```bash
cd /root/ws
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
source install/setup.bash
```

以后重启树莓派或容器时，只需要：

```bash
docker start alohamini_nav2
docker exec -it alohamini_nav2 bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
```

## 本机 PC 的 RViz/C1 Docker 容器

本机 PC 使用 `docker.io/osrf/ros:humble-desktop` 运行 RViz，通过 host 网络接收树莓派
容器发布的 `/scan_filtered`、`/map`、TF、Nav2 路径和相机话题。PC 和树莓派必须在
同一局域网，并使用相同的 `ROS_DOMAIN_ID=5`。建图和导航的实际启动命令在
[WORKFLOW.md](WORKFLOW.md)；建图时键盘会话在树莓派容器中运行，PC 容器负责 RViz 显示。

先在 PC 宿主机拉取镜像：

```bash
docker pull docker.io/osrf/ros:humble-desktop
```

本机的 `alohamini_rviz_humble` 容器需要在创建时映射宿主机 `/dev`，运行中的
容器不能临时增加这个挂载。因此如果该容器以前没有映射 `/dev`，需要按下面命令
用同名容器重新创建；工作区在宿主机，代码不会丢失，但容器内通过 `apt` 安装的
软件包需要重新安装。只查看树莓派数据时并不需要 PC 的 `/dev`，保留映射是为了让
同一个容器也能在 C1 插到 PC 时运行独立雷达测试。

在宿主机项目根目录执行：

```bash
cd /home/yigeoooo/project/alohamini_lidar_imu
xhost +local:root  # 如果本机 X11 已允许 root，可跳过
docker rm -f alohamini_rviz_humble 2>/dev/null || true

docker run -dit \
  --name alohamini_rviz_humble \
  --net=host \
  --ipc=host \
  --privileged \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD/ros2_ws:/root/ws" \
  -e DISPLAY="${DISPLAY:-:1}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=5 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  docker.io/osrf/ros:humble-desktop \
  bash
```

进入容器安装本项目和 RViz 所需依赖。只需首次创建容器时执行：

```bash
docker exec -it alohamini_rviz_humble bash
apt-get update
apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions python3-zmq libopencv-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-rviz-plugins ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-realtime-tools \
  ros-humble-tf2-ros ros-humble-tf2-tools \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-image-transport-plugins \
  ros-humble-rmw-fastrtps-cpp iproute2 iputils-ping net-tools \
  ros-humble-ament-cmake-google-benchmark ros-humble-ament-lint-common \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-gen-version-h

source /opt/ros/humble/setup.bash
cd /root/ws
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
source install/setup.bash
```

整个工作区会一起编译项目节点及内置的 `tf2` / `tf2_ros` 依赖。树莓派和 PC
分别在各自容器中编译；每个运行终端都依次 source ROS 2 和工作区的 `setup.bash`。

确认容器已经看到雷达：

```bash
ls -l /dev/ttyUSB* /dev/lidar_c1
```

如果没有配置 udev 别名，使用 `/dev/ttyUSB0`；如果已配置序列号别名，使用
`/dev/lidar_c1`。测试容器必须在雷达插入后启动，或者重新插拔雷达 USB 线。

确认当前终端使用工作区：

```bash
ros2 pkg prefix alohamini_bringup
ros2 pkg prefix tf2
ros2 pkg prefix tf2_ros
```

路径应位于 `/root/ws/install/` 下。随后按 [WORKFLOW.md](WORKFLOW.md) 启动应用。

## 首次检查

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 launch alohamini_rplidar_c1 test_rplidar_c1.launch.py \
  serial_port:=/dev/ttyUSB0
```

随后确认：

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

测试节点会报告 frame、点数、有效点数、频率、角度范围和距离范围。

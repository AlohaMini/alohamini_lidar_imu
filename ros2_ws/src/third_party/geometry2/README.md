# TF2 0.25.24（ROS 2 Humble）

本目录包含 ROS 官方 geometry2 **0.25.24** 的 `tf2`、`tf2_ros` 两个源码包及其测试。
源码未经本项目修改，保留上游 BSD 许可证及各文件版权声明。

- 上游：https://github.com/ros2/geometry2/tree/0.25.24
- 源码包：https://github.com/ros2/geometry2/archive/refs/tags/0.25.24.tar.gz
- 源码包 SHA-256：`9fb5244616d06029bd7510a52e2fd5ab6f9aae6995e1d8bcda14fbe5140b979a`

## 构建

依赖安装见 [INSTALL.md](../../../../docs/INSTALL.md)。树莓派和 PC 分别在各自
ROS Humble 容器中编译整个工作区：

```bash
source /opt/ros/humble/setup.bash
cd /root/ws
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
source install/setup.bash
```

只编译本目录的两个包时，在构建命令末尾追加 `--packages-select tf2 tf2_ros`。
树莓派内存较小时可用 `-j1`。如果 colcon 提示允许覆盖系统包，按提示追加
`--allow-overriding tf2 tf2_ros`。

每个运行终端都需要 source `/root/ws/install/setup.bash`。
`ros2 pkg prefix tf2` 和 `ros2 pkg prefix tf2_ros` 应分别返回
`/root/ws/install/tf2` 与 `/root/ws/install/tf2_ros`。

## 功能测试

在已完成构建并 source 工作区的容器内执行：

```bash
cd /root/ws
colcon test --packages-select tf2 tf2_ros --event-handlers console_direct+ \
  --ctest-args -R '^(test_|tf2_ros_test_)' --timeout 90
colcon test-result --test-result-base build/tf2 --verbose
colcon test-result --test-result-base build/tf2_ros --verbose
```

# AlohaMini RPLIDAR C1 ROS 2 adapter

This package contains a self-contained ROS 2 adapter for the Slamtec RPLIDAR C1.
The SDK is copied into `sdk/` from the supplied `rplidar_ros` example so this
repository does not depend on that checkout at build or runtime.

The driver publishes `sensor_msgs/msg/LaserScan` on `/scan`. Defaults match the
C1 example: `/dev/ttyUSB0`, `460800` baud, `Standard` scan mode, compensation
enabled, and 10 Hz. The frame defaults to `laser_frame` to match this robot's URDF.
For a stable USB name, install `udev/99-alohamini-rplidar-c1.rules` on the host
and pass `serial_port:=/dev/lidar_c1`; the full installation procedure is in
`../../../docs/INSTALL.md`.

## Basic smoke test

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 launch alohamini_rplidar_c1 test_rplidar_c1.launch.py \
  serial_port:=/dev/ttyUSB0
```

The scan test reports the message frame, point count, valid range count,
scan rate, angle range, and observed range once per second. It fails with exit code
1 if no valid scan arrives within the default 10-second startup window; set
`startup_timeout_sec:=0` to disable that gate. In another terminal:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

The default robot configuration uses an upright C1 with its cable toward the rear
and `inverted=false`. This driver publishes the full scan. The mapping and
navigation bringup selects the front 180-degree sector on `/scan_filtered` and
loads the sensor transform from the robot URDF.

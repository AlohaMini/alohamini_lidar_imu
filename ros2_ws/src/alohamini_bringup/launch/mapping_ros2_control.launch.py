# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

# SLAM mapping using the native C++ ros2_control base driver
# (alohamini_base_control) instead of the ZMQ bridge. Mirrors mapping.launch.py but
# swaps the sensors layer for sensors_ros2_control.launch.py.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    base_yaw_deg = LaunchConfiguration("base_yaw_deg")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_serial_baudrate = LaunchConfiguration("lidar_serial_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")
    lidar_scan_frequency = LaunchConfiguration("lidar_scan_frequency")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    slam_params_file = LaunchConfiguration("slam_params_file")
    enable_head_camera = LaunchConfiguration("enable_head_camera")
    head_camera_device = LaunchConfiguration("head_camera_device")
    head_camera_image_topic = LaunchConfiguration("head_camera_image_topic")
    head_camera_frame_id = LaunchConfiguration("head_camera_frame_id")
    head_camera_fps = LaunchConfiguration("head_camera_fps")
    head_camera_width = LaunchConfiguration("head_camera_width")
    head_camera_height = LaunchConfiguration("head_camera_height")
    head_camera_jpeg_quality = LaunchConfiguration("head_camera_jpeg_quality")

    sensors_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "launch", "sensors_ros2_control.launch.py"]
    )
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "config", "slam_toolbox.yaml"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch),
        launch_arguments={
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "use_mock_hardware": use_mock_hardware,
            "base_yaw_deg": base_yaw_deg,
            "lidar_serial_port": lidar_serial_port,
            "lidar_serial_baudrate": lidar_serial_baudrate,
            "lidar_frame_id": lidar_frame_id,
            "lidar_scan_mode": lidar_scan_mode,
            "lidar_scan_frequency": lidar_scan_frequency,
            "lidar_inverted": lidar_inverted,
            "lidar_angle_compensate": lidar_angle_compensate,
            "enable_head_camera": enable_head_camera,
            "head_camera_device": head_camera_device,
            "head_camera_image_topic": head_camera_image_topic,
            "head_camera_frame_id": head_camera_frame_id,
            "head_camera_fps": head_camera_fps,
            "head_camera_width": head_camera_width,
            "head_camera_height": head_camera_height,
            "head_camera_jpeg_quality": head_camera_jpeg_quality,
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial bus with the base wheels (motor IDs 8/9/10).",
            ),
            DeclareLaunchArgument("baud_rate", default_value="1000000"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("base_yaw_deg", default_value="90.0"),
            DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar_serial_baudrate", default_value="460800"),
            DeclareLaunchArgument("lidar_frame_id", default_value="laser_frame"),
            DeclareLaunchArgument("lidar_scan_mode", default_value="Standard"),
            DeclareLaunchArgument("lidar_scan_frequency", default_value="10.0"),
            DeclareLaunchArgument("lidar_inverted", default_value="false"),
            DeclareLaunchArgument("lidar_angle_compensate", default_value="true"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            DeclareLaunchArgument(
                "enable_head_camera",
                default_value="false",
                description="Publish the head camera stream (disabled by default to reduce DDS/Wi-Fi load).",
            ),
            DeclareLaunchArgument("head_camera_device", default_value="/dev/am_camera_forward"),
            DeclareLaunchArgument("head_camera_image_topic", default_value="/head_camera/image_raw"),
            DeclareLaunchArgument("head_camera_frame_id", default_value="head_camera"),
            DeclareLaunchArgument("head_camera_fps", default_value="10.0"),
            DeclareLaunchArgument("head_camera_width", default_value="640"),
            DeclareLaunchArgument("head_camera_height", default_value="480"),
            DeclareLaunchArgument("head_camera_jpeg_quality", default_value="70"),
            sensors,
            slam,
        ]
    )

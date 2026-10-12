"""Replay a measured front wall through the shipped C1 TF and sensor launch files."""
import json
import math
from pathlib import Path
import runpy
import statistics
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
import pytest
import xacro


PACKAGE = Path(__file__).resolve().parents[1]
SENSOR_LAUNCHES = ("sensors_ros2_control.launch.py", "sensors_bridge.launch.py")


def front_wall():
    capture = json.loads((PACKAGE / "test/fixtures/c1_front_wall.json").read_text())
    return [
        (capture["angle_min"] + index * capture["angle_increment"], distance)
        for index, distance in capture["wall_samples"]
    ]


def sensor_launch(filename):
    description = runpy.run_path(str(PACKAGE / "launch" / filename))[
        "generate_launch_description"
    ]()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return description, context


def test_measured_front_wall_is_in_front_of_base_footprint():
    base = Path(get_package_share_directory("alohamini_base_control"))
    robot = ET.fromstring(xacro.process_file(
        str(base / "urdf/alohamini_base.ros2_control.xacro")
    ).toxml())
    yaw = sum(
        float(robot.find(f"joint[@name='{name}']/origin").attrib["rpy"].split()[2])
        for name in ("base_footprint_joint", "base_link_to_laser_frame")
    )
    # Fit the measured wall's normal so range noise and the +/-pi seam do not
    # make the test depend on one beam. Translation does not change its normal.
    points = [(r * math.cos(a + yaw), r * math.sin(a + yaw)) for a, r in front_wall()]
    mx, my = (statistics.mean(p[i] for p in points) for i in (0, 1))
    xx = statistics.mean((x - mx) ** 2 for x, y in points)
    yy = statistics.mean((y - my) ** 2 for x, y in points)
    xy = statistics.mean((x - mx) * (y - my) for x, y in points)
    normal = 0.5 * math.atan2(2 * xy, xx - yy) + math.pi / 2
    if mx * math.cos(normal) + my * math.sin(normal) < 0:
        normal += math.pi
    normal = math.atan2(math.sin(normal), math.cos(normal))
    assert abs(normal) < math.radians(5), (
        f"Physical front wall appears at {math.degrees(normal):+.2f} deg in base_footprint"
    )


@pytest.mark.parametrize("filename", SENSOR_LAUNCHES)
def test_front_sector_preserves_both_sides_of_measured_wall(filename):
    _, context = sensor_launch(filename)
    sector = SimpleNamespace(
        min_angle=float(context.launch_configurations["scan_min_angle"]),
        max_angle=float(context.launch_configurations["scan_max_angle"]),
    )
    keep = runpy.run_path(str(PACKAGE / "scripts/scan_sector_filter"))[
        "ScanSectorFilter"
    ].keep_angle
    assert all(keep(sector, angle) for angle, _ in front_wall()), (
        "The configured sector cuts away part of the physical front wall"
    )
    assert not keep(sector, 0.0), "The rear-facing zero-degree ray must be excluded"


@pytest.mark.parametrize("filename", SENSOR_LAUNCHES)
def test_lidar_publishes_filter_input_without_overwriting_filter_output(filename):
    description, context = sensor_launch(filename)
    # Non-default names catch accidental hard-coding as well as the original
    # collision between the parent's and the C1 child's 'scan_topic' arguments.
    context.launch_configurations["scan_input_topic"] = "/test/c1_raw"
    context.launch_configurations["scan_topic"] = "/test/c1_filtered"
    driver_topics = []

    def visit(actions, in_lidar=False):
        for action in actions:
            if isinstance(action, Node):
                if in_lidar:
                    driver_topics.append(context.launch_configurations["scan_topic"])
            elif isinstance(action, IncludeLaunchDescription):
                source = action.launch_description_source
                source.get_launch_description(context)
                if source.location.endswith("/rplidar_c1.launch.py"):
                    visit(action.execute(context) or [], in_lidar=True)
            elif isinstance(action, LaunchDescription):
                visit(action.entities, in_lidar)
            elif isinstance(action, GroupAction):
                visit(action.execute(context) or [], in_lidar)
            else:
                visit(action.execute(context) or [], in_lidar)

    visit(description.entities)
    assert driver_topics == ["/test/c1_raw"]
    assert context.launch_configurations["scan_topic"] == "/test/c1_filtered"

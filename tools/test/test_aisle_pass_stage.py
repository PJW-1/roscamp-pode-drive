"""통로 교차 시연 준비 도구: Gazebo와 AMCL에 보내는 자세(ROS 2가 있을 때만)."""
import math
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("geometry_msgs")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import aisle_pass_stage as stage  # noqa: E402


def test_the_default_spot_is_the_shelf_side_of_the_south_aisle():
    """v7 남쪽 동쪽 진열대 남쪽 면에서 몸 옆면(반폭 0.056)을 3 cm 띄운 자리, 통로와 나란히."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc/fleet_core"))
    from fleet_core.fleet_map import load_fleet_map
    shelf = load_fleet_map("v7").sim_shelves[2]
    x, y, yaw = stage.DEFAULT_POSE
    assert stage.DEFAULT_ROBOT == "pode3"
    assert y == pytest.approx(shelf[1] - 0.03 - 0.056) and yaw == 0.0 and shelf[0] < x < shelf[2]


def test_the_gazebo_request_carries_the_name_place_and_heading():
    text = stage.pose_request("pode3", 1.4, 0.2615, 0.01, math.pi / 2)
    assert 'name: "pode3"' in text and "x: 1.4, y: 0.2615, z: 0.01" in text
    assert f"z: {math.sin(math.pi / 4)}, w: {math.cos(math.pi / 4)}" in text


def test_the_first_pose_for_amcl_is_in_the_map_frame_with_a_tight_spread():
    msg = stage.initial_pose(1.4, 0.2615, math.pi)
    assert msg.header.frame_id == "map"
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.4, 0.2615)
    assert msg.pose.pose.orientation.z == pytest.approx(1.0)
    assert msg.pose.covariance[0] == msg.pose.covariance[7] == 0.0004

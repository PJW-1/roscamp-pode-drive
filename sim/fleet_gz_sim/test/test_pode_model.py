"""바퀴 충돌 모양을 실제 폭의 원판으로 바꾸는 선택(pode_model)."""
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from fleet_gz_sim import pode_model

XACRO = Path(__file__).resolve().parents[1] / "urdf" / "pode_sim.urdf.xacro"
JOINT_Y, SHIFT_Y, BODY_HALF_M = 0.04055, 0.008, 0.056


def _urdf() -> str:
    try:
        return subprocess.run(["xacro", str(XACRO), "name:=pode1"], check=True,
                              capture_output=True, text=True).stdout
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"xacro or pinky_description is not here: {exc}")


def _wheel(root, name):
    link = next(link for link in root.iter("link") if link.get("name") == name)
    collision = link.find("collision")
    return collision.find("origin"), collision.find("geometry")


def test_the_maker_model_has_ball_wheels_wider_than_the_footprint():
    root = ET.fromstring(_urdf())
    for name in pode_model.WHEELS:
        origin, geometry = _wheel(root, name)
        ball = geometry.find("sphere")
        y = abs(float(origin.get("xyz").split()[1]))
        assert y == SHIFT_Y and float(ball.get("radius")) == 0.028
        assert JOINT_Y + y + float(ball.get("radius")) == pytest.approx(0.07655)


def test_disc_wheels_end_at_the_footprint_side():
    before = _urdf()
    root = ET.fromstring(pode_model.thin_wheels(before))
    for name in pode_model.WHEELS:
        origin, geometry = _wheel(root, name)
        disc = geometry.find("cylinder")
        assert geometry.find("sphere") is None and float(disc.get("radius")) == 0.028
        # 원통 축(z)을 바퀴 축(y)으로 눕힌다
        assert [float(v) for v in origin.get("rpy").split()] == pytest.approx([math.pi / 2, 0, 0])
        y = abs(float(origin.get("xyz").split()[1]))
        outer = JOINT_Y + y + float(disc.get("length")) / 2
        assert outer == pytest.approx(BODY_HALF_M, abs=1e-3)
    # 바퀴 충돌 모양 밖은 그대로다
    old = ET.fromstring(before)
    assert [link.get("name") for link in root.iter("link")] == [
        link.get("name") for link in old.iter("link")]
    assert len(list(root.iter("joint"))) == len(list(old.iter("joint")))
    assert len(list(root.iter("plugin"))) == len(list(old.iter("plugin")))


def test_a_model_without_ball_wheels_is_refused():
    with pytest.raises(ValueError):
        pode_model.thin_wheels("<robot><link name='l_wheel'/><link name='r_wheel'/></robot>")


def test_the_launch_default_keeps_the_maker_wheels():
    from fleet_gz_sim import launch_parts

    assert launch_parts.model.__defaults__[-1] == pode_model.SPHERE
    with pytest.raises(ValueError):
        launch_parts.model("pode1", 0, 0, 0, wheel_shape="cube")


def test_the_fleet_launch_picks_the_passing_values_by_the_cleaners():
    """청소 로봇이 있으면 원판 바퀴, 선 로봇 발자국, 띠 0.30 m. 없으면 예전 값. 준 값은 그대로 쓴다."""
    import importlib.util

    path = Path(__file__).resolve().parents[1] / "launch" / "gz.launch.py"
    spec = importlib.util.spec_from_file_location("gz_launch_for_pass", path)
    gz = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gz)
    assert [gz.ARGS[name][0] for name in gz.PASS_DEFAULTS] == ["", "", ""]
    assert [gz.pass_value(name, "", "pode3") for name in gz.PASS_DEFAULTS] == [
        "footprint", "0.30", "disc"]
    assert [gz.pass_value(name, " ", "") for name in gz.PASS_DEFAULTS] == ["disc", "0.0", "sphere"]
    assert gz.pass_value("wheel_shape", "sphere", "pode3") == "sphere"
    assert gz.pass_value("peer_lane_m", "0.2", "") == "0.2"
    # 관제가 띠를 피해 설 자리를 고를 때 쓰는 길이와 같다
    from fleet_core.common.params import load_params
    assert float(gz.PASS_DEFAULTS["peer_lane_m"][0]) == load_params().clean_tuck_lane_m

"""바퀴 충돌 모양을 얇은 원판으로 바꾸는 선택(pode_model)."""
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from fleet_gz_sim import pode_model

XACRO = Path(__file__).resolve().parents[1] / "urdf" / "pode_sim.urdf.xacro"
JOINT_Y, SHIFT_Y, BODY_HALF_M = 0.04055, 0.008, 0.056


def _urdf() -> str:
    """제조사 모델의 URDF. xacro나 pinky_description이 없는 곳에서만 건너뛰고, xacro가 실패하면 시험도 실패한다."""
    if shutil.which("xacro") is None:
        pytest.skip("xacro is not here")
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("pinky_description")
    except (ImportError, LookupError) as exc:
        pytest.skip(f"pinky_description is not here: {exc}")
    return subprocess.run(["xacro", str(XACRO), "name:=pode1"], check=True,
                          capture_output=True, text=True).stdout


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


def test_disc_wheels_touch_the_floor_where_the_drive_counts_and_stay_inside_the_footprint():
    before = _urdf()
    root = ET.fromstring(pode_model.thin_wheels(before))
    half = float(next(root.iter("wheel_separation")).text) / 2
    for name, side in zip(pode_model.WHEELS, (1, -1)):
        origin, geometry = _wheel(root, name)
        disc = geometry.find("cylinder")
        assert geometry.find("sphere") is None and float(disc.get("radius")) == 0.028
        # 원통 축(z)을 바퀴 축(y)으로 눕힌다
        assert [float(v) for v in origin.get("rpy").split()] == pytest.approx([math.pi / 2, 0, 0])
        centre = JOINT_Y + side * float(origin.get("xyz").split()[1])
        width = float(disc.get("length"))
        # 원판 어느 모서리가 바닥에 닿아도 제자리 회전의 주행 기록이 2 % 안에서 맞는다
        assert centre == pytest.approx(half) and width / 2 <= 0.02 * half
        assert centre + width / 2 < BODY_HALF_M
    # 바퀴 충돌 모양 밖은 그대로다
    old = ET.fromstring(before)
    assert [link.get("name") for link in root.iter("link")] == [
        link.get("name") for link in old.iter("link")]
    assert len(list(root.iter("joint"))) == len(list(old.iter("joint")))
    assert len(list(root.iter("plugin"))) == len(list(old.iter("plugin")))


def test_a_model_without_ball_wheels_is_refused():
    with pytest.raises(ValueError):
        pode_model.thin_wheels("<robot><link name='l_wheel'/><link name='r_wheel'/></robot>")


def _toy(left: str, right: str, separation: float = 0.0961, joint_y: float = 0.04055) -> str:
    """바퀴 둘과 구동 플러그인만 있는 모델. left, right는 그 바퀴 링크의 충돌 모양들."""
    ball = "<collision><origin xyz='0 {y} 0'/><geometry><sphere radius='0.028'/></geometry></collision>"
    parts = {"ball+": ball.format(y=0.008), "ball-": ball.format(y=-0.008), "": ""}
    links = "".join(f"<link name='{name}'>{''.join(parts[k] for k in kinds.split(','))}</link>"
                    for name, kinds in (("l_wheel", left), ("r_wheel", right)))
    joints = "".join(f"<joint name='{name}_joint' type='continuous'><origin xyz='0 {y} 0'/>"
                     f"<parent link='base_link'/><child link='{name}'/></joint>"
                     for name, y in (("l_wheel", joint_y), ("r_wheel", -joint_y if joint_y else 0)))
    return (f"<robot><link name='base_link'/>{links}{joints}<gazebo><plugin>"
            f"<wheel_separation>{separation}</wheel_separation></plugin></gazebo></robot>")


def test_wheels_whose_joints_sit_on_the_body_line_still_go_to_their_own_side():
    root = ET.fromstring(pode_model.thin_wheels(_toy("ball+", "ball-", joint_y=0.0)))
    ys = [float(_wheel(root, name)[0].get("xyz").split()[1]) for name in pode_model.WHEELS]
    assert ys == pytest.approx([0.04805, -0.04805])


def test_each_wheel_needs_its_own_ball():
    with pytest.raises(ValueError):
        pode_model.thin_wheels(_toy("ball+,ball+", ""))


def test_a_disc_outside_the_footprint_is_refused():
    with pytest.raises(ValueError, match="footprint"):
        pode_model.thin_wheels(_toy("ball+", "ball-", separation=0.112))


def test_a_model_without_the_drive_wheel_separation_is_refused():
    urdf = _urdf().replace("wheel_separation", "wheel_gap")
    with pytest.raises(ValueError, match="wheel_separation"):
        pode_model.thin_wheels(urdf)


def test_the_launch_default_keeps_the_maker_wheels():
    from fleet_gz_sim import launch_parts

    assert launch_parts.model.__defaults__[-1] == pode_model.SPHERE
    with pytest.raises(ValueError):
        launch_parts.model("pode1", 0, 0, 0, wheel_shape="cube")


def test_the_fleet_launch_picks_the_passing_values_by_the_cleaners():
    """청소 로봇이 있으면 원판 바퀴, 선 로봇 발자국, 띠 0.30 m, 실물에서 잰 라이다 잡음, 통로 스캔 띠. 없으면 예전 값. 준 값은 그대로 쓴다."""
    import importlib.util

    path = Path(__file__).resolve().parents[1] / "launch" / "gz.launch.py"
    spec = importlib.util.spec_from_file_location("gz_launch_for_pass", path)
    gz = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gz)
    assert [gz.ARGS[name][0] for name in gz.PASS_DEFAULTS] == ["", "", "", "", ""]
    assert [gz.pass_value(name, "", "pode3") for name in gz.PASS_DEFAULTS] == [
        "footprint", "0.30", "disc", "0.003", "true"]
    assert [gz.pass_value(name, " ", "") for name in gz.PASS_DEFAULTS] == [
        "disc", "0.0", "sphere", "0.02", "false"]
    # 청소 로봇이 없을 때의 라이다 잡음은 모델 기본값과 같다
    xacro = ET.parse(XACRO).getroot()
    arg = next(e for e in xacro.iter() if e.tag.endswith("arg") and e.get("name") == "lidar_noise")
    assert float(gz.PASS_DEFAULTS["lidar_noise"][1]) == float(arg.get("default"))
    assert gz.pass_value("lidar_noise", "0.01", "pode3") == "0.01"
    assert gz.pass_value("wheel_shape", "sphere", "pode3") == "sphere"
    assert gz.pass_value("peer_lane_m", "0.2", "") == "0.2"
    # 관제가 띠를 피해 설 자리를 고를 때 쓰는 길이와 같다
    from fleet_core.common.params import load_params
    assert float(gz.PASS_DEFAULTS["peer_lane_m"][0]) == load_params().clean_tuck_lane_m

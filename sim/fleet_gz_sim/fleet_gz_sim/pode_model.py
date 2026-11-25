"""Gazebo용 Pode 설명(URDF)을 낸다: python3 -m fleet_gz_sim.pode_model <sphere|disc> <xacro 인자...>

제조사 모델(pinky_description)의 바퀴 충돌 모양은 반지름 0.028 m 구다. 구는 몸 옆으로 0.0766 m까지 나와
발자국 반폭 0.056 m보다 2 cm 넓다. 그래서 두 로봇이 나란히 지날 때 몸 사이가 4.1 cm보다 좁으면 바퀴끼리 닿는다.
disc는 그 구를 얇은 원판으로 바꾸고, 원판을 구동 플러그인이 세는 바퀴 간격(wheel_separation)의 절반 자리에 둔다.
원통은 모서리로 바닥에 닿으므로, 원판이 두껍거나 그 자리에서 벗어나면 제자리 회전의 주행 기록이 그만큼 틀린다.
제조사 파일은 고치지 않는다.
"""
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

SPHERE, DISC = "sphere", "disc"
SHAPES = (SPHERE, DISC)
WHEELS = ("l_wheel", "r_wheel")
DISC_WIDTH_M = 0.0015  # 바퀴 축 방향 폭. 바닥에 닿는 자리가 이 폭 안에서만 움직인다
BODY_HALF_M = 0.056  # 발자국 반폭


def _joint_y(root, link_name: str) -> float:
    """링크를 몸에 다는 관절의 y(몸 기준)."""
    for joint in root.iter("joint"):
        child, origin = joint.find("child"), joint.find("origin")
        if child is not None and child.get("link") == link_name:
            if origin is not None and any(float(v) for v in origin.get("rpy", "0 0 0").split()):
                raise ValueError(f"the joint of {link_name} is turned")
            return float(origin.get("xyz", "0 0 0").split()[1]) if origin is not None else 0.0
    raise ValueError(f"no joint holds {link_name}")


def _ball(root, link_name: str):
    """바퀴 링크의 구 충돌 모양 하나: (collision, sphere). 하나가 아니면 ValueError."""
    found = []
    for link in root.iter("link"):
        if link.get("name") != link_name:
            continue
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            ball = geometry.find("sphere") if geometry is not None else None
            if ball is not None:
                found.append((collision, ball))
    if len(found) != 1:
        raise ValueError(f"expected one sphere collision on {link_name}, found {len(found)}")
    return found[0]


def thin_wheels(urdf: str) -> str:
    """바퀴 링크의 구 충돌 모양을 같은 반지름의 얇은 원판(축은 바퀴 축)으로 바꾼 URDF."""
    root = ET.fromstring(urdf)
    balls = {name: _ball(root, name) for name in WHEELS}
    separation = next(root.iter("wheel_separation"), None)
    if separation is None:
        raise ValueError("no wheel_separation in the drive plugin")
    half = float(separation.text) / 2
    if half + DISC_WIDTH_M / 2 > BODY_HALF_M:
        raise ValueError(f"a disc at {half} m would stand outside the footprint ({BODY_HALF_M} m)")
    for name, (collision, ball) in balls.items():
        joint_y = _joint_y(root, name)
        geometry = collision.find("geometry")
        geometry.remove(ball)
        ET.SubElement(geometry, "cylinder", radius=ball.get("radius"), length=str(DISC_WIDTH_M))
        origin = collision.find("origin")
        if origin is None:
            origin = ET.SubElement(collision, "origin", xyz="0 0 0")
        x, old_y, z = origin.get("xyz").split()
        # 구가 있던 쪽(몸 기준)에 둔다
        side = joint_y + float(old_y)
        if side == 0.0:
            raise ValueError(f"cannot tell which side {name} is on")
        y = math.copysign(half, side) - joint_y
        origin.set("xyz", f"{x} {y:.6g} {z}")
        origin.set("rpy", f"{math.pi / 2} 0 0")
    return ET.tostring(root, encoding="unicode")


def main(argv=None) -> None:
    shape, *args = sys.argv[1:] if argv is None else argv
    if shape not in SHAPES:
        raise SystemExit(f"wheel shape must be one of {SHAPES}, got {shape!r}")
    urdf = subprocess.run(["xacro", *args], check=True, capture_output=True, text=True).stdout
    print(thin_wheels(urdf) if shape == DISC else urdf)


if __name__ == "__main__":
    main()

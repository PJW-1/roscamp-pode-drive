"""Gazebo용 Pode 설명(URDF)을 낸다: python3 -m fleet_gz_sim.pode_model <sphere|disc> <xacro 인자...>

제조사 모델(pinky_description)의 바퀴 충돌 모양은 반지름 0.028 m 구다. 구는 몸 옆으로 0.0766 m까지 나와
발자국 반폭 0.056 m보다 2 cm 넓다. 그래서 두 로봇이 나란히 지날 때 몸 사이가 4.1 cm보다 좁으면 바퀴끼리 닿는다.
disc는 그 구를 바깥 면이 발자국 반폭에 오는 원판으로 바꾼다. 제조사 파일은 고치지 않는다.
"""
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

SPHERE, DISC = "sphere", "disc"
SHAPES = (SPHERE, DISC)
WHEELS = ("l_wheel", "r_wheel")
DISC_WIDTH_M = 0.015  # 바퀴 축 방향 폭


def thin_wheels(urdf: str) -> str:
    """바퀴 링크의 구 충돌 모양을 같은 반지름의 원판(축은 바퀴 축)으로 바꾼 URDF."""
    root = ET.fromstring(urdf)
    changed = 0
    for link in root.iter("link"):
        if link.get("name") not in WHEELS:
            continue
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            ball = geometry.find("sphere") if geometry is not None else None
            if ball is None:
                continue
            geometry.remove(ball)
            ET.SubElement(geometry, "cylinder", radius=ball.get("radius"),
                          length=str(DISC_WIDTH_M))
            origin = collision.find("origin")
            if origin is None:
                origin = ET.SubElement(collision, "origin", xyz="0 0 0")
            origin.set("rpy", f"{math.pi / 2} 0 0")
            changed += 1
    if changed != len(WHEELS):
        raise ValueError(f"expected a sphere collision on each of {WHEELS}, changed {changed}")
    return ET.tostring(root, encoding="unicode")


def main(argv=None) -> None:
    shape, *args = sys.argv[1:] if argv is None else argv
    if shape not in SHAPES:
        raise SystemExit(f"wheel shape must be one of {SHAPES}, got {shape!r}")
    urdf = subprocess.run(["xacro", *args], check=True, capture_output=True, text=True).stdout
    print(thin_wheels(urdf) if shape == DISC else urdf)


if __name__ == "__main__":
    main()

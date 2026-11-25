"""서 있는 청소 로봇의 몸: 몸 폭에 맞춘 캡슐. 다른 로봇은 반지름 원이다. ROS를 쓰지 않는다."""
from __future__ import annotations

import math

from .geom import segment_gap

CLEANER_HALF_M = 0.056  # 청소 로봇 몸 반폭(폭 11.2 cm)


def standing_cleaner(r, cleaners) -> bool:
    """캡슐로 볼 로봇: 청소 로봇이고 goal이 없거나(run 없음) 고장 주입으로 멈췄다."""
    return r.name in cleaners and (r.run is None or r.stuck)


def spine(x, y, yaw, radius):
    """캡슐 뼈대: 중심에서 yaw 방향으로 ±(radius - 반폭)인 선분."""
    d = radius - CLEANER_HALF_M
    dx, dy = math.cos(yaw) * d, math.sin(yaw) * d
    return (x - dx, y - dy), (x + dx, y + dy)


def capsule_gap(p, bone) -> float:
    """점 p에서 캡슐 겉까지(안이면 음수)."""
    return segment_gap(p, *bone) - CLEANER_HALF_M


def body_gap(a, b, radius, cleaners) -> tuple[float, float]:
    """두 로봇 사이 (잴 거리, 이보다 작으면 닿음). 서 있는 청소 로봇이 있으면 그 캡슐에서
    상대 중심까지, 없으면 중심 거리."""
    for narrow, other in ((a, b), (b, a)):
        if standing_cleaner(narrow, cleaners):
            return capsule_gap(other.pose, spine(narrow.x, narrow.y, narrow.yaw, radius)), radius
    return math.dist(a.pose, b.pose), 2 * radius

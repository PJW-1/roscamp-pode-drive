"""평가 시나리오(시뮬레이터 없이): 로봇 0 의 만남을 라이다보다 intent 가 먼저 알려 주는 배치.

corner, tee, entry 후보는 좌표를 손으로 적지 않고 우리 전역 경로(장애물 없는 배치 0)와 HALL 그래프
경로(robot_spec.node_graph)의 기하로 고른다. 로봇 0 은 그래프 경로 a -> b 의 호장 s0 에서 출발하며,
s0 는 둘 다 같은 속도로 달리면 만남점에 같이 닿도록 정한다 (로봇 경로 길이 - s0 = 우리 경로 길이).
  corner: 우리 중심에서 로봇 중심과 로봇 박스 네 모서리로 가는 선분이 출발 때 선반에 막히고, 같이
          달리는 동안 중심 거리가 CLOSE_M 아래가 될 때까지 계속 막힘.
  tee:    만남점에서 두 진행 방향의 사잇각이 TEE_DEG 안 (옆에서 건넘).
  entry:  만남점에서 마주 오고 (방향 cos < ENTRY_COS), 만남점의 좌우 빈 폭이 PASSAGE_M 미만 (통로).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import torch
from arena_spec import raycast_rects
from robot_spec import ROBOT
from route_spec import point_rect_distance

SCENARIOS = ("corner", "tee", "entry")
# ---- 후보 판정 기준 (한 곳) ----
TOUCH_M = 0.03  # 우리 경로 점이 로봇 경로에서 이 이내면 두 경로가 만남
MIN_MEET_M = 0.35  # 우리 시작에서 만남점까지 최소 경로 길이
START_SEP_M = 0.30  # 두 시작점 중심 사이 최소 거리
START_CLEAR_M = 0.09  # 로봇 시작점과 벽/선반 사이 최소 거리
TEE_DEG = (60.0, 120.0)  # tee 만남점 사잇각 범위
ENTRY_COS = -0.7  # entry: 진행 방향 cos 상한 (마주 옴)
PASSAGE_M = 0.40  # entry: 만남점의 좌우 빈 폭 상한
# corner: 출발 때 막히고 중심 거리가 이 아래가 될 때까지 막힘. 4 cm 두께 선반은 0.5 m 안팎에서 로봇을
# 드러내므로 이 아레나에서 더 줄일 수 없음
CLOSE_M = 0.60
SAMPLE_M = 0.01  # 같이 달리기 검사의 한 걸음 [m]
HEADING_M = 0.05  # 만남점 진행 방향을 재는 앞뒤 거리
ROBOT_HALF = (0.5 * ROBOT.size[0], 0.5 * ROBOT.size[1])  # 스크립트 로봇 박스 반폭


@dataclass(frozen=True)
class Scenario:
    """한 시나리오의 후보 C 개 (같은 행끼리 한 후보)."""

    name: str
    route: np.ndarray  # (C,) 우리 경로 인덱스 (배치 0)
    a: np.ndarray  # (C,) 로봇 0 그래프 경로 시작 노드
    b: np.ndarray  # (C,) 끝 노드
    s0: np.ndarray  # (C,) 로봇 0 출발 호장 [m]
    meet: np.ndarray  # (C, 2) 만남점
    s_meet: np.ndarray  # (C,) 우리 시작에서 만남점까지 경로 길이 [m]

    @property
    def count(self) -> int:
        return len(self.route)


def arc_of(p: np.ndarray) -> np.ndarray:
    """폴리라인 (L, 2) 의 누적 호장 (L,)."""
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=-1))))


def point_at(p: np.ndarray, cum: np.ndarray, s) -> np.ndarray:
    """폴리라인의 호장 s (...) 점 (..., 2), 범위 밖은 끝점."""
    s = np.clip(np.asarray(s, dtype=float), 0.0, cum[-1])
    i = np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(p) - 2)
    seg = cum[i + 1] - cum[i]
    f = np.where(seg > 0, (s - cum[i]) / np.where(seg > 0, seg, 1.0), 0.0)
    return p[i] + f[..., None] * (p[i + 1] - p[i])


def heading_at(p: np.ndarray, cum: np.ndarray, s: float) -> np.ndarray:
    """호장 s 에서 진행 방향 단위 벡터 (2,)."""
    d = point_at(p, cum, s + HEADING_M) - point_at(p, cum, s - HEADING_M)
    return d / max(float(np.linalg.norm(d)), 1e-9)


def project(q: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """점 x (M, 2) 를 폴리라인 q (L, 2) 에 투영: (거리 (M,), q 의 호장 (M,))."""
    cum = arc_of(q)
    a, b = q[:-1], q[1:]
    ab = b - a
    t = ((x[:, None] - a) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12)
    t = np.clip(t, 0.0, 1.0)
    d = np.linalg.norm(x[:, None] - (a + t[..., None] * ab), axis=-1)  # (M, L - 1)
    j = d.argmin(-1)
    rows = np.arange(len(x))
    return d[rows, j], cum[j] + t[rows, j] * np.linalg.norm(ab[j], axis=-1)


def free_width(xy: np.ndarray, direction: np.ndarray, rects: np.ndarray) -> float:
    """xy 에서 진행 방향 좌우로 rect 까지 레이 거리의 합 [m] (arena_spec.raycast_rects)."""
    yaw = math.atan2(direction[1], direction[0])
    r = raycast_rects(
        torch.as_tensor(np.asarray(xy, dtype=np.float32)[None]), torch.tensor([yaw]),
        torch.as_tensor(rects, dtype=torch.float32), min_range=0.0, max_range=1e6, n_rays=4,
    )[0]  # 레이 -180, -90, 0, 90 deg
    return float(r[1] + r[3])


def box_corners(xy: np.ndarray, yaw: np.ndarray, half=ROBOT_HALF) -> np.ndarray:
    """yaw 로 돈 박스 중심 xy (T, 2) 의 네 모서리 (T, 4, 2)."""
    sx = np.array([1.0, 1.0, -1.0, -1.0]) * half[0]
    sy = np.array([1.0, -1.0, -1.0, 1.0]) * half[1]
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    return np.stack((xy[:, 0:1] + c * sx - s * sy, xy[:, 1:2] + s * sx + c * sy), -1)


def sight_blocked(src: np.ndarray, dst: np.ndarray, rects: np.ndarray) -> np.ndarray:
    """src (T, 2) 에서 dst (T, 2) 로 가는 선분이 rect 에 막히는지 (T,) (arena_spec.raycast_rects)."""
    d = dst - src
    dist = np.linalg.norm(d, axis=-1)
    yaw = np.arctan2(d[:, 1], d[:, 0])
    hit = raycast_rects(
        torch.as_tensor(src, dtype=torch.float32), torch.as_tensor(yaw, dtype=torch.float32),
        torch.as_tensor(rects, dtype=torch.float32), min_range=0.0, max_range=1e6, n_rays=2,
    )[:, 1].numpy()  # 레이 1 = yaw 방향
    return hit < dist


def robot_hidden(our: np.ndarray, rob: np.ndarray, rob_yaw: np.ndarray, shelves: np.ndarray):
    """(T,) 우리 중심에서 로봇 중심과 박스 네 모서리로 가는 선분이 모두 선반에 막힘."""
    ok = sight_blocked(our, rob, shelves)
    corners = box_corners(rob, rob_yaw)
    for k in range(4):
        ok &= sight_blocked(our, corners[:, k], shelves)
    return ok


def drive_together(p, q, s0: float, length: float):
    """우리는 p 의 0, 로봇은 q 의 s0 에서 같은 속도로 출발: SAMPLE_M 걸음마다 (우리 (T, 2), 로봇 (T, 2),
    로봇 yaw (T,)), 길이 length 까지. 로봇은 q 끝에서 멈춤."""
    s = np.arange(0.0, length + 1e-9, SAMPLE_M)
    cp, cq = arc_of(p), arc_of(q)
    ours = point_at(p, cp, s)
    rob = point_at(q, cq, s0 + s)
    d = point_at(q, cq, s0 + s + HEADING_M) - point_at(q, cq, s0 + s - HEADING_M)
    return ours, rob, np.arctan2(d[:, 1], d[:, 0])


def _meeting(p, q, kind: str, rects: np.ndarray):
    """우리 경로 p 와 로봇 경로 q 의 만남점 (우리 호장, 로봇 호장) 또는 None.
    tee/corner 는 첫 만남, entry 는 마주 오는 구간 중 좌우 폭이 가장 좁은 곳."""
    d, sq = project(q, p)
    touch = np.nonzero(d < TOUCH_M)[0]
    if len(touch) == 0:
        return None
    cp, cq = arc_of(p), arc_of(q)
    if kind != "entry":
        i = touch[0]
        return cp[i], sq[i]
    best = None
    for i in touch:
        u, w = heading_at(p, cp, cp[i]), heading_at(q, cq, sq[i])
        if float(u @ w) >= ENTRY_COS:
            continue
        width = free_width(p[i], u, rects)
        if width < PASSAGE_M and (best is None or width < best[0]):
            best = (width, cp[i], sq[i])
    return None if best is None else best[1:]


def candidate(p, q, kind: str, rects: np.ndarray, shelves: np.ndarray):
    """(우리 경로 p, 로봇 경로 q) 가 kind 시나리오면 (s0, 만남점, 우리 만남 호장), 아니면 None."""
    m = _meeting(p, q, kind, rects)
    if m is None:
        return None
    s_our, s_rob = m
    s0 = s_rob - s_our
    if s_our < MIN_MEET_M or s0 < 0.0:
        return None
    cp, cq = arc_of(p), arc_of(q)
    start = point_at(q, cq, s0)
    if np.linalg.norm(start - p[0]) < START_SEP_M:
        return None
    if point_rect_distance(start[0:1], start[1:2], rects)[0] < START_CLEAR_M:
        return None
    u, w = heading_at(p, cp, s_our), heading_at(q, cq, s_rob)
    ang = math.degrees(math.acos(float(np.clip(u @ w, -1.0, 1.0))))
    if kind == "tee" and not TEE_DEG[0] <= ang <= TEE_DEG[1]:
        return None
    if kind == "corner":
        ours, rob, ryaw = drive_together(p, q, s0, s_our)
        gap = np.linalg.norm(ours - rob, axis=-1)
        close = np.nonzero(gap < CLOSE_M)[0]
        upto = max(close[0] if len(close) else len(gap), 1)  # 출발 자세는 늘 검사
        if not robot_hidden(ours[:upto], rob[:upto], ryaw[:upto], shelves).all():
            return None
    return s0, point_at(p, cp, s_our), s_our


def build_scenario(kind: str, routes, graph, rects: np.ndarray, shelves: np.ndarray) -> Scenario:
    """모든 (우리 시작/목표 쌍, 로봇 그래프 경로 a -> b) 중 kind 후보."""
    if kind not in SCENARIOS:
        raise ValueError(f"robot_scenario {kind!r} is not one of {SCENARIOS}")
    rows = []
    k = len(graph.names)
    for pi in range(len(routes.pairs)):
        r = pi * routes.layouts  # 배치 0: 박스 없음
        p = routes.path[r, : routes.npts[r]]
        for a in range(k):
            for b in range(k):
                if a == b:
                    continue
                q = graph.path[a, b, : graph.npts[a, b]]
                c = candidate(p, q, kind, rects, shelves)
                if c is not None:
                    rows.append((r, a, b, c[0], c[1], c[2]))
    if not rows:
        raise ValueError(f"no {kind} scenario on this map")
    return Scenario(
        name=kind,
        route=np.array([x[0] for x in rows]),
        a=np.array([x[1] for x in rows]),
        b=np.array([x[2] for x in rows]),
        s0=np.array([x[3] for x in rows]),
        meet=np.array([x[4] for x in rows]),
        s_meet=np.array([x[5] for x in rows]),
    )


def scenario_names(setting: str) -> tuple[str, ...]:
    """ArenaSpec.robot_scenario (쉼표 구분, "" 은 없음) 를 이름 튜플로."""
    names = tuple(s.strip() for s in setting.split(",") if s.strip())
    bad = [s for s in names if s not in SCENARIOS]
    if bad:
        raise ValueError(f"robot_scenario {bad} not in {SCENARIOS}")
    return names


def pack(scenarios: list[Scenario], device="cpu") -> dict:
    """후보를 이어 붙인 텐서와 시나리오별 [시작, 끝) 행."""
    route = np.concatenate([s.route for s in scenarios])
    lo = np.cumsum([0] + [s.count for s in scenarios])
    return {
        "route": torch.as_tensor(route, device=device),
        "a": torch.as_tensor(np.concatenate([s.a for s in scenarios]), device=device),
        "b": torch.as_tensor(np.concatenate([s.b for s in scenarios]), device=device),
        "s0": torch.as_tensor(
            np.concatenate([s.s0 for s in scenarios]), dtype=torch.float32, device=device
        ),
        "lo": torch.as_tensor(lo[:-1], device=device),
        "count": torch.as_tensor(lo[1:] - lo[:-1], device=device),
    }


def sample(packed: dict, n: int, gen=None) -> torch.Tensor:
    """env n 개의 후보 행 (n,): 시나리오 균등, 그 안에서 후보 균등."""
    dev = packed["lo"].device
    k = len(packed["lo"])
    which = torch.randint(0, k, (n,), device=dev, generator=gen)
    u = torch.rand(n, device=dev, generator=gen)
    off = (u * packed["count"][which]).long().clamp(max=packed["count"][which] - 1)
    return packed["lo"][which] + off

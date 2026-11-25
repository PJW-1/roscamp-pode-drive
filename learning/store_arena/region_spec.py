"""아레나 영역(시뮬레이터 없이): 아레나 범위와 벽/선반 rect 만으로 통로, 틈, 열린 곳을 나눈다.

통로: 긴 두 옆이 선반 면이나 벽 면으로 막힌 빈 띠 (폭 AISLE_MAX_WIDTH_M 이하, 길이 AISLE_MIN_LENGTH_M 이상).
같은 폭 띠의 조각은 사이가 비어 있으면 한 통로로 잇고, 이은 자리의 면 쪽 빈 곳은 틈(opening)이다.
열린 곳: 나머지 빈 바닥의 연결 성분.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

AISLE_MAX_WIDTH_M = 0.45  # 이보다 넓은 띠는 통로가 아님
AISLE_MIN_LENGTH_M = 0.30  # 이보다 짧은 띠는 통로가 아님 (선반 끝과 벽 사이 등)
OPEN_CELL_M = 0.01  # 열린 곳을 나누는 격자
OPEN_MIN_CELLS = 25  # 이보다 작은 성분은 영역이 아님


@dataclass(frozen=True)
class Region:
    """영역 하나. rect 는 통로/틈의 띠, 열린 곳은 성분을 감싸는 rect."""

    kind: str  # "aisle", "opening", "open"
    rect: tuple[float, float, float, float]
    axis: int = -1  # 통로가 뻗는 축 (0: x, 1: y), 아니면 -1
    openings: tuple[tuple[float, float], ...] = ()  # 통로 면의 틈 구간 (뻗는 축 좌표)

    @property
    def width(self) -> float:
        a = 1 - self.axis
        return self.rect[a + 2] - self.rect[a]


@dataclass
class Regions:
    regions: list[Region]
    bounds: tuple[float, float, float, float]
    open_label: np.ndarray = field(repr=False)  # (nx, ny) 열린 곳 영역 인덱스, 아니면 -1

    def ids(self, kind: str) -> list[int]:
        return [i for i, r in enumerate(self.regions) if r.kind == kind]

    def index_of(self, xy) -> int:
        """점이 속한 영역 인덱스, 없으면 (장애물 안 등) -1. 통로와 틈이 열린 곳보다 먼저."""
        for i, r in enumerate(self.regions):
            if r.kind != "open" and _inside(xy, r.rect):
                return i
        nx, ny = self.open_label.shape
        ci = int((xy[0] - self.bounds[0]) / OPEN_CELL_M)
        cj = int((xy[1] - self.bounds[1]) / OPEN_CELL_M)
        if 0 <= ci < nx and 0 <= cj < ny:
            return int(self.open_label[ci, cj])
        return -1

    def rect_ids(self, rect) -> set[int]:
        """rect 가 걸친 영역 인덱스 모음."""
        out = {i for i, r in enumerate(self.regions)
               if r.kind != "open" and _overlap(np.asarray(rect), np.asarray(r.rect))}
        x0, y0 = self.bounds[:2]
        nx, ny = self.open_label.shape
        i0 = max(int(np.floor((rect[0] - x0) / OPEN_CELL_M)), 0)
        i1 = min(int(np.ceil((rect[2] - x0) / OPEN_CELL_M)), nx)
        j0 = max(int(np.floor((rect[1] - y0) / OPEN_CELL_M)), 0)
        j1 = min(int(np.ceil((rect[3] - y0) / OPEN_CELL_M)), ny)
        lab = self.open_label[i0:i1, j0:j1]
        return out | {int(v) for v in np.unique(lab) if v >= 0}


def _inside(xy, r) -> bool:
    return r[0] <= xy[0] <= r[2] and r[1] <= xy[1] <= r[3]


def _overlap(a, b) -> bool:
    return bool(a[0] < b[2] and a[2] > b[0] and a[1] < b[3] and a[3] > b[1])


def _free(strip, rects) -> bool:
    return not any(_overlap(strip, r) for r in rects)


def _strip(axis, lo_l, hi_l, lo_a, hi_a):
    """뻗는 축 구간 [lo_l, hi_l] x 가로 구간 [lo_a, hi_a] 의 rect."""
    if axis == 0:
        return np.array((lo_l, lo_a, hi_l, hi_a))
    return np.array((lo_a, lo_l, hi_a, hi_l))


# ---- 마주 보는 면 사이 띠 조각 ----
def _pieces(rects, axis, max_w):
    """(가로 lo, 가로 hi, 뻗는 lo, 뻗는 hi, 아래쪽 rect, 위쪽 rect) 조각들: 사이가 빈 마주 보는 면."""
    a, l = 1 - axis, axis
    out = []
    for i, ri in enumerate(rects):
        for j, rj in enumerate(rects):
            gap = rj[a] - ri[a + 2]
            lo, hi = max(ri[l], rj[l]), min(ri[l + 2], rj[l + 2])
            if not (0.0 < gap <= max_w and hi > lo):
                continue
            if _free(_strip(axis, lo, hi, ri[a + 2], rj[a]), rects):
                out.append((ri[a + 2], rj[a], lo, hi, i, j))
    return out


def build_regions(bounds, rects, max_w: float = AISLE_MAX_WIDTH_M,
                  min_len: float = AISLE_MIN_LENGTH_M) -> Regions:
    """bounds (x0, y0, x1, y1) 와 장애물 rects (B, 4, 벽 포함) 로 영역을 나눈다."""
    rects = np.asarray(rects, dtype=float)
    aisles, openings = [], {}
    # ---- 통로: 같은 가로 구간의 조각을 뻗는 축 순서로 잇기 ----
    for axis in (0, 1):
        a = 1 - axis
        groups: dict[tuple, list] = {}
        for p in _pieces(rects, axis, max_w):
            groups.setdefault((round(p[0], 6), round(p[1], 6)), []).append(p)
        for (lo_a, hi_a), ps in sorted(groups.items()):
            ps.sort(key=lambda p: p[2])
            runs = [[ps[0]]]
            for p in ps[1:]:
                q = runs[-1][-1]
                bridge = _strip(axis, q[3], p[2], lo_a, hi_a)
                if p[2] <= q[3] or (p[2] - q[3] <= max_w and _free(bridge, rects)):
                    runs[-1].append(p)
                else:
                    runs.append([p])
            for run in runs:
                lo_l, hi_l = run[0][2], max(p[3] for p in run)
                # 면으로 막힌 길이 (선반 끝 두 개와 벽 사이처럼 면이 짧으면 통로 아님)
                walled = sum(p[3] - p[2] for p in run)
                if hi_l - lo_l < min_len or walled < min_len:
                    continue
                spans = []
                for q, p in zip(run[:-1], run[1:]):
                    if p[2] <= q[3]:
                        continue
                    spans.append((float(q[3]), float(p[2])))
                    for side in (4, 5):  # 면 쪽 rect 가 바뀐 쪽이 틈
                        if q[side] == p[side]:
                            continue
                        rq, rp = rects[q[side]], rects[p[side]]
                        c0, c1 = max(rq[a], rp[a]), min(rq[a + 2], rp[a + 2])
                        o = tuple(float(v) for v in _strip(axis, q[3], p[2], c0, c1))
                        openings[tuple(round(v, 6) for v in o)] = (o, axis)
                aisles.append(Region("aisle", tuple(float(v) for v in
                                                    _strip(axis, lo_l, hi_l, lo_a, hi_a)),
                                     axis, tuple(spans)))
    regions = aisles + [Region("opening", o, ax) for o, ax in openings.values()]
    # ---- 열린 곳: 장애물, 통로, 틈을 뺀 빈 셀의 연결 성분 ----
    x0, y0, x1, y1 = bounds
    nx = int(np.ceil((x1 - x0) / OPEN_CELL_M - 1e-9))
    ny = int(np.ceil((y1 - y0) / OPEN_CELL_M - 1e-9))
    cx, cy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * OPEN_CELL_M,
                         y0 + (np.arange(ny) + 0.5) * OPEN_CELL_M, indexing="ij")
    free = np.ones((nx, ny), dtype=bool)
    for r in list(rects) + [np.array(g.rect) for g in regions]:
        free &= ~((cx >= r[0]) & (cx <= r[2]) & (cy >= r[1]) & (cy <= r[3]))
    lab, n = ndimage.label(free)
    open_label = np.full((nx, ny), -1)
    for k in range(1, n + 1):
        sel = lab == k
        if sel.sum() < OPEN_MIN_CELLS:
            continue
        idx = len(regions)
        open_label[sel] = idx
        xs, ys = cx[sel], cy[sel]
        h = 0.5 * OPEN_CELL_M
        regions.append(Region("open", (float(xs.min() - h), float(ys.min() - h),
                                       float(xs.max() + h), float(ys.max() + h))))
    return Regions(regions, tuple(float(v) for v in bounds), open_label)


def path_length_in(path: np.ndarray, rect) -> float:
    """경로 (L, 2) 중 중점이 rect 안인 구간 길이 합."""
    if len(path) < 2:
        return 0.0
    mid = 0.5 * (path[1:] + path[:-1])
    seg = np.hypot(*np.diff(path, axis=0).T)
    inside = ((mid[:, 0] >= rect[0]) & (mid[:, 0] <= rect[2])
              & (mid[:, 1] >= rect[1]) & (mid[:, 1] <= rect[3]))
    return float(seg[inside].sum())


def side_gaps(region: Region, box) -> tuple[float, float]:
    """통로 안 박스 rect 와 아래쪽 면, 위쪽 면 사이 가로 간격 [m]."""
    a = 1 - region.axis
    return box[a] - region.rect[a], region.rect[a + 2] - box[a + 2]


def passing_gap(regions: Regions, box, path: np.ndarray, half_w: float) -> float:
    """박스 rect 가 통로 안에 있고 경로 (L, 2) 의 통로 구간 몸체 띠(경로 +- half_w)에 걸치면 지나갈 틈
    (두 옆 간격 중 큰 쪽), 아니면 NaN."""
    c = (0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3]))
    k = regions.index_of(c)
    if k < 0 or regions.regions[k].kind != "aisle":
        return float("nan")
    reg = regions.regions[k]
    if not on_corridor(reg, box, path, half_w):
        return float("nan")
    return float(max(max(side_gaps(reg, box)), 0.0))


def on_corridor(region: Region, box, path: np.ndarray, half_w: float) -> bool:
    """통로 안 경로 점 중 하나라도 박스 rect 에서 half_w 보다 가까움 (로봇이 비켜야 함)."""
    r = region.rect
    p = path[(path[:, 0] >= r[0]) & (path[:, 0] <= r[2])
             & (path[:, 1] >= r[1]) & (path[:, 1] <= r[3])]
    if len(p) == 0:
        return False
    dx = np.maximum(np.maximum(box[0] - p[:, 0], 0.0), p[:, 0] - box[2])
    dy = np.maximum(np.maximum(box[1] - p[:, 1], 0.0), p[:, 1] - box[3])
    return bool(np.hypot(dx, dy).min() < half_w)

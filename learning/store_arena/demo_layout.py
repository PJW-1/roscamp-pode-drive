"""데모 장애물 배치(시뮬레이터 없이): 실제 데모처럼 영역마다 박스 하나, 통로 박스는 옆에 지나갈 틈을 남김.

route_spec.build_routes 가 obstacle_demo_share 비율의 배치를 이 규칙으로 만든다 (별도 난수 생성기).
"""

from __future__ import annotations

import math

import numpy as np

import region_spec as rg
import route_spec as rs


def is_demo(m: int, share: float) -> bool:
    """배치 m (1..) 이 데모 배치인지: 난수 없이 배치 번호로 share 비율을 고르게 고름. 배치 0 은 늘 빈 배치."""
    if m < 1 or share <= 0:
        return False
    return math.floor(m * share + 1e-9) > math.floor((m - 1) * share + 1e-9)


def half_width(spec) -> float:
    """몸체 띠 반폭: 우리 로봇 몸체 폭의 절반."""
    return 0.5 * (spec.footprint_y[1] - spec.footprint_y[0])


def aisle_box(rng, spec, region, path0, sizes, slots, fits, half_w):
    """통로 region 에 경로 옆 박스 하나: (슬롯, 중심) 또는 None.

    틈 g 를 obstacle_demo_gap_m 에서 균등으로 뽑고, 박스를 한쪽 면 쪽에 두어 반대쪽 면과의 간격이 g 가 되게 한다.
    다른 쪽 간격은 0..g (선반에 붙을 수 있음). 박스는 경로의 몸체 띠에 걸쳐야 하고 (region_spec.on_corridor)
    통로 길이 안, 면의 틈 구간 밖에 있어야 한다. 맞는 슬롯/쪽이 없으면 g 를 다시 뽑는다."""
    a, l = 1 - region.axis, region.axis
    r = region.rect
    w = region.width
    inside = path0[(path0[:, 0] >= r[0]) & (path0[:, 0] <= r[2])
                   & (path0[:, 1] >= r[1]) & (path0[:, 1] <= r[3])]
    if len(inside) == 0:
        return None
    for _ in range(50):
        g = rng.uniform(*spec.obstacle_demo_gap_m)
        s = inside[rng.integers(len(inside)), l]
        cands = []
        for slot in slots:
            side = sizes[slot]
            other = w - side - g
            if other < 0 or other > g:
                continue
            lo_l, hi_l = s - 0.5 * side, s + 0.5 * side
            if lo_l < r[l] or hi_l > r[l + 2]:
                continue
            if any(lo_l < o1 and hi_l > o0 for o0, o1 in region.openings):
                continue
            for lo_a in (r[a] + other, r[a] + g):  # 틈이 위쪽 면, 아래쪽 면
                box = rg._strip(region.axis, lo_l, hi_l, lo_a, lo_a + side)
                if rg.on_corridor(region, box, path0, half_w) and fits(box):
                    cands.append((slot, box))
        if cands:
            slot, box = cands[int(rng.integers(len(cands)))]
            return slot, np.array((0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])))
    return None


def demo_layout(rng, spec, regions, bounds, static_rects, path0, start, goal, sizes):
    """데모 배치: (박스 중심 (K, 2), NaN = 안 쓰는 슬롯; 통로 박스 슬롯, 없으면 -1).

    박스 수는 obstacle_demo_counts 에서 균등. 영역마다 박스 하나까지. 경로가 한 통로 안을
    obstacle_demo_aisle_min_m 이상 지나면 obstacle_demo_aisle_prob 로 그 통로에 aisle_box 하나.
    나머지는 지금 표본기의 경로 위 규칙 (obstacle_on_path_prob, obstacle_lateral_m) 으로 빈 영역에 둔다.
    모든 박스는 시작/목표에서 obstacle_node_clear_m 떨어지고 벽, 선반, 다른 박스와 안 겹친다."""
    k = len(sizes)
    out = np.full((k, 2), np.nan)
    lo, hi = spec.obstacle_demo_counts
    n_used = int(rng.integers(int(lo), int(hi) + 1))
    x0, y0, x1, y1 = bounds
    se = np.array([start, goal])
    taken: set[int] = set()
    aisle_slot = -1

    def fits(r) -> bool:
        if point_clear(r) < spec.obstacle_node_clear_m:
            return False
        others = [rs.box_rect(out[i], sizes[i]) for i in range(k) if not np.isnan(out[i, 0])]
        if rs.rects_overlap(r, np.array(list(static_rects) + others)):
            return False
        return not (regions.rect_ids(r) & taken)

    def point_clear(r) -> float:
        return float(rs.point_rect_distance(se[:, 0], se[:, 1], r[None]).min())

    slots = list(rng.permutation(k))
    aisles = [i for i in regions.ids("aisle")
              if rg.path_length_in(path0, regions.regions[i].rect) >= spec.obstacle_demo_aisle_min_m]
    if n_used > 0 and aisles and rng.random() < spec.obstacle_demo_aisle_prob:
        reg = aisles[int(rng.integers(len(aisles)))]
        got = aisle_box(rng, spec, regions.regions[reg], path0, sizes, slots, fits,
                        half_width(spec))
        if got is not None:
            slot, xy = got
            out[slot] = xy
            aisle_slot = int(slot)
            taken |= regions.rect_ids(rs.box_rect(xy, sizes[slot]))
            slots.remove(slot)
            n_used -= 1
    seg = np.diff(path0, axis=0)
    cum = np.concatenate(([0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))))
    for slot in slots[:n_used]:
        side = sizes[slot]
        for _ in range(50):
            if rng.random() < spec.obstacle_on_path_prob and cum[-1] > 0:
                s = rng.uniform(0.0, cum[-1])
                j = min(int(np.searchsorted(cum, s)), len(path0) - 1)
                xy = path0[j] + rng.uniform(-spec.obstacle_lateral_m, spec.obstacle_lateral_m, 2)
            else:
                xy = np.array((rng.uniform(x0, x1), rng.uniform(y0, y1)))
            r = rs.box_rect(xy, side)
            if fits(r):
                out[slot] = xy
                taken |= regions.rect_ids(r)
                break
    return out, aisle_slot


def box_records(regions, obstacle_xy, sizes, path0, half_w, final=None):
    """한 배치의 슬롯별 (영역 인덱스, 지나갈 틈 [m]): 안 쓰는 슬롯은 (-1, NaN).
    틈은 빈 아레나 경로 path0 의 몸체 띠에 걸치고 박스를 피한 경로 final 이 통로 안에서 지나는 통로 박스만."""
    k = len(sizes)
    reg = np.full(k, -1)
    gap = np.full(k, np.nan)
    for i in range(k):
        if np.isnan(obstacle_xy[i, 0]):
            continue
        reg[i] = regions.index_of(obstacle_xy[i])
        gap[i] = rg.passing_gap(regions, rs.box_rect(obstacle_xy[i], sizes[i]), path0, half_w,
                                final)
    return reg, gap


def aisle_passed(regions, obstacle_xy, sizes, slot, final) -> bool:
    """슬롯 slot 의 통로 박스를 박스 피한 경로 final 이 그 통로 안에서 지나감."""
    box = rs.box_rect(obstacle_xy[slot], sizes[slot])
    reg = regions.regions[regions.index_of(obstacle_xy[slot])]
    return rg.passes_beside(reg, box, final)

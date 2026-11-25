"""데모 배치 테스트(시뮬 없이): share 0 은 지금 표본기와 같음, 데모 규칙의 성질."""

import dataclasses
import hashlib
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import demo_layout as dl  # noqa: E402
import route_spec as R  # noqa: E402

SPEC = dataclasses.replace(A.ARENA, obstacle_layouts=6)
# 데모 배치를 넣기 전 route_spec (커밋 9a09378a) 이 SPEC 으로 만든 값
OLD_OBSTACLE_SHA = "39de98466546282eb1bdeb7be21bf4779006d9e12b69b96f4cdbbc03334d824e"
OLD_TOTAL_SHA = "fd5e1ae53ea744d2e9e50053a6ba44418e09377b354d17b33bcd7c036e1a3b2c"
OLD_PATH_SHA = "7641da56e177b29040ed93fff8b5c9b52b81196dbb2ebc6c21894b4576546efa"
OLD_LAYOUT_1_2 = [
    [[0.8251299553916254, 0.4870311473832741], [0.1159647089142848, 0.3293947193347472],
     [1.2856048992964095, 0.6206086275759819]],
    [[1.2144224710349143, 0.6332380976810397], [0.9848969370654855, 0.518725916992835],
     [np.nan, np.nan]],
]


def _sha(a):
    a = np.round(np.nan_to_num(a, nan=-9.0), 9)
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


@pytest.fixture(scope="module")
def site_static():
    site = A.load_site(SPEC)
    return site, np.array([(b.x0, b.y0, b.x1, b.y1) for b in A.arena_boxes(SPEC, site)])


def _build(site_static, **over):
    site, static = site_static
    return R.build_routes(dataclasses.replace(SPEC, **over), site, static)


def _demo_rows(r):
    m = np.arange(r.count) % r.layouts
    return np.array([dl.is_demo(int(k), 1.0) for k in m])


def test_share_zero_is_the_old_sampler(site_static):
    """share 0: 배치, 경로, 길이가 데모 코드 전 표본기의 값과 같다 (keep_path 등 다른 데모 값과 무관)."""
    for over in ({}, {"obstacle_demo_keep_path": 0, "obstacle_demo_aisle_prob": 1.0}):
        r = _build(site_static, **over)
        assert _sha(r.obstacle_xy) == OLD_OBSTACLE_SHA
        assert _sha(r.total) == OLD_TOTAL_SHA
        assert _sha(r.path) == OLD_PATH_SHA
        np.testing.assert_array_equal(r.obstacle_xy[1:3], np.array(OLD_LAYOUT_1_2))


def test_is_demo_share():
    assert not any(dl.is_demo(m, 0.0) for m in range(100))
    assert not dl.is_demo(0, 1.0) and all(dl.is_demo(m, 1.0) for m in range(1, 100))
    assert sum(dl.is_demo(m, 0.25) for m in range(1, 97)) == 24


@pytest.fixture(scope="module")
def demo2(site_static):
    return _build(site_static, obstacle_demo_share=1.0)


@pytest.fixture(scope="module")
def aisle1(site_static):
    return _build(site_static, obstacle_demo_share=1.0, obstacle_demo_counts=(1, 1),
                  obstacle_demo_aisle_prob=1.0)


@pytest.fixture(scope="module")
def regions(site_static):
    import region_spec as rg

    site, static = site_static
    return rg.build_regions(site.bounds(), static)


def test_demo_counts_and_one_box_per_region(demo2, regions):
    r = demo2
    rows = _demo_rows(r)
    used = ~np.isnan(r.obstacle_xy[..., 0])
    n = used.sum(1)
    assert set(n[rows]) <= {0, 1, 2} and (n[rows] >= 1).mean() > 0.99
    assert {1, 2} <= set(n[rows])
    for i in np.flatnonzero(rows & (n == 2)):
        a, b = (regions.rect_ids(R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k]))
                for k in np.flatnonzero(used[i]))
        assert a and b and not (a & b)


def test_aisle_boxes_leave_a_drawn_passable_gap_and_block_the_corridor(aisle1, regions,
                                                                         site_static):
    """통로 박스: 틈은 gap 범위 안 (몸체 폭보다 넓음), 다른 쪽 간격은 0..틈, 빈 아레나 경로의 몸체 띠에 걸침,
    경로는 빈 아레나 경로 그대로 (keep_path), 시작/목표 간격과 겹침 없음."""
    import region_spec as rg

    _, static = site_static
    r = aisle1
    rows = _demo_rows(r)
    lo, hi = SPEC.obstacle_demo_gap_m
    half_w = dl.half_width(SPEC)
    body = SPEC.footprint_y[1] - SPEC.footprint_y[0]
    m = r.layouts
    n_aisle = 0
    for i in np.flatnonzero(rows):
        g = r.obstacle_gap[i]
        k = np.flatnonzero(np.isfinite(g))
        if len(k) == 0:
            continue
        n_aisle += 1
        k = int(k[0])
        assert lo - 1e-9 <= g[k] <= hi + 1e-9 and g[k] > body
        reg = r.regions[r.obstacle_region[i, k]]
        assert reg.kind == "aisle"
        box = R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k])
        gaps = rg.side_gaps(reg, box)
        assert max(gaps) == pytest.approx(g[k]) and min(gaps) >= -1e-9
        path0 = r.path[(i // m) * m, : r.npts[(i // m) * m]]  # 배치 0 = 빈 아레나 경로
        assert rg.on_corridor(reg, box, path0, half_w)
        assert np.array_equal(r.path[i, : r.npts[i]], path0)
        assert not R.rects_overlap(box, static)
        s, gl = r.start_goal(i)
        d = R.point_rect_distance(np.array([s[0], gl[0]]), np.array([s[1], gl[1]]), box[None])
        assert d.min() >= SPEC.obstacle_node_clear_m
        for o0, o1 in reg.openings:
            assert not (box[reg.axis] < o1 and box[reg.axis + 2] > o0)
    assert n_aisle > 0.5 * rows.sum()
    # 통로 박스는 경로가 통로를 obstacle_demo_aisle_min_m 이상 지나는 쌍마다 있다
    for i in np.flatnonzero(rows):
        path0 = r.path[(i // m) * m, : r.npts[(i // m) * m]]
        long_aisle = any(rg.path_length_in(path0, regions.regions[a].rect)
                         >= SPEC.obstacle_demo_aisle_min_m for a in regions.ids("aisle"))
        if not long_aisle:
            assert not np.isfinite(r.obstacle_gap[i]).any()


def test_gap_draws_cover_the_range(site_static):
    r = _build(site_static, obstacle_demo_share=1.0, obstacle_demo_counts=(1, 1),
               obstacle_demo_aisle_prob=1.0, obstacle_layouts=24)
    g = r.obstacle_gap[np.isfinite(r.obstacle_gap)]
    lo, hi = SPEC.obstacle_demo_gap_m
    assert g.min() < lo + 0.02 and g.max() > hi - 0.02
    # 고르게: 범위를 넷으로 나눈 칸마다 10 % 이상
    counts = np.histogram(g, bins=np.linspace(lo, hi, 5))[0]
    assert (counts > 0.1 * len(g)).all()


def test_keep_path_layouts_stay_solvable_without_the_aisle_box(demo2, site_static):
    """keep_path 1: 통로 박스 말고 다른 박스와 선반으로는 목표까지 계획 가능 (통로 박스 옆 틈은 몸체보다 넓음)."""
    _, static = site_static
    r = demo2
    grid = R.Grid(_REG_BOUNDS(site_static), SPEC.grid_m)
    checked = 0
    for i in np.flatnonzero(_demo_rows(r) & np.isfinite(r.obstacle_gap).any(1)):
        k = int(np.flatnonzero(np.isfinite(r.obstacle_gap[i]))[0])
        others = [R.box_rect(r.obstacle_xy[i, j], r.obstacle_sizes[j])
                  for j in range(len(r.obstacle_sizes))
                  if j != k and not np.isnan(r.obstacle_xy[i, j, 0])]
        if not others:
            continue
        rects = np.concatenate((static, np.array(others)))
        cost = grid.costs(rects, SPEC.inscribed_m, SPEC.inflation_m, SPEC.cost_scaling)
        s, g = r.start_goal(i)
        assert R.plan(grid, cost, s, g) is not None
        checked += 1
    assert checked > 0


def _REG_BOUNDS(site_static):
    return site_static[0].bounds()


def test_keep_path_zero_accepts_only_paths_that_pass_inside_the_aisle(site_static):
    import region_spec as rg

    r = _build(site_static, obstacle_demo_share=1.0, obstacle_demo_keep_path=0)
    for i in np.flatnonzero(np.isfinite(r.obstacle_gap).any(1)):
        k = int(np.flatnonzero(np.isfinite(r.obstacle_gap[i]))[0])
        reg = r.regions[r.obstacle_region[i, k]]
        box = R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k])
        assert rg.passes_beside(reg, box, r.path[i, : r.npts[i]])

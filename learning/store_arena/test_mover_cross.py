"""건넘 구간과 mover_kinds 테스트(시뮬 없이): v7 남쪽 진열대 틈, 구간 기하, 가림, 기본값 불변."""

import dataclasses
import math
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import crossing_eval as CE  # noqa: E402
import mover_spec as M  # noqa: E402
import route_spec as R  # noqa: E402

SPEC = dataclasses.replace(A.ARENA, obstacle_layouts=4)
CROSS = dataclasses.replace(SPEC, mover_cross_prob=1.0, mover_prob=1.0)
PERSON, CART = M.MOVERS


def _sample_legs_before(rects, bounds, path, npts, start, goal, shape, spec, gen, candidates=256):
    """이 변경 전 sample_legs 그대로 (기본값 불변 비교용)."""
    n, dev = start.shape[0], start.device

    def rand(*s):
        return torch.rand(*s, device=dev, generator=gen)

    x0, y0, x1, y1 = bounds
    c = candidates
    on_path = rand(n, c) < spec.mover_on_path_prob
    j = (rand(n, c) * npts[:, None]).long().clamp(max=path.shape[1] - 1)
    centre_path = torch.gather(path, 1, j[..., None].expand(n, c, 2))
    centre_any = torch.stack((x0 + rand(n, c) * (x1 - x0), y0 + rand(n, c) * (y1 - y0)), -1)
    centre = torch.where(on_path[..., None], centre_path, centre_any)
    ang = rand(n, c) * 2 * math.pi
    lo, hi = spec.mover_leg_m
    half = 0.5 * (lo + rand(n, c) * (hi - lo))
    d = torch.stack((torch.cos(ang), torch.sin(ang)), -1)
    a, b = centre - half[..., None] * d, centre + half[..., None] * d
    r = shape.radius
    ends = torch.stack((a, b), -2)
    inside = (ends[..., 0] > x0 + r) & (ends[..., 0] < x1 - r)
    inside &= (ends[..., 1] > y0 + r) & (ends[..., 1] < y1 - r)
    ok = (M.leg_clearance(a, b, rects) > r + M.LEG_CHECK_MARGIN_M) & inside.all(-1)
    ok &= M.point_segment_distance(start, a, b) > spec.mover_node_clear_m
    ok &= M.point_segment_distance(goal, a, b) > spec.mover_node_clear_m
    on = torch.arange(path.shape[1], device=dev)[None, None] < npts[:, None, None]
    for end in (a, b):
        d = (end[:, :, None] - path[:, None]).norm(dim=-1)
        d = torch.where(on, d, torch.full_like(d, 9.0)).amin(-1)
        ok &= d - r >= spec.mover_end_path_clear_m
    first = ok.float().argmax(-1)
    rows = torch.arange(n, device=dev)
    active = ok.any(-1) & (rand(n) < spec.mover_prob)
    k0, k1 = spec.mover_speed_scale
    speed = shape.speed * (k0 + rand(n) * (k1 - k0))
    p0, p1 = spec.mover_pause_s
    return {
        "a": a[rows, first],
        "b": b[rows, first],
        "speed": speed,
        "pause": p0 + rand(n) * (p1 - p0),
        "phase": rand(n),
        "active": active,
    }


@pytest.fixture(scope="module")
def batch():
    site = A.load_site(SPEC)
    boxes = A.arena_boxes(SPEC, site)
    static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes])
    shelves = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes if b.kind == "shelf"])
    r = R.build_routes(SPEC, site, static)
    rects = []
    for i in range(r.count):
        bx = [
            R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k])
            for k in range(len(r.obstacle_sizes))
            if not np.isnan(r.obstacle_xy[i, k, 0])
        ]
        bx += [(9.0, 9.0, 9.1, 9.1)] * (len(r.obstacle_sizes) - len(bx))
        rects.append(np.concatenate((static, np.array(bx))))
    ends = np.array([r.start_goal(i) for i in range(r.count)])
    f32 = dict(dtype=torch.float32)
    return {
        "bounds": site.bounds(),
        "shelves": torch.tensor(shelves, **f32),
        "openings": M.find_openings(shelves, site.bounds()),
        "rects": torch.tensor(np.array(rects), **f32),
        "path": torch.tensor(r.path, **f32),
        "npts": torch.tensor(r.npts),
        "start": torch.tensor(ends[:, 0], **f32),
        "goal": torch.tensor(ends[:, 1], **f32),
    }


def _legs(t, shape, spec, seed=0, openings=True):
    gen = torch.Generator().manual_seed(seed)
    legs = M.sample_legs(
        t["rects"], t["bounds"], t["path"], t["npts"], t["start"], t["goal"], shape, spec, gen,
        openings=t["openings"] if openings else None,
    )
    return legs, gen


# ---- 틈 찾기 ----
def test_v7_has_one_opening_the_south_shelf_gap(batch):
    o = batch["openings"]
    assert o.shape == (1, 7)
    axis, g0, g1, far_lo, b0, b1, far_hi = o[0].tolist()
    assert axis == 0 and g1 - g0 == pytest.approx(0.25, abs=1e-5)  # 남쪽 진열대 두 개 사이
    assert (b0, b1) == pytest.approx((0.364, 0.404), abs=1e-5)
    assert far_lo == pytest.approx(0.0) and far_hi == pytest.approx(0.754, abs=1e-5)  # 벽, 진열대 A


def test_opening_rules_on_made_up_shelves():
    bounds = (0.0, 0.0, 2.0, 2.0)
    # 세로 줄 틈 0.2 (axis 1) 와 벽에 닿는 칸막이 둘 사이 0.2
    shelves = [
        (0.50, 0.30, 0.54, 0.80), (0.50, 1.00, 0.54, 1.70),  # 세로 틈 y 0.80..1.00
        (0.80, 0.00, 0.82, 0.20), (1.02, 0.00, 1.04, 0.20),  # 벽에 닿음
    ]
    o = M.find_openings(shelves, bounds)
    assert o.shape == (1, 7) and o[0, 0] == 1  # 세로 틈 하나, 칸막이는 뺌
    assert o[0, 1:3].tolist() == pytest.approx([0.80, 1.00])
    assert o[0, 3].item() == pytest.approx(0.0) and o[0, 6].item() == pytest.approx(2.0)
    wide = [(0.3, 1.0, 0.8, 1.04), (1.4, 1.0, 1.9, 1.04)]  # 폭 0.6: 틈 아님 (넓은 바닥)
    assert len(M.find_openings(wide, bounds)) == 0
    stacked = [(0.3, 1.0, 1.5, 1.04), (0.3, 1.30, 1.5, 1.34)]  # 통로 0.26, 같은 줄 아님
    assert len(M.find_openings(stacked, bounds)) == 0
    shallow = [(0.3, 0.10, 0.8, 0.14), (1.0, 0.10, 1.5, 0.14)]  # 남쪽 통로 깊이 0.10
    assert len(M.find_openings(shallow, bounds)) == 0


# ---- 건넘 구간 기하 ----
def _blocked(p, q, rects):
    """선분 p-q (2,) 가 rect (B, 4) 하나라도 지나는지 (slab)."""
    d = q - p
    t0, t1 = torch.zeros(len(rects)), torch.ones(len(rects))
    for k in range(2):
        if abs(float(d[k])) < 1e-12:
            inside = (p[k] >= rects[:, k]) & (p[k] <= rects[:, k + 2])
            t1 = torch.where(inside, t1, -torch.ones_like(t1))
            continue
        a = (rects[:, k] - p[k]) / d[k]
        b = (rects[:, k + 2] - p[k]) / d[k]
        t0, t1 = torch.maximum(t0, torch.minimum(a, b)), torch.minimum(t1, torch.maximum(a, b))
    return bool((t0 <= t1).any())


def test_crossing_legs_go_through_the_opening_from_aisle_centre_to_aisle_centre(batch):
    t = batch
    legs, _ = _legs(t, PERSON, CROSS)
    assert legs["cross"].all()
    act = legs["active"]
    # v7 시드 0 실측 0.70: 박스가 틈/통로 끝을 막거나
    # 경로가 틈을 지나 끝점이 경로 옆이 못 되면 비활성
    assert act.float().mean() > 0.6
    _, g0, g1, far_lo, b0, b1, far_hi = t["openings"][0].tolist()
    r, m = PERSON.radius, M.OPEN_MARGIN_M
    a, b = legs["a"][act], legs["b"][act]
    for end in (a, b):  # 틈 방향: 사람 몸이 선반 끝에서 OPEN_MARGIN_M 이상
        assert end[:, 0].min() >= g0 + r + m - 1e-5 and end[:, 0].max() <= g1 - r - m + 1e-5
    # 양끝이 각 통로 중심선까지 또는 그 너머: 남쪽 통로 0..0.364, 가운데 통로 0.404..0.754
    assert a[:, 1].max() <= 0.5 * (far_lo + b0) + 1e-5
    assert b[:, 1].min() >= 0.5 * (b1 + far_hi) - 1e-5
    s = torch.linspace(0, 1, 200)
    pts = a[:, None] + s[None, :, None] * (b - a)[:, None]
    assert M.point_rect_distance(pts, t["rects"][act]).min() > r  # 벽, 선반, 박스 여유
    for key in ("start", "goal"):
        d = (pts - t[key][act][:, None]).norm(dim=-1).amin(-1)
        assert d.min() > CROSS.mover_node_clear_m
    valid = torch.arange(t["path"].shape[1])[None] < t["npts"][act][:, None]
    for end in (a, b):
        d = (end[:, None] - t["path"][act]).norm(dim=-1)
        d = torch.where(valid, d, torch.full_like(d, 9.0)).amin(-1)
        assert d.min() - r >= CROSS.mover_end_path_clear_m - 1e-5
    # 길이는 mover_leg_m 이 아니라 틈이 정함 (두 통로 중심선 사이 이상)
    assert (b - a)[:, 1].min() >= 0.5 * (b1 + far_hi) - 0.5 * (far_lo + b0) - 1e-5


def test_a_shelf_hides_the_crossing_person_for_part_of_the_leg(batch):
    """옆 통로 중심선 위 점(틈에서 옆으로 0.35 m)에서 사람까지 직선: 구간 일부에서 선반에 막히고
    일부에서는 보인다. 양쪽 통로에서 모두."""
    t = batch
    legs, _ = _legs(t, PERSON, CROSS)
    act = legs["active"].nonzero()[:, 0][:40]
    _, g0, g1, far_lo, b0, b1, far_hi = t["openings"][0].tolist()
    gx = 0.5 * (g0 + g1)
    views = [
        torch.tensor([gx - 0.35, 0.5 * (b1 + far_hi)]),
        torch.tensor([gx + 0.35, 0.5 * (far_lo + b0)]),
    ]
    s = torch.linspace(0, 1, 60)
    for i in act.tolist():
        a, b = legs["a"][i], legs["b"][i]
        for v in views:
            hidden = [_blocked(v, a + k * (b - a), t["shelves"]) for k in s]
            assert any(hidden) and not all(hidden)


def test_crossing_flag_for_our_path():
    a, b = torch.tensor([[1.14, 0.10], [1.14, 0.10]]), torch.tensor([[1.14, 0.60], [1.14, 0.60]])
    xs = torch.linspace(0.5, 1.6, 45)
    across = torch.stack((xs, torch.full_like(xs, 0.25)), -1)  # 구간을 가로지르는 경로
    beside = torch.stack((xs * 0.3 + 0.4, torch.full_like(xs, 0.25)), -1)  # x 0.55..0.88, 안 닿음
    on = M.leg_on_path(a, b, torch.stack((across, beside)), torch.tensor([45, 45]), PERSON.radius)
    assert on.tolist() == [True, False]
    # 패딩 점은 안 셈
    on = M.leg_on_path(a[:1], b[:1], across[None], torch.tensor([5]), PERSON.radius)
    assert on.tolist() == [False]


def test_crossing_flag_matches_a_dense_check_on_v7(batch):
    t = batch
    legs, _ = _legs(t, PERSON, CROSS)
    act = legs["active"]
    a, b = legs["a"][act], legs["b"][act]
    s = torch.linspace(0, 1, 400)
    pts = a[:, None] + s[None, :, None] * (b - a)[:, None]  # (n, 400, 2)
    path, npts = t["path"][act], t["npts"][act]
    d = (pts[:, :, None] - path[:, None]).norm(dim=-1)  # (n, 400, L)
    valid = torch.arange(path.shape[1])[None, None] < npts[:, None, None]
    d = torch.where(valid, d, torch.full_like(d, 9.0)).flatten(1).amin(-1)
    flag = legs["cross_path"][act]
    assert (flag == (d < PERSON.radius)).float().mean() > 0.99  # 400 점 근사
    assert flag.any() and (~flag).any()


def test_crossing_only_for_the_person_and_appended_draws(batch):
    """건넘을 켜도 수레는 그대로이고, 사람은 속도/정지/시작 위치가 그대로 (난수는 맨 뒤에 뽑음)."""
    t = batch
    spec = dataclasses.replace(SPEC, mover_cross_prob=0.5)
    base, _ = _legs(t, CART, SPEC)
    cart, _ = _legs(t, CART, spec)
    for k in base:
        assert torch.equal(base[k], cart[k])
    assert not cart["cross"].any()
    base, _ = _legs(t, PERSON, SPEC)
    cross, _ = _legs(t, PERSON, spec)
    for k in ("speed", "pause", "phase"):
        assert torch.equal(base[k], cross[k])
    f = cross["cross"].float().mean().item()
    assert 0.35 < f < 0.65
    same = ~cross["cross"]
    assert torch.equal(base["a"][same], cross["a"][same])
    assert torch.equal(base["active"][same], cross["active"][same])


# ---- mover_kinds ----
def test_mover_kinds_turns_off_the_others_without_changing_draws(batch):
    t = batch
    only = dataclasses.replace(SPEC, mover_kinds="person")
    for shape in M.MOVERS:
        base, g0 = _legs(t, shape, SPEC)
        legs, g1 = _legs(t, shape, only)
        assert torch.equal(g0.get_state(), g1.get_state())
        for k in ("a", "b", "speed", "pause", "phase"):
            assert torch.equal(base[k], legs[k])
        if shape.name == "person":
            assert torch.equal(base["active"], legs["active"]) and legs["active"].any()
        else:
            assert not legs["active"].any()
    assert M.kind_on(dataclasses.replace(SPEC, mover_kinds=" cart , person"), CART)
    with pytest.raises(ValueError):
        M.kind_on(dataclasses.replace(SPEC, mover_kinds="persn"), PERSON)


# ---- 기본값 불변 ----
@pytest.mark.parametrize("shape", M.MOVERS, ids=lambda s: s.name)
@pytest.mark.parametrize("openings", [True, False])
def test_defaults_give_the_legs_and_draws_of_before(batch, shape, openings):
    t = batch
    assert SPEC.mover_cross_prob == 0.0 and SPEC.mover_kinds == ""
    for seed in (0, 1):
        gen = torch.Generator().manual_seed(seed)
        ref = _sample_legs_before(
            t["rects"], t["bounds"], t["path"], t["npts"], t["start"], t["goal"], shape, SPEC, gen
        )
        legs, g1 = _legs(t, shape, SPEC, seed, openings)
        assert torch.equal(gen.get_state(), g1.get_state())  # 더 뽑은 난수 없음
        for k, v in ref.items():
            assert torch.equal(v, legs[k]), k
        assert not legs["cross"].any() and not legs["cross_path"].any()


# ---- 평가 집계 ----
def test_episode_flags_and_rates_split():
    legs = [
        {"active": torch.tensor([True, True, False, True]),
         "cross": torch.tensor([True, True, True, False]),
         "cross_path": torch.tensor([True, False, True, False])},
        {"active": torch.tensor([True, True, True, True]),
         "cross": torch.zeros(4, dtype=torch.bool),
         "cross_path": torch.zeros(4, dtype=torch.bool)},
    ]
    f = CE.episode_flags(legs, torch.arange(4))
    assert f.tolist() == [[True, True], [True, False], [False, False], [False, False]]
    keys = ("success", "collision", "oob", "timeout")
    out = CE.rates_by_cross([0, 1, 0, 3], f[:, 0].numpy(), f[:, 1].numpy(), keys)
    assert out["cross_on_path"] == {"episodes": 1, "success": 1.0, "collision": 0.0,
                                    "oob": 0.0, "timeout": 0.0}
    assert out["cross_off_path"]["episodes"] == 1 and out["cross_off_path"]["collision"] == 1.0
    assert out["no_cross"]["episodes"] == 2 and out["no_cross"]["success"] == 0.5
    assert out["no_cross"]["timeout"] == 0.5

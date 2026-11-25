"""자기 대국 규칙 테스트 (Isaac 없이): 짝 뽑기, 같이 끝내기, ghost intent, 짝 시나리오.
실행: python -m pytest learning/store_arena -q"""

import dataclasses
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import ghost_spec as G  # noqa: E402
import route_spec as R  # noqa: E402
import scenario_spec as S  # noqa: E402

SPEC = dataclasses.replace(A.ARENA, obstacle_layouts=2)


@pytest.fixture(scope="module")
def world():
    site = A.load_site(SPEC)
    boxes = A.arena_boxes(SPEC, site)
    rects = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes])
    shelves = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes if b.kind == "shelf"])
    routes = R.build_routes(SPEC, site, rects)
    scen = {k: S.build_pair_scenario(k, routes, rects, shelves) for k in ("tee", "entry")}
    return routes, rects, shelves, scen


def test_partner_swaps_pairs():
    assert G.partner(torch.arange(6)).tolist() == [1, 0, 3, 2, 5, 4]


def test_paired_tee_and_entry_meet_within_the_tolerance(world):
    """둘 다 자기 시작 노드에서 출발: 만남점까지 두 경로 길이 차가 PAIR_TOL_M 이하."""
    routes, rects, _, scen = world
    assert len(scen["tee"]) >= 10 and len(scen["entry"]) >= 2
    for kind, rows in scen.items():
        for ra, rb in rows:
            assert ra % routes.layouts == 0 and rb % routes.layouts == 0
            p = routes.path[ra, : routes.npts[ra]]
            q = routes.path[rb, : routes.npts[rb]]
            d, sq = S.project(q, p)
            hit = np.nonzero(d < S.TOUCH_M)[0]
            assert len(hit)
            sp = S.arc_of(p)
            # 고른 만남점 (tee: 첫 만남) 에서 길이 차
            if kind == "tee":
                i = hit[0]
                assert abs(sq[i] - sp[i]) <= S.PAIR_TOL_M + 1e-9
            assert np.linalg.norm(p[0] - q[0]) >= S.START_SEP_M
            assert routes.pairs[ra // routes.layouts][1] != routes.pairs[rb // routes.layouts][1]


def test_paired_corner_is_empty_on_v7(world):
    """노드에서 같이 출발하는 corner 짝은 v7 맵에 없음 (보고서에 적음)."""
    routes, rects, shelves, _ = world
    assert len(S.build_pair_scenario("corner", routes, rects, shelves)) == 0


def test_tables_and_draws(world):
    routes, _, _, scen = world
    t = G.pair_tables(routes, S.START_SEP_M, scen["tee"])
    pairs = np.array(routes.pairs)
    rev = t["reverse"].numpy()
    for i, j in enumerate(rev):
        assert j >= 0 and tuple(pairs[j]) == (pairs[i][1], pairs[i][0])
    g = torch.Generator().manual_seed(0)
    a, b, kind = G.draw_pairs(t, 6000, 0.25, 0.25, g)
    share = torch.bincount(kind, minlength=3).float() / 6000
    assert abs(float(share[1]) - 0.25) < 0.03 and abs(float(share[2]) - 0.25) < 0.03
    rnd = kind == 0
    assert bool(t["ok"][a[rnd], b[rnd]].all())
    head = kind == 1
    assert bool((b[head] == t["reverse"][a[head]]).all())
    sc = {tuple(r) for r in t["scen"].tolist()}
    for x, y in zip(a[kind == 2].tolist(), b[kind == 2].tolist(), strict=True):
        assert (x, y) in sc or (y, x) in sc
    # 시나리오 후보가 없으면 그 몫은 무작위
    t0 = G.pair_tables(routes, S.START_SEP_M, None)
    _, _, k0 = G.draw_pairs(t0, 500, 0.0, 0.5, g)
    assert int((k0 == 2).sum()) == 0


def test_pair_priority_is_never_both():
    g = torch.Generator().manual_seed(1)
    pa, pb = G.pair_priority(4000, 0.5, g)
    assert not bool((pa & pb).any())
    assert abs(float((pa | pb).float().mean()) - 0.5) < 0.04
    za, zb = G.pair_priority(10, 0.0)
    assert not bool(za.any() or zb.any())


def test_pair_dones_end_both_together():
    t, f = True, False
    term = torch.tensor([t, f, f, f, f, f, t, t])
    to = torch.tensor([f, f, f, t, f, f, f, f])
    mutual = torch.tensor([t, f, f, f, f, f, f, f])
    term2, to2, hit = G.pair_dones(term, to, mutual)
    # 짝 0-1: 0 이 ghost 와 충돌 -> 1 도 충돌
    assert term2[:2].tolist() == [t, t] and hit[:2].tolist() == [f, t]
    assert not bool(hit[2:].any())
    # 짝 2-3: 3 시간초과 -> 2 도 시간초과
    assert to2[2:4].tolist() == [t, t] and not bool(term2[2:4].any())
    # 짝 4-5: 아무 일 없음
    assert not bool(term2[4:6].any() or to2[4:6].any())
    # 짝 6-7: 둘 다 끝남, 시간초과 아님
    assert term2[6:].tolist() == [t, t] and not bool(to2[6:].any())
    # 늘 짝이 같이 끝남
    done = term2 | to2
    assert bool((done == done[G.partner(torch.arange(8))]).all())


def test_path_ahead_walks_the_remaining_path_and_stops_at_the_goal():
    path = torch.tensor([[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [1.0, 1.0]]])
    cum = torch.tensor([[0.0, 1.0, 2.0, 2.0]])
    npts = torch.tensor([3])
    pts = G.path_ahead(path, cum, npts, torch.tensor([0.5]), 0.4, 4)
    want = [[0.9, 0.0], [1.0, 0.3], [1.0, 0.7], [1.0, 1.0]]
    assert torch.allclose(pts[0], torch.tensor(want), atol=1e-6)


def test_mutual_collision_overrides_a_partner_success_in_the_same_step():
    """짝이 같은 스텝에 성공해도 서로 부딪혔으면 충돌 (성공 보상을 받지 않게 env 가 success 를 끔)."""
    term = torch.tensor([True, True])
    to = torch.tensor([False, False])
    mutual = torch.tensor([True, False])  # 0 이 ghost 와 충돌, 1 은 같은 스텝에 성공
    term2, to2, hit = G.pair_dones(term, to, mutual)
    assert hit.tolist() == [False, True]
    assert term2.tolist() == [True, True] and not bool(to2.any())


def test_path_progress_projects_onto_the_segment():
    path = torch.tensor([[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]])
    cum = torch.tensor([[0.0, 1.0, 2.0]])
    npts = torch.tensor([3])
    done = G.path_progress(path, cum, npts, torch.tensor([0]), torch.tensor([[0.4, 0.05]]))
    assert float(done[0]) == pytest.approx(0.4, abs=1e-6)
    done = G.path_progress(path, cum, npts, torch.tensor([1]), torch.tensor([[0.95, 0.3]]))
    assert float(done[0]) == pytest.approx(1.3, abs=1e-6)
    done = G.path_progress(path, cum, npts, torch.tensor([2]), torch.tensor([[1.0, 1.2]]))
    assert float(done[0]) == pytest.approx(2.0, abs=1e-6)  # 목표에서 멈춤

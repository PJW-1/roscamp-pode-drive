"""평가 시나리오 테스트 (Isaac 없이): 후보가 시나리오의 정의를 아레나 기하로 만족하는지.
실행: python -m pytest learning/store_arena -q"""

import dataclasses
import math
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import robot_spec as rb  # noqa: E402
import route_spec as R  # noqa: E402
import scenario_spec as S  # noqa: E402

SPEC = dataclasses.replace(A.ARENA, obstacle_layouts=2)  # 배치 0 만 써야 함
MIN_PAIRS = {"corner": 6, "tee": 12, "entry": 6}  # 시나리오별 최소 시작/목표 쌍
TIMING_TOL_M = 0.05  # 만남점까지 두 경로 길이 차 허용 (경로 투영 오차 TOUCH_M 포함)


@pytest.fixture(scope="module")
def world():
    site = A.load_site(SPEC)
    boxes = A.arena_boxes(SPEC, site)
    rects = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes])
    shelves = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes if b.kind == "shelf"])
    routes = R.build_routes(SPEC, site, rects)
    graph = rb.node_graph(site)
    built = {k: S.build_scenario(k, routes, graph, rects, shelves) for k in S.SCENARIOS}
    return routes, graph, rects, shelves, built


def rows(world, kind):
    routes, graph, _, _, built = world
    sc = built[kind]
    for i in range(sc.count):
        r = int(sc.route[i])
        p = routes.path[r, : routes.npts[r]]
        q = graph.path[sc.a[i], sc.b[i], : graph.npts[sc.a[i], sc.b[i]]]
        start = S.point_at(q, S.arc_of(q), sc.s0[i])
        yield sc, i, r, p, q, start


def heading(p, s):
    return S.heading_at(p, S.arc_of(p), s)


@pytest.mark.parametrize("kind", S.SCENARIOS)
def test_every_scenario_has_enough_distinct_pairs_on_layout_0(world, kind):
    routes, _, _, _, built = world
    sc = built[kind]
    assert (sc.route % routes.layouts == 0).all()  # 박스 없는 배치
    assert len(set(sc.route.tolist())) >= MIN_PAIRS[kind]


@pytest.mark.parametrize("kind", S.SCENARIOS)
def test_both_reach_the_meeting_point_together_at_equal_speed(world, kind):
    """만남점까지 우리 경로 길이와 로봇 출발점부터 로봇 경로 길이의 차가 TIMING_TOL_M 미만."""
    for sc, i, _, p, q, start in rows(world, kind):
        m = sc.meet[i][None]
        d_p, s_p = S.project(p, m)
        d_q, s_q = S.project(q, m)
        assert d_p[0] < 1e-6 and d_q[0] < S.TOUCH_M
        assert abs(s_p[0] - sc.s_meet[i]) < 1e-6
        assert abs((s_q[0] - sc.s0[i]) - s_p[0]) < TIMING_TOL_M
        assert s_p[0] >= S.MIN_MEET_M
        assert np.linalg.norm(start - p[0]) >= S.START_SEP_M


@pytest.mark.parametrize("kind", S.SCENARIOS)
def test_robot_starts_in_free_space(world, kind):
    rects = world[2]
    for _, _, _, _, _, start in rows(world, kind):
        assert R.point_rect_distance(start[0:1], start[1:2], rects)[0] >= S.START_CLEAR_M


def test_corner_start_line_is_blocked_by_a_shelf(world):
    """두 시작 자세 사이 선분이 선반 박스에 막힘: 레이캐스트 거리가 두 중심 거리보다 짧음."""
    shelves = world[3]
    for _, _, _, p, _q, start in rows(world, "corner"):
        d = start - p[0]
        hit = A.raycast_rects(
            torch.tensor(p[0:1], dtype=torch.float32),
            torch.tensor([math.atan2(d[1], d[0])], dtype=torch.float32),
            torch.tensor(shelves, dtype=torch.float32),
            min_range=0.0, max_range=1e6, n_rays=2,
        )[0, 1]
        assert float(hit) < float(np.linalg.norm(d))


def test_corner_stays_hidden_until_close(world):
    shelves = world[3]
    for sc, i, _, p, q, _ in rows(world, "corner"):
        ours, rob, ryaw = S.drive_together(p, q, sc.s0[i], sc.s_meet[i])
        gap = np.linalg.norm(ours - rob, axis=-1)
        far = np.cumprod(gap >= S.CLOSE_M).astype(bool)  # 처음 CLOSE_M 아래가 되기 전
        far[0] = True
        assert S.robot_hidden(ours[far], rob[far], ryaw[far], shelves).all()


def test_tee_crosses_from_the_side(world):
    for sc, i, _, p, q, _ in rows(world, "tee"):
        s_q = S.project(q, sc.meet[i][None])[1][0]
        c = float(heading(p, sc.s_meet[i]) @ heading(q, s_q))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        assert S.TEE_DEG[0] - 1.0 <= ang <= S.TEE_DEG[1] + 1.0


def test_entry_meets_head_on_in_a_narrow_passage(world):
    rects = world[2]
    for sc, i, _, p, q, _ in rows(world, "entry"):
        s_q = S.project(q, sc.meet[i][None])[1][0]
        u = heading(p, sc.s_meet[i])
        assert float(u @ heading(q, s_q)) < S.ENTRY_COS + 0.05
        assert S.free_width(sc.meet[i], u, rects) < S.PASSAGE_M


def test_robot_box_half_matches_the_robot():
    assert S.ROBOT_HALF == (0.5 * rb.ROBOT.size[0], 0.5 * rb.ROBOT.size[1])


def test_scenario_names_and_sampling(world):
    assert S.scenario_names("") == ()
    assert S.scenario_names("tee, entry") == ("tee", "entry")
    with pytest.raises(ValueError):
        S.scenario_names("tee,nope")
    built = world[4]
    packed = S.pack([built["corner"], built["tee"]])
    g = torch.Generator().manual_seed(0)
    pick = S.sample(packed, 4000, g)
    nc = built["corner"].count
    assert pick.min() >= 0 and pick.max() < nc + built["tee"].count
    share = float((pick < nc).float().mean())
    assert abs(share - 0.5) < 0.05  # 시나리오는 고르게
    assert len(set(pick[pick < nc].tolist())) == nc  # 후보는 모두 나옴


def test_robot_route_starts_at_s0_and_meets_on_time(world):
    """env 와 같은 방식(chain_route, t0 = s0 / speed)으로 깐 로봇이 우리 만남 시각에 만남점에 있음."""
    routes, graph, _, _, built = world
    sc = built["tee"]
    gt = rb.graph_tensors(graph)
    n = sc.count
    chain = torch.stack((torch.as_tensor(sc.a), torch.as_tensor(sc.b), torch.as_tensor(sc.a)), -1)
    speed = torch.full((n,), rb.ROBOT.speed)
    route = rb.chain_route(gt, chain, speed, torch.zeros(n, 2))
    t0 = torch.as_tensor(sc.s0, dtype=torch.float32) / speed
    pos, _, _ = rb.robot_pose(route, t0 + torch.as_tensor(sc.s_meet, dtype=torch.float32) / speed)
    assert (pos - torch.as_tensor(sc.meet, dtype=torch.float32)).norm(dim=-1).max() < TIMING_TOL_M

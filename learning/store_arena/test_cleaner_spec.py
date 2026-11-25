"""청소 로봇 칸 테스트(시뮬 없이): 줄 모양, 바퀴 시간, 비키기, 로봇 수, 기본값 그대로.

env 연결은 store_arena_env.py 의 메서드 소스를 ast 로 꺼내 가짜 env 에 붙여 돌린다 (Isaac 없이).
"""

import ast
import math
import os
import subprocess
import sys
import types
from types import SimpleNamespace

import numpy as np
import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))

import arena_spec as A  # noqa: E402
import cleaner_spec as C  # noqa: E402
import ghost_spec as gh  # noqa: E402
import robot_spec as RS  # noqa: E402
import scenario_spec as scn  # noqa: E402
import task_spec as ts  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
CLEAN_BRANCH = "origin/feat/cleaner-yield-2026-11-25"
BASE = "9a09378a"  # 청소 칸을 넣기 전 커밋


@pytest.fixture(scope="module")
def site():
    return A.load_site(A.ARENA)


@pytest.fixture(scope="module")
def lap(site):
    return C.clean_lap(site)


def _git_show(path: str) -> str | None:
    try:
        out = subprocess.run(
            ["git", "show", path], cwd=REPO, capture_output=True, text=True, encoding="utf-8"
        )
    except OSError:
        return None
    return out.stdout if out.returncode == 0 else None


@pytest.fixture(scope="module")
def real_coverage():
    """청소 브랜치의 coverage.py 를 임시 모듈로 (이 저장소의 fleet_core.fleet_map 을 씀)."""
    src = _git_show(f"{CLEAN_BRANCH}:pc/fleet_core/fleet_core/coverage.py")
    if src is None:
        pytest.skip(f"{CLEAN_BRANCH} not available")
    import fleet_core

    m = types.ModuleType("fleet_core._coverage_ref")
    m.__package__ = fleet_core.__name__
    sys.modules[m.__name__] = m
    exec(compile(src, "coverage.py", "exec"), m.__dict__)
    return m


# ---- 줄 모양: 실제 coverage.py 와 같음, 성질 ----
def test_rows_and_transits_match_the_real_coverage(site, real_coverage):
    real, ours = real_coverage.make_plan(site), C.make_plan(site)
    assert ours.lines == real.lines
    assert [t[2] for t in ours.transits] == [t[2] for t in real.transits]
    assert ours.start == real.start
    s = real_coverage.summary(real)
    lap = C.clean_lap(site)
    assert len(lap.rows) == s["lines"]
    assert lap.plan_lap_s == pytest.approx(s["lap_time_s"], abs=0.05)


def test_the_branch_map_gives_17_rows_and_116_8_s(tmp_path, real_coverage):
    """청소 브랜치의 지도(옛 v7)에서는 그 브랜치의 coverage_v7.yaml 요약과 같다."""
    from fleet_core.fleet_map import load_fleet_map

    text = _git_show(f"{CLEAN_BRANCH}:pc/fleet_core/fleet_core/data/fleet_map_v7.yaml")
    p = tmp_path / "fleet_map_old_v7.yaml"
    p.write_text(text, encoding="utf-8")
    lap = C.clean_lap(load_fleet_map(str(p)))
    assert len(lap.rows) == 17
    assert round(lap.plan_lap_s, 1) == 116.8


def test_rows_are_x_lines_on_the_free_grid(site, lap):
    plan = C.make_plan(site)
    grid = plan.grid
    ys = sorted({a[1] for a, _ in lap.rows})
    for a, b in lap.rows:
        assert a[1] == b[1]  # x 방향 줄
        # 줄은 벽·진열대·다른 구역에서 몸 반지름 + 여유 이상, HALL 안
        assert C.path_clear(grid, [a, b])
        assert C.segment_clearance(grid, a, b) >= C.ROBOT_RADIUS_M + C.MARGIN_M - 1e-9
    # 한 셀 안 이웃 줄 간격은 0.10 m 이하 (격자 0.025 m 단위)
    for cell in plan.cells:
        cy = sorted({a[1] for a, b, c in plan.lines if c == cell["id"]})
        for y0, y1 in zip(cy, cy[1:], strict=False):
            assert y1 - y0 <= C.LINE_SPACING_M + 1e-9
    assert len(ys) >= 10


def test_lap_runs_bay_out_rows_back_bay_in(site, lap):
    assert tuple(lap.pts[0]) == site.node_pose("P-1") == tuple(lap.pts[-1])
    e1 = np.array(site.node_pose("E-1"))
    d_e1 = np.linalg.norm(lap.pts - e1, axis=1)
    # 나갈 때(둥근 모서리라 몇 cm 안)와 돌아올 때(후진 시작점) E-1
    assert d_e1[:10].min() < 0.03 and d_e1[-10:].min() < 1e-9
    assert lap.reverse.sum() >= 1 and lap.reverse[-1]
    assert not lap.reverse[: len(lap.reverse) // 2].any()
    # 모든 줄을 차례로 지남: 줄 끝은 다리 끝이라 그대로, 줄 시작은 둥근 모서리로 회전 반경 안
    k = 0
    for a, b in lap.rows:
        assert np.linalg.norm(lap.pts[k:] - np.array(a), axis=1).min() < C.TURN_RADIUS_M
        d = np.linalg.norm(lap.pts[k:] - np.array(b), axis=1)
        assert d.min() < 1e-6
        k += int(np.flatnonzero(d < 1e-6)[0])
    assert np.diff(lap.arc).max() <= C.DENSE_M + 1e-9


# ---- 바퀴 시간과 출발 시각 ----
def test_lap_time_at_cleaner_speed(lap):
    table = C.lap_table(lap, 3)
    route = C.lap_route(table, torch.tensor([C.SPEED_MPS, C.SPEED_MPS / 2]))
    t_lap = lap.length / C.SPEED_MPS
    assert route["time"][0, -1].item() == pytest.approx(3 * t_lap, rel=1e-4)
    # 한 바퀴 뒤 같은 자리, 반 속도는 두 배 걸림
    t = torch.tensor([t_lap, 2 * t_lap])
    pos, _, arc = RS.robot_pose(route, t)
    home = torch.tensor(lap.pts[0], dtype=torch.float32).expand(2, 2)
    assert torch.allclose(pos, home, atol=1e-4)
    assert arc[0].item() == pytest.approx(lap.length, rel=1e-4)
    # 바퀴는 줄 + 이동 계획 시간에서 크게 안 벗어남 (모서리 이음과 나가기/들어가기 차이)
    assert abs(t_lap - lap.plan_lap_s) < 0.05 * lap.plan_lap_s
    assert C.laps_needed(lap, 0.0) == 2


def test_intent_is_the_next_row_points(lap):
    table = C.lap_table(lap, 2)
    route = C.lap_route(table, torch.tensor([C.SPEED_MPS]))
    a, b = lap.rows[len(lap.rows) // 2]
    k = int(np.linalg.norm(lap.pts - np.array(a), axis=1).argmin())
    t = torch.tensor([lap.arc[k] / C.SPEED_MPS + 0.5], dtype=torch.float32)
    pts = RS.intent_points(route, t, 0.1, 5)[0]
    assert torch.allclose(pts[:, 1], torch.full((5,), float(a[1])), atol=1e-4)  # 같은 줄 위
    sign = math.copysign(1.0, b[0] - a[0])
    end = torch.tensor(b, dtype=torch.float32)
    assert (sign * torch.diff(pts[:, 0]) > 0).all() or (pts[-1] - end).norm() < 1e-4


def test_intent_end_switches_at_each_leg_joint(lap):
    """다리 이음 바로 뒤 intent 끝은 새 다리 끝 (앞 다리 끝에서 잘리지 않음)."""
    table = C.lap_table(lap, 2)
    route = C.lap_route(table, torch.tensor([C.SPEED_MPS]))
    ends = np.unique(table["leg_end_arc"].numpy())
    for e, nxt in zip(ends[:-1], ends[1:], strict=True):
        t = torch.tensor([e / C.SPEED_MPS + 1e-3], dtype=torch.float32)
        out, _ = RS._interp(route["time"], t, route["leg_end_arc"][..., None])
        assert out[0, 0].item() == pytest.approx(nxt, abs=1e-3)


def test_start_clock_keeps_away_from_given_points(lap):
    table = C.lap_table(lap, 2)
    speed = torch.full((64,), C.SPEED_MPS)
    route = C.lap_route(table, speed)
    lap_s = lap.length / speed
    g = torch.Generator().manual_seed(0)
    u = torch.rand(64, 8, generator=g)
    first, _, _ = RS.robot_pose(route, u[:, 0] * lap_s)
    avoid = first[:, None].clone()  # 첫 후보 자리를 피해야 함
    t = C.start_clock(route, lap_s, u, avoid, torch.ones(64, 1, dtype=torch.bool), 0.3)
    pos, _, _ = RS.robot_pose(route, t)
    far = (pos - first).norm(dim=-1)
    assert (t < lap_s + 1e-4).all() and (t >= 0).all()
    assert (far >= 0.3).float().mean() > 0.9
    # 피할 점이 꺼져 있으면 첫 후보
    t2 = C.start_clock(route, lap_s, u, avoid, torch.zeros(64, 1, dtype=torch.bool), 0.3)
    assert torch.allclose(t2, u[:, 0] * lap_s)


# ---- 비키기: robot_step 의 대기·후퇴, 가장 가까운 상대 ----
def test_cleaner_waits_then_backs_off_a_short_way(lap):
    table = C.lap_table(lap, 2)
    route = C.lap_route(table, torch.tensor([C.SPEED_MPS]))
    a, b = lap.rows[len(lap.rows) // 2]
    k = int(np.linalg.norm(lap.pts - np.array(a), axis=1).argmin())
    t = torch.tensor([lap.arc[k] / C.SPEED_MPS + 2.0], dtype=torch.float32)
    pos, _, arc0 = RS.robot_pose(route, t)
    ahead = torch.tensor([[pos[0, 0].item() + math.copysign(0.25, b[0] - a[0]), a[1]]])
    wait, back, dt = torch.zeros(1), torch.zeros(1), 0.1
    t0 = t.clone()

    def step(t, wait, back, who=ahead, blocked=None):
        return RS.robot_step(route, t, wait, back, dt, who, 0.30, 10.0, 0.0, True, blocked=blocked)

    for _ in range(int(10.0 / dt) - 1):
        t, wait, back = step(t, wait, back)
    assert t.item() == pytest.approx(t0.item()) and back.item() == 0.0  # 10 s 동안 섬
    for _ in range(22):
        t, wait, back = step(t, wait, back)
    _, _, arc1 = RS.robot_pose(route, t)
    assert 0.0 < (arc0 - arc1).item() <= 2 * C.DENSE_M + 1e-6  # 줄을 따라 조금 물러남
    # 노드 예약 밖: 노드가 없으니 blocked 가 있어도 같음
    blocked = torch.ones(1, 7, dtype=torch.bool)
    far = torch.tensor([[9.0, 9.0]])
    t_a = step(t0, torch.zeros(1), torch.zeros(1), far, blocked)[0]
    assert t_a.item() == pytest.approx(t0.item() + dt)


def test_watch_picks_the_nearest_active_robot():
    cl = torch.tensor([[0.0, 0.0], [0.0, 0.0]])
    ours = torch.tensor([[1.0, 0.0], [1.0, 0.0]])
    others = torch.tensor([[[0.5, 0.0]], [[0.5, 0.0]]])
    on = torch.tensor([[True], [False]])
    w = C.watch_xy(cl, ours, others, on)
    assert torch.equal(w, torch.tensor([[0.5, 0.0], [1.0, 0.0]]))
    none = torch.zeros(2, 0, dtype=torch.bool)
    assert torch.equal(C.watch_xy(cl, ours, torch.zeros(2, 0, 2), none), ours)


# ---- env 메서드를 가짜 env 로: 로봇 수, 칸 배치, 기본값 그대로 ----
def _methods(src: str, names: set[str]) -> dict:
    tree = ast.parse(src)
    ns = {"torch": torch, "np": np, "rbs": RS, "cln": C, "scn": scn, "ts": ts, "gh": gh}
    out = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module([node], []), "store_arena_env.py", "exec"), ns)
            out[node.name] = ns[node.name]
    return out


ENV_METHODS = {
    "_sample_robots", "_cleaner_init", "_cleaner_pick", "_cleaner_route", "_cleaner_reset",
    "_cleaner_yield", "robot_state",
}


def _fake_env(methods: dict, n: int = 256, **over):
    a = A.make_arena_spec({"robot_max": 2, "robot_reserve": 0, **over})
    site = A.load_site(a)
    g = RS.node_graph(site)
    gt = RS.graph_tensors(g)
    nk = a.robot_max
    lay = RS.chain_route(gt, torch.zeros(n, a.robot_routes + 1, dtype=torch.long), torch.ones(n),
                         torch.zeros(n, a.robot_routes))
    env = type("FakeEnv", (), methods)()
    env.__dict__.update(
        arena=a, device="cpu", num_envs=n, graph=g, g_t=gt, robot_objs=[None] * nk,
        r_sg_node=torch.tensor([[g.names.index("E-1"), g.names.index("Q2")]]),
        _new_scen=torch.full((n,), -1), scen=None, scen_names=(), ghost=False, cap_active=False,
        v_cap=torch.full((n,), ts.NAV_SPEED_MPS), robot_park=torch.zeros(nk, 2),
        robot_routes=[{k: v.clone() for k, v in lay.items()} for _ in range(nk)],
        robot_active=torch.zeros(nk, n, dtype=torch.bool), robot_t=torch.zeros(nk, n),
        robot_wait=torch.zeros(nk, n), robot_back=torch.zeros(nk, n),
        robot_priority=torch.zeros(nk, n, dtype=torch.bool), start_xy=torch.full((n, 2), 1.0),
        spec=SimpleNamespace(speed_cap_range=(ts.NAV_SPEED_MPS, ts.NAV_SPEED_MPS)),
        cfg=SimpleNamespace(episode_length_s=a.episode_s),
    )
    if "_cleaner_init" in methods:
        env._cleaner_init(nk, lay, site)
    return env


@pytest.fixture(scope="module")
def env_src():
    with open(os.path.join(HERE, "store_arena_env.py"), encoding="utf-8") as f:
        return f.read()


def _run(env, seed=1):
    torch.manual_seed(seed)
    env._sample_robots(torch.arange(env.num_envs), torch.zeros(env.num_envs, dtype=torch.long))
    return torch.rand(4)  # 뒤 난수열


def test_default_is_unchanged_draws_and_routes(env_src):
    """기본값(청소 0, robot_min 0)에서 바꾸기 전 _sample_robots 와 결과·난수열이 같다."""
    old_src = _git_show(f"{BASE}:learning/store_arena/store_arena_env.py")
    if old_src is None:
        pytest.skip("base commit not available")
    new = _fake_env(_methods(env_src, ENV_METHODS))
    old = _fake_env(_methods(old_src, {"_sample_robots", "robot_state"}))
    assert new.cleaner is None and not new.cleaner_on.any()
    tail_new, tail_old = _run(new), _run(old)
    assert torch.equal(tail_new, tail_old)
    assert torch.equal(new.robot_active, old.robot_active)
    assert torch.equal(new.robot_t, old.robot_t)
    for rn, ro in zip(new.robot_routes, old.robot_routes, strict=True):
        assert rn.keys() == ro.keys()
        for k in rn:
            assert rn[k].shape == ro[k].shape and torch.equal(rn[k], ro[k])


def test_robot_min_forces_the_count(env_src):
    env = _fake_env(_methods(env_src, ENV_METHODS), robot_min=2)
    _run(env)
    assert env.robot_active.all()
    env = _fake_env(_methods(env_src, ENV_METHODS), robot_min=1)
    _run(env)
    cnt = env.robot_active.sum(0)
    assert cnt.min() == 1 and cnt.max() == 2
    with pytest.raises(ValueError):
        _fake_env(_methods(env_src, ENV_METHODS), robot_min=3)


def test_two_robots_one_transport_one_cleaner(env_src, lap):
    env = _fake_env(_methods(env_src, ENV_METHODS), robot_min=2, robot_cleaner_prob=1.0)
    _run(env)
    n = env.num_envs
    assert env.cleaner_on.all() and env.robot_active.all()
    transport, cleaner = env.robot_routes
    assert (transport["vnode"] >= 0).all()  # 칸 0: 그래프
    assert (cleaner["vnode"] == -1).all()  # 칸 1: 청소 바퀴
    pos, _, _ = RS.robot_pose(cleaner, env.robot_t[1])
    d_start = (pos - env.start_xy).norm(dim=-1)
    assert (d_start >= scn.START_SEP_M).float().mean() > 0.95
    # 출발 시각은 한 바퀴 안에 고르게
    lap_s = lap.length / C.SPEED_MPS
    t = env.robot_t[1]
    assert (t >= 0).all() and (t < lap_s).all() and t.std() > lap_s / 5
    assert not env.robot_priority[1].any()
    # 비키기 값: 청소 칸만 clean_yield 값과 가장 가까운 로봇
    xy = torch.full((n, 2), 5.0)
    watch, ym, ws = env._cleaner_yield(xy, torch.full((n,), 0.25), 3.0)
    tpos, _, _ = RS.robot_pose(transport, env.robot_t[0])
    assert torch.allclose(watch, tpos)
    assert (ym == 0.30).all() and (ws == 10.0).all()


def test_cleaner_prob_mixes_slots(env_src):
    env = _fake_env(_methods(env_src, ENV_METHODS), robot_cleaner_prob=0.5)
    _run(env)
    on = env.cleaner_on
    assert 0.3 < on.float().mean() < 0.7
    act = env.robot_active
    assert act[1, on].all()  # 청소 env 는 마지막 칸이 켜짐
    cleaner = env.robot_routes[1]
    assert (cleaner["vnode"][on] == -1).all() and (cleaner["vnode"][~on] >= 0).all()
    # 청소가 아닌 env 의 그래프 경로는 끝 꼭짓점을 되풀이해 늘렸을 뿐: 위치가 같음
    t = torch.linspace(0, 200, (~on).sum().item())
    sub = {k: v[~on] for k, v in cleaner.items()}
    v0 = env.robot_routes[0]["pts"].shape[1]
    short = {k: v[:, : v0 - (1 if k == "heading" else 0)] for k, v in sub.items()}
    assert torch.allclose(RS.robot_pose(sub, t)[0], RS.robot_pose(short, t)[0])


def test_ghost_slot_is_avoided_at_its_partner_start(env_src):
    env = _fake_env(_methods(env_src, ENV_METHODS), robot_cleaner_prob=1.0)
    env.ghost, env._new_gprio = True, torch.zeros(env.num_envs, dtype=torch.bool)
    env.start_xy = torch.rand(env.num_envs, 2) * torch.tensor([1.9, 0.9]) + 0.04
    _run(env)
    pos, _, _ = RS.robot_pose(env.robot_routes[1], env.robot_t[1])
    partner = env.start_xy[gh.partner(torch.arange(env.num_envs))]
    assert ((pos - partner).norm(dim=-1) >= scn.START_SEP_M).float().mean() > 0.95


def test_cleaner_needs_a_free_slot(env_src):
    with pytest.raises(ValueError):
        _fake_env(_methods(env_src, ENV_METHODS), robot_max=1, robot_cleaner_prob=1.0,
                  robot_headon=1)

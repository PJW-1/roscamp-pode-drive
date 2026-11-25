"""움직이는 것 근처 보상 항 테스트 (Isaac 없이): 끄면 0, 손으로 만든 경우의 부호와 크기.
실행: python -m pytest learning/store_arena -q"""

import dataclasses
import inspect
import os
import sys

import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import social_reward as S  # noqa: E402

INF = float("inf")


def test_defaults_are_off_and_settable():
    a = A.ARENA
    for f in ("w_near_dyn", "w_yield", "w_block", "near_dyn_m", "yield_m", "block_m"):
        assert getattr(a, f) == 0.0
    task, arena = A.split_overrides(
        ["near_dyn_m=0.1", "w_near_dyn=-0.2", "yield_m=0.2", "w_yield=0.05", "block_m=0.1",
         "w_block=-0.1", "block_v_mps=0.03"]
    )
    assert task == {}
    spec = A.make_arena_spec(arena)
    assert (spec.near_dyn_m, spec.w_near_dyn, spec.w_yield, spec.block_v_mps) == (
        0.1, -0.2, 0.05, 0.03)


def test_near_dyn_is_linear_inside_and_zero_outside_or_off():
    gap = torch.tensor([0.0, 0.05, 0.1, 0.2, INF])
    p = S.near_dyn_penalty(gap, 0.1, -0.2)
    assert p.tolist() == pytest.approx([-0.2, -0.1, 0.0, 0.0, 0.0])
    assert (S.near_dyn_penalty(gap, 0.0, -0.2) == 0).all()
    assert (S.near_dyn_penalty(gap, 0.1, 0.0) == 0).all()


def yield_case(rob_prev, rob_now, our_prev, our_now, prio=True, gap=0.1, yield_m=0.2, w=0.5):
    t = torch.tensor
    return float(S.yield_reward(
        t([[gap]]), t([[prio]]), t([[rob_prev]]), t([[rob_now]]), t([our_prev]), t([our_now]),
        yield_m, w,
    )[0])


def test_yield_rewards_waiting_or_backing_off_from_an_approaching_priority_robot():
    # 로봇이 +x 에서 다가옴 (0.30 -> 0.29)
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0, 0]) == 0.5  # 기다림
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [-0.01, 0]) == 0.5  # 물러남
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0.01, 0]) == 0.0  # 다가감
    # 옆으로 비켜도 간격이 안 줄면 보상
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0, 0.01]) == 0.5


def test_yield_needs_priority_proximity_and_approach():
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0, 0], prio=False) == 0.0
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0, 0], gap=0.25) == 0.0
    assert yield_case([0.30, 0], [0.30, 0], [0, 0], [0, 0]) == 0.0  # 로봇이 서 있음
    assert yield_case([0.30, 0], [0.31, 0], [0, 0], [0, 0]) == 0.0  # 멀어짐
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0, 0], yield_m=0.0) == 0.0
    assert yield_case([0.30, 0], [0.29, 0], [0, 0], [0, 0], w=0.0) == 0.0


def test_yield_must_keep_the_gap_to_every_approaching_priority_robot():
    t = torch.tensor
    gap = t([[0.1, 0.1]])
    prio = t([[True, True]])
    rob_prev = t([[[0.30, 0.0], [-0.30, 0.0]]])
    rob_now = t([[[0.29, 0.0], [-0.29, 0.0]]])
    # +x 쪽에서 물러나면 -x 로봇에 다가감
    r = S.yield_reward(gap, prio, rob_prev, rob_now, t([[0.0, 0.0]]), t([[-0.01, 0.0]]), 0.2, 0.5)
    assert float(r[0]) == 0.0
    r = S.yield_reward(gap, prio, rob_prev, rob_now, t([[0.0, 0.0]]), t([[0.0, 0.0]]), 0.2, 0.5)
    assert float(r[0]) == 0.5


def test_block_needs_both_still_and_close():
    t = torch.tensor
    gap = t([[0.05], [0.05], [0.05], [0.2], [INF]])
    v_ours = t([0.0, 0.1, 0.0, 0.0, 0.0])
    v_rob = t([[0.0], [0.0], [0.1], [0.0], [0.0]])
    r = S.block_penalty(gap, v_ours, v_rob, 0.1, 0.02, -0.1)
    assert r.tolist() == pytest.approx([-0.1, 0.0, 0.0, 0.0, 0.0])
    assert (S.block_penalty(gap, v_ours, v_rob, 0.0, 0.02, -0.1) == 0).all()
    assert (S.block_penalty(gap, v_ours, v_rob, 0.1, 0.02, 0.0) == 0).all()


def test_no_robots_gives_zero():
    z = torch.zeros(3, 0)
    assert (S.block_penalty(z, torch.zeros(3), z, 0.1, 0.02, -1.0) == 0).all()
    e2 = torch.zeros(3, 0, 2)
    r = S.yield_reward(z, z.bool(), e2, e2, torch.zeros(3, 2), torch.zeros(3, 2), 0.2, 1.0)
    assert (r == 0).all()


def test_terms_do_not_see_episode_outcomes():
    """끝 보상(성공/충돌)은 task_spec.compute_reward 몫: 이 항들은 결과를 입력으로 받지 않음."""
    for fn in (S.near_dyn_penalty, S.yield_reward, S.block_penalty):
        names = set(inspect.signature(fn).parameters)
        assert not names & {"success", "collided", "terminal", "done"}
    assert dataclasses.is_dataclass(A.ARENA)

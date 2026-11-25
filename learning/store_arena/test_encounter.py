"""만남 지표 테스트 (Isaac 없이): 최소 간격, 첫 만남 시간, 교착.
실행: python -m pytest learning/store_arena -q"""

import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import encounter as E  # noqa: E402

DT = 0.05
INF = float("inf")


def run(gaps, v_ours, v_rob):
    """스텝별 값 목록으로 env 하나, 로봇 하나 상태를 돌림."""
    st = E.new_state(1, 1)
    for g, vo, vr in zip(gaps, v_ours, v_rob, strict=True):
        E.update(st, torch.tensor([[g]]), torch.tensor([vo]), torch.tensor([[vr]]), DT)
    return st


def test_min_gap_and_first_encounter_only():
    near, far = 0.5 * E.ENCOUNTER_M, 2 * E.ENCOUNTER_M
    gaps = [far] * 3 + [near] * 4 + [far] * 2 + [near] * 10  # 둘째 만남은 안 셈
    st = run(gaps, [0.1] * len(gaps), [0.1] * len(gaps))
    assert float(st["min_gap"][0]) == pytest.approx(near)
    assert bool(st["enc_seen"][0])
    assert float(st["enc_s"][0]) == pytest.approx(4 * DT)
    assert not bool(st["deadlock"][0])


def test_encounter_runs_to_the_episode_end():
    gaps = [INF, 0.0, 0.01, 0.02]
    st = run(gaps, [0.1] * 4, [0.1] * 4)
    assert float(st["enc_s"][0]) == pytest.approx(3 * DT)
    assert float(st["min_gap"][0]) == 0.0


def test_no_robot_is_no_encounter():
    st = run([INF] * 5, [0.0] * 5, [0.0] * 5)
    assert not bool(st["enc_seen"][0]) and float(st["min_gap"][0]) == INF
    assert not bool(st["deadlock"][0])


def test_deadlock_needs_both_still_close_for_longer_than_the_limit():
    steps = int(round(E.DEADLOCK_S / DT))
    near = 0.5 * E.ENCOUNTER_M
    st = run([near] * steps, [0.0] * steps, [0.0] * steps)
    assert not bool(st["deadlock"][0])  # 딱 DEADLOCK_S 는 아직 아님
    st = run([near] * (steps + 2), [0.0] * (steps + 2), [0.0] * (steps + 2))
    assert bool(st["deadlock"][0])
    # 로봇이 한 번 움직이면 다시 셈
    v = [0.0] * steps + [0.1] + [0.0] * 5
    st = run([near] * len(v), [0.0] * len(v), v)
    assert not bool(st["deadlock"][0])
    # 우리가 움직이면 교착 아님
    st = run([near] * (steps + 5), [0.1] * (steps + 5), [0.0] * (steps + 5))
    assert not bool(st["deadlock"][0])


def test_deadlock_is_per_robot():
    """멈춘 로봇 0 은 멀고 가까운 로봇 1 은 움직이면 교착 아님."""
    st = E.new_state(1, 2)
    steps = int(E.DEADLOCK_S / DT) + 5
    for _ in range(steps):
        E.update(st, torch.tensor([[1.0, 0.05]]), torch.tensor([0.0]),
                 torch.tensor([[0.0, 0.1]]), DT)
    assert not bool(st["deadlock"][0])


def test_reset_and_snapshot():
    st = E.new_state(2, 1)
    E.update(st, torch.tensor([[0.0], [1.0]]), torch.zeros(2), torch.zeros(2, 1), DT)
    snap = E.snapshot(st, torch.tensor([0]))
    E.reset(st, torch.tensor([0]))
    assert float(snap["min_gap"][0]) == 0.0 and bool(snap["enc_seen"][0])
    assert float(st["min_gap"][0]) == INF and not bool(st["enc_seen"][0])
    assert float(st["min_gap"][1]) == 1.0


def test_summary():
    out = E.summary([INF, 0.0, 0.1, 0.3], [0.0, 2.0, 1.0, 0.0], [False, True, True, False],
                    [False, True, False, False])
    assert out["min_robot_gap_m"]["episodes_with_robot"] == 3
    assert out["min_robot_gap_m"]["share_touching"] == pytest.approx(1 / 3, abs=1e-4)
    assert out["min_robot_gap_m"]["share_under_encounter"] == pytest.approx(2 / 3, abs=1e-4)
    assert out["encounter_s"]["episodes"] == 2 and out["encounter_s"]["mean"] == 1.5
    assert out["deadlocks"] == 1 and out["deadlock_rate"] == 0.25
    empty = E.summary(np.array([]), np.array([]), np.array([]), np.array([]))
    assert empty["deadlocks"] == 0 and empty["min_robot_gap_m"]["mean"] is None

import json

import cross_warning as cw
import numpy as np

WALL = (0.2, -1.0, 0.5, -0.1)  # 로봇과 숨은 사람 사이를 막는 진열대
HIDDEN, SHOWN, RESET = (0.6, -0.5), (0.6, 0.0), (0.35, -2.0)


def fake_run(path, shown_from, robot_v, hit=("mover",) * 5, on_path=True, end=100, code=1,
             stop_from=None, untracked_hits=0):
    """env마다 end 스텝에서 끝나는 에피소드 하나. 사람은 shown_from 스텝부터 보이는 자리에 있다.
    로봇은 stop_from 스텝부터 서 있고, 끝난 스텝부터는 리셋 자세(진열대 뒤)다. episodes.npz는 env 순서를 뒤집어
    적고, 추적하지 않은 env의 사람 충돌 untracked_hits건을 더 적는다."""
    steps, envs, dt = 120, len(shown_from), 0.05
    xy = np.zeros((steps, envs, 2), dtype=np.float32)
    moved = np.minimum(np.arange(steps), steps if stop_from is None else stop_from)
    xy[:, :, 0] = moved[:, None] * dt * np.array(robot_v)[None, :]
    xy[end:] = RESET
    mover = np.zeros((steps, envs, 2, 2), dtype=np.float32)
    mover[:, :, 1] = (9.0, 9.0)
    for e, k in enumerate(shown_from):
        mover[:k, e, 0], mover[k:, e, 0] = HIDDEN, SHOWN
    on = np.zeros((steps, envs, 2), dtype=bool)
    on[:, :, 0] = True
    done = np.zeros((steps, envs), dtype=bool)
    done[end] = True
    outcome = np.full((steps, envs), -1)
    outcome[end] = code
    path.mkdir(parents=True, exist_ok=True)
    np.savez(path / "rollouts.npz", xy=xy, done=done, outcome=outcome, mover_xy=mover, mover_on=on,
             route=np.zeros((steps, envs), dtype=np.int64), boxes=np.array([WALL]),
             obstacle_xy=np.full((1, 2, 2), np.nan), obstacle_sizes=np.array([0.2, 0.1]), dt=dt)
    kinds = ["wall_shelf", "box", "mover", "robot"]
    order = np.arange(envs + untracked_hits)[::-1]
    np.savez(path / "episodes.npz", env=order, code=np.full(len(order), code),
             keys=np.array(["success", "collision", "oob", "timeout"]),
             hit_kind=np.array([kinds.index(hit[e] if e < envs else "mover") for e in order]),
             hit_kinds=np.array(kinds), cross_path=np.full(len(order), on_path))
    (path / "eval.json").write_text(json.dumps({"rates": {"collision": 1.0}}), encoding="utf-8")


def test_segment_against_a_rect():
    assert cw.blocked(np.array([0.0, 0.0]), np.array(HIDDEN), WALL)
    assert not cw.blocked(np.array([0.0, 0.0]), np.array(SHOWN), WALL)
    assert not cw.blocked(np.array([0.0, -2.0]), np.array([0.1, -0.5]), WALL)
    assert cw.blocked(np.array([0.35, -2.0]), np.array([0.35, 0.5]), WALL)


def test_a_person_sticking_out_past_the_shelf_end_counts_as_visible():
    robot = np.array([0.0, 0.0])
    edge = (0.2, -0.005, 0.5, 0.5)  # 중심선은 막지만 아래쪽 끝은 보인다
    assert cw.blocked(robot, np.array([0.6, 0.0]), edge)
    assert cw.visible(robot, np.array([0.6, 0.0]), [edge])
    assert not cw.visible(robot, np.array(HIDDEN), [WALL])


def test_warning_time_and_class_per_episode(tmp_path):
    fake_run(tmp_path, shown_from=[90, 20, 90, 60, 60], robot_v=[0.04, 0.04, 0.0, 0.04, 0.04],
             hit=("mover", "mover", "mover", "mover", "box"))
    rows = cw.rows(tmp_path)
    assert [r["env"] for r in rows] == [0, 1, 2, 3]
    assert [r["kind"] for r in rows] == ["late_sight", "seen_long", "robot_stopped", "seen_short"]
    assert [round(r["warning_s"], 2) for r in rows] == [0.5, 4.0, 0.5, 2.0]
    assert abs(rows[0]["robot_mps"] - 0.04) < 1e-3 and rows[2]["robot_mps"] == 0.0


def test_a_robot_that_stopped_before_the_contact_counts_as_stopped(tmp_path):
    fake_run(tmp_path, shown_from=[90], robot_v=[0.04], stop_from=80)
    row = cw.rows(tmp_path)[0]
    assert row["kind"] == "robot_stopped" and row["robot_mps"] == 0.0


def test_only_person_collisions_with_the_crossing_on_the_path_are_listed(tmp_path):
    fake_run(tmp_path, shown_from=[90], robot_v=[0.04], on_path=False)
    assert cw.rows(tmp_path) == []
    fake_run(tmp_path, shown_from=[90], robot_v=[0.04], code=0)
    assert cw.rows(tmp_path) == []


def test_verdict_needs_twenty_collisions_and_more_than_half():
    base = dict.fromkeys(cw.CLASSES, 0)
    assert cw.verdict({**base, "late_sight": 19}).startswith("not judged")
    assert cw.verdict({**base, "late_sight": 11, "seen_long": 9}) == "mostly late_sight"
    assert cw.verdict({**base, "late_sight": 10, "seen_long": 10}) == "no majority"
    assert cw.verdict({**base, "seen_short": 5, "robot_stopped": 16}) == "mostly robot_stopped"


def test_check_counts_person_collisions_among_the_tracked_envs(tmp_path):
    fake_run(tmp_path / "new", shown_from=[90, 90], robot_v=[0.04, 0.04], untracked_hits=3)
    fake_run(tmp_path / "old", shown_from=[90, 90], robot_v=[0.04, 0.04], hit=("box", "mover"))
    assert cw.check(tmp_path / "new", tmp_path / "old") == {"rates_equal": True, "person_hits_in_tracked_envs": [2, 1]}
    assert cw.check(tmp_path / "old", tmp_path / "new")["person_hits_in_tracked_envs"] == [1, 2]

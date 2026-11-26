import json

import numpy as np
import timeout_motion as tm


def fake_run(path, cmd_w, front, code=3, moved=0.0, name="rollouts.npz", ends=None):
    """env마다 ends의 (스텝, 결과)에서 끝나는 에피소드들(기본은 40스텝에서 code로 끝나는 하나).
    cmd_w, front, moved는 env별 값이다. 경로 번호는 에피소드 순서이고, 끝난 스텝까지 그 에피소드의 번호다(play.py는
    스텝 전의 경로를 기록한다)."""
    steps, envs = 50, len(cmd_w)
    ends = ends or ((40, code),)
    xy = np.zeros((steps, envs, 2), dtype=np.float32)
    xy[..., 0] = 1.0 + np.linspace(0, 1, steps)[:, None] * np.broadcast_to(np.asarray(moved, dtype=float), (envs,))
    done = np.zeros((steps, envs), dtype=bool)
    outcome = np.full((steps, envs), -1)
    route = np.zeros((steps, envs), dtype=np.int64)
    for step, result in ends:
        done[step], outcome[step] = True, result
        route[step + 1:] += 1
    cmd = np.zeros((steps, envs, 2), dtype=np.float32)
    cmd[..., 1] = np.array(cmd_w)[None, :]
    path.mkdir(parents=True, exist_ok=True)
    np.savez(path / name, xy=xy, done=done, outcome=outcome, cmd=cmd,
             front_m=np.tile(np.array(front, dtype=np.float32), (steps, 1)),
             route=route, obstacle_sizes=np.array([0.2, 0.1]), dt=0.1,
             obstacle_xy=np.array([[[1.5 + 3.5 * r, 0.0], [np.nan, np.nan]] for r in range(len(ends) + 1)]))


def test_turning_in_place_next_to_a_box_is_not_counted_as_standing(tmp_path):
    fake_run(tmp_path, cmd_w=[0.0, 1.0], front=[0.1, 0.1])
    rows = tm.rows(tmp_path)
    assert [r["motion"] for r in rows] == ["stopped", "dithering"]
    assert [r["front_near"] for r in rows] == [True, True]


def test_only_timeouts_of_the_first_episode_are_listed(tmp_path):
    fake_run(tmp_path, cmd_w=[0.0, 0.0], front=[1.0, 1.0], code=0)
    assert tm.rows(tmp_path) == []
    fake_run(tmp_path, cmd_w=[0.0], front=[1.0], moved=[1.0])
    rows = tm.rows(tmp_path)
    assert [r["motion"] for r in rows] == ["moving"] and rows[0]["front_near"] is False


def test_a_timeout_after_the_first_episode_is_not_listed(tmp_path):
    fake_run(tmp_path, cmd_w=[0.0], front=[1.0], ends=((20, 0), (45, 3)))
    assert tm.rows(tmp_path) == []


def test_the_box_is_the_one_of_the_episode_not_of_its_neighbours(tmp_path):
    fake_run(tmp_path, cmd_w=[0.0], front=[1.0], ends=((5, 0), (40, 3)))
    assert tm.rows(tmp_path) == []
    fake_run(tmp_path, cmd_w=[0.0], front=[1.0])
    assert abs(tm.rows(tmp_path)[0]["box_m"] - 0.4) < 1e-6


def test_box_distance_is_to_the_nearest_edge_and_skips_unused_slots():
    centers = np.array([[1.5, 0.0], [np.nan, np.nan]])
    assert abs(tm.box_distance(np.array([1.0, 0.0]), centers, np.array([0.2, 0.1])) - 0.4) < 1e-9
    assert tm.box_distance(np.array([1.5, 0.05]), centers, np.array([0.2, 0.1])) == 0.0
    assert np.isnan(tm.box_distance(np.array([1.0, 0.0]), np.full((2, 2), np.nan), np.array([0.2, 0.1])))


def test_verdict_needs_twenty_timeouts_and_more_than_half():
    assert tm.verdict({"stopped": 19, "dithering": 0, "moving": 0}).startswith("not judged")
    assert tm.verdict({"stopped": 11, "dithering": 9, "moving": 0}) == "mostly stopped"
    assert tm.verdict({"stopped": 10, "dithering": 10, "moving": 0}) == "no majority"
    assert tm.verdict({"stopped": 5, "dithering": 4, "moving": 12}) == "mostly moving"


def test_check_compares_rates_and_timeouts_among_the_tracked_envs(tmp_path):
    for name, codes in (("new", [3, 0, 3, 3]), ("old", [0, 0, 3, 3])):
        d = tmp_path / name
        fake_run(d, cmd_w=[0.0, 0.0], front=[1.0, 1.0])
        np.savez(d / "episodes.npz", env=np.arange(4), code=np.array(codes),
                 keys=np.array(["success", "collision", "oob", "timeout"]))
        (d / "eval.json").write_text(json.dumps({"rates": {"timeout": codes.count(3) / 4}}), encoding="utf-8")
    out = tm.check(tmp_path / "new", tmp_path / "old")
    assert out == {"rates_equal": False, "timeouts_in_tracked_envs": [1, 0]}
    assert tm.check(tmp_path / "old", tmp_path / "new")["timeouts_in_tracked_envs"] == [0, 1]
    assert tm.check(tmp_path / "new", tmp_path / "new")["rates_equal"] is True

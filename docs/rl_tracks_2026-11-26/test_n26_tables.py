import json
import os

import n26_tables as nt
import numpy as np
import pytest

KEYS = np.array(["success", "collision", "oob", "timeout"])


def fake_eval(runs, tag, code, route_shift=0):
    d = runs / tag
    os.makedirs(d)
    code = np.array(code)
    n = len(code)
    np.savez(d / "episodes.npz", env=np.arange(n), code=code, keys=KEYS,
             flips=np.zeros(n), length_s=np.full(n, 10.0))
    np.savez(d / "rollouts.npz", route=np.arange(4)[None, :] + route_shift + np.zeros((3, 4), int),
             mover_xy=np.zeros((3, 4, 2, 2)), robot_xy=np.zeros((3, 4, 2, 2)))
    rates = {k: float((code == i).mean()) for i, k in enumerate(KEYS)}
    json.dump({"episodes": n, "rates": rates,
               "collisions_by_nearest": {"wall_shelf": 0, "box": 0, "mover": 0,
                                         "robot": int((code == 1).sum())},
               "collisions_graze_only": 0, "w_flips_per_min": 40.0,
               "success_time_over_path_at_vmax_mean": 1.2}, open(d / "eval.json", "w"))


def status(runs, lines):
    (runs / "queue_status.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_missing_and_unfinished_runs_are_skipped(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [0, 1, 0, 3])
    fake_eval(tmp_path, "e_std_rand", [0, 0, 0, 3])  # 폴더는 있지만 마지막 줄이 실패
    status(tmp_path, ["e_ctrl_rand exit=0 wall_s=1", "e_std_rand exit=0 wall_s=1",
                      "e_std_rand exit=1 wall_s=2", "e_sysid_rand exit=0 wall_s=3"])
    out = tmp_path / "out"
    t = nt.main([str(tmp_path), str(out)])
    assert [(r["policy"], r["cell"]) for r in t["cells"]] == [("ctrl", "rand")]
    assert t["paired"] == [] and t["regression"] == [] and t["training"] == []
    assert (out / "tables.md").exists()


def test_malformed_last_line_and_truncated_files_skip_the_run(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [0, 1, 0, 3])
    fake_eval(tmp_path, "e_std_rand", [0, 0, 0, 3])
    fake_eval(tmp_path, "e_acd3_rand", [0, 0, 0, 3])
    (tmp_path / "e_acd3_rand" / "episodes.npz").write_bytes(b"PK\x03\x04trunc")
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0", "e_std_rand exit",
                      "e_acd3_rand exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    assert [(r["policy"], r["cell"]) for r in t["cells"]] == [("ctrl", "rand")]
    assert "e_acd3_rand" in (tmp_path / "out" / "tables.md").read_text(encoding="utf-8")


def test_paired_row_arithmetic(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [1, 1, 0, 3])
    fake_eval(tmp_path, "e_std_rand", [0, 0, 0, 0])
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0"])
    (row,) = nt.main([str(tmp_path), str(tmp_path / "out")])["paired"]
    assert (row["policy"], row["vs_policy"]) == ("std", "ctrl")
    assert row["same_episodes"] == "yes (4 tracked envs)"
    d = np.array([1, 1, 0, 1.0])  # std 성공 - ctrl 성공
    assert row["success_diff_pp"] == pytest.approx(75.0)
    assert row["success_se_pp"] == pytest.approx(round(100 * d.std(ddof=1) / 2, 4))
    assert row["collision_diff_pp"] == pytest.approx(-50.0)
    assert row["timeout_diff_pp"] == pytest.approx(-25.0)


def test_mismatched_episodes_are_reported_not_paired(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [1, 1, 0, 3])
    fake_eval(tmp_path, "e_std_rand", [0, 0, 0, 0], route_shift=1)
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0"])
    (row,) = nt.main([str(tmp_path), str(tmp_path / "out")])["paired"]
    assert row["same_episodes"] == "different start episodes"
    assert "success_diff_pp" not in row


def test_training_diff_and_std_curve(tmp_path):
    torch = pytest.importorskip("torch")
    for tag, std, extra in (("t_ctrl", 0.0, {}), ("t_std", 0.3, {"gamma": 0.995})):
        d = tmp_path / tag
        os.makedirs(d)
        cfg = {"init": "x/model_100.pt", "init_std": [std, std], "seed": 3, "agent": extra}
        json.dump(cfg, open(d / "run_config.json", "w"))
        (d / "metrics.csv").write_text(
            "wall_s,iteration,success_rate,collision_rate,timeout_rate,success_time_s\n"
            "1,1,0.1,0.2,0.7,9\n2,50,0.5,0.4,0.1,10\n", encoding="utf-8")
        torch.save({"actor_state_dict": {"distribution.std_param": torch.tensor([0.2, 0.1])}},
                   d / "model_150.pt")
    status(tmp_path, ["t_ctrl exit=0 wall_s=11", "t_std exit=0 wall_s=12"])
    rows, curve = nt.training(str(tmp_path), nt.finished(str(tmp_path)))
    assert rows[1]["differs_from_t_ctrl"] == (
        "agent.gamma=0.995 (ctrl -); init_std=[0.3, 0.3] (ctrl [0.0, 0.0])")
    assert rows[1]["wall_s"] == "12"
    std = [r for r in curve if r["run"] == "t_std"]
    assert [(r["iteration"], r["success_rate"], r["std_v"]) for r in std] == [
        (0, "", 0.3), (50, "0.5", 0.2)]


def test_regression_skips_an_episodes_file_without_length(tmp_path):
    fake_eval(tmp_path, "e_acd3_rand", [0, 1, 0, 3])
    fake_eval(tmp_path, "e_acd3_rand_chk", [0, 0, 0, 1])
    np.savez(tmp_path / "e_acd3_rand_chk" / "episodes.npz", env=np.arange(4),
             code=np.array([0, 0, 0, 1]), keys=KEYS)
    status(tmp_path, ["e_acd3_rand exit=0", "e_acd3_rand_chk exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    assert t["regression"] == []
    assert "e_acd3_rand_chk" in (tmp_path / "out" / "tables.md").read_text(encoding="utf-8")


def test_regression_row(tmp_path):
    fake_eval(tmp_path, "e_acd3_rand", [0, 1, 0, 3])
    fake_eval(tmp_path, "e_acd3_rand_chk", [0, 0, 0, 1])
    status(tmp_path, ["e_acd3_rand exit=0", "e_acd3_rand_chk exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    (row,) = t["regression"]
    assert row["code_differs"] == 2 and row["length_differs"] == 0
    assert row["success_diff_pp"] == pytest.approx(25.0)  # chk - rand
    assert row["from_collision_changed"] == 1 and row["from_timeout_changed"] == 1
    assert ("acd3", "rand_chk") in {(r["policy"], r["cell"]) for r in t["cells"]}

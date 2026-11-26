import csv
import json
import os

import n26_tables as nt
import numpy as np
import pytest

KEYS = np.array(["success", "collision", "oob", "timeout"])


def fake_eval(runs, tag, code, route_shift=0, npz=None, extra=None):
    d = runs / tag
    os.makedirs(d)
    code = np.array(code)
    n = len(code)
    np.savez(d / "episodes.npz", env=np.arange(n), code=code, keys=KEYS,
             flips=np.zeros(n), length_s=np.full(n, 10.0), **(npz or {}))
    np.savez(d / "rollouts.npz", route=np.arange(4)[None, :] + route_shift + np.zeros((3, 4), int),
             mover_xy=np.zeros((3, 4, 2, 2)), robot_xy=np.zeros((3, 4, 2, 2)))
    rates = {k: float((code == i).mean()) for i, k in enumerate(KEYS)}
    json.dump({"episodes": n, "rates": rates,
               "collisions_by_nearest": {"wall_shelf": 0, "box": 0, "mover": 0,
                                         "robot": int((code == 1).sum())},
               "collisions_graze_only": 0, "w_flips_per_min": 40.0,
               "success_time_over_path_at_vmax_mean": 1.2, **(extra or {})},
              open(d / "eval.json", "w"))


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


OLD_PAIR_KEYS = {"policy", "cell", "vs_policy", "vs_cell", "n", "same_episodes",
                 "success_diff_pp", "success_se_pp", "success_noise", "collision_diff_pp",
                 "collision_se_pp", "collision_noise", "timeout_diff_pp", "timeout_se_pp",
                 "timeout_noise"}
KINDS = np.array(["wall_shelf", "box", "mover", "robot"])
GAPS = [np.nan, 0.14, 0.15, 0.20, 0.30, np.nan]


def read_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def test_runs_without_new_keys_keep_the_old_output(tmp_path):
    fake_eval(tmp_path, "e_ctrl_aisle", [1, 1, 0, 3])
    fake_eval(tmp_path, "e_demo_aisle", [0, 0, 0, 0])
    status(tmp_path, ["e_ctrl_aisle exit=0", "e_demo_aisle exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    (row,) = t["paired"]
    assert (row["policy"], row["vs_policy"], row["cell"]) == ("demo", "ctrl", "aisle")
    assert set(row) == OLD_PAIR_KEYS
    assert t["gap"] == [] and t["gap_paired"] == [] and t["crossing"] == []


def test_new_comparisons_are_listed():
    c = set(nt.COMPARISONS)
    for cell in ("aisle", "demo1", "demo2"):
        for a, b in (("demo", "ctrl"), ("demo", "acd3"), ("demostd", "std"), ("demostd", "demo"),
                     ("ctrl", "acd3"), ("std", "acd3"), ("std", "ctrl")):
            assert (a, cell, b, cell) in c
    for cell in ("rand", "head"):
        for a, b in (("demo", "ctrl"), ("demostd", "std"), ("demostd", "demo"),
                     ("demo", "acd3"), ("demostd", "acd3")):
            assert (a, cell, b, cell) in c
    for cell in ("rand", "head", "aisle"):
        for a, b in (("sym", "ctrl"), ("sym", "acd3")):
            assert (a, cell, b, cell) in c
    for cell in ("demo1", "demo2", "aisle", "rand", "head"):
        for a, b in (("demohist", "demostd"), ("demohist", "acd3"), ("demohist", "std")):
            assert (a, cell, b, cell) in c
        for a, b in (("histstd", "std"), ("histstd", "demohist"), ("histstd", "acd3")):
            assert (a, cell, b, cell) in c
    for cell in ("demo1", "aisle", "rand", "head"):
        for a, b in (("stds4", "std"), ("demohists4", "demohist"), ("demohist", "stds4"),
                     ("demohists4", "std"), ("demohists4", "stds4"), ("stds4", "ctrl")):
            assert (a, cell, b, cell) in c
    for pol in ("acd3", "std", "demohist"):
        assert (pol, "demo1c08", pol, "demo1") in c
    for a, b in (("std", "acd3"), ("demohist", "acd3"), ("demohist", "std")):
        assert (a, "demo1c08", b, "demo1c08") in c
    assert len(c) == len(nt.COMPARISONS)


def test_collision_kind_paired_diff_and_missing_key(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [1, 1, 0, 1],
              npz={"hit_kind": np.array([3, 1, 0, 2]), "hit_kinds": KINDS})
    fake_eval(tmp_path, "e_std_rand", [1, 0, 0, 0],
              npz={"hit_kind": np.array([1, 0, 0, 0]), "hit_kinds": KINDS})
    fake_eval(tmp_path, "e_acd3_rand", [0, 0, 0, 0])
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0", "e_acd3_rand exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    rows = {(r["policy"], r["vs_policy"]): r for r in t["paired"]}
    r = rows[("std", "ctrl")]
    assert r["coll_robot_diff_pp"] == pytest.approx(-25.0)
    assert r["coll_mover_diff_pp"] == pytest.approx(-25.0)
    assert r["coll_box_diff_pp"] == pytest.approx(0.0)
    assert r["coll_box_se_pp"] == pytest.approx(round(100 * np.sqrt(2 / 3) / 2, 4))
    assert (r["coll_wall_shelf_diff_pp"], r["coll_wall_shelf_noise"]) == (0.0, "inside")
    assert set(rows[("ctrl", "acd3")]) == OLD_PAIR_KEYS
    csv_rows = {(c["policy"], c["vs_policy"]): c
                for c in read_csv(tmp_path / "out" / "paired.csv")}
    assert csv_rows[("ctrl", "acd3")]["coll_robot_diff_pp"] == ""
    md = (tmp_path / "out" / "tables.md").read_text(encoding="utf-8")
    assert "로봇 충돌 차 %p" in md


def test_gap_table_with_empty_bins_and_no_aisle_box(tmp_path):
    code = [0, 0, 1, 3, 0, 0]
    keys = list(KEYS)
    gaps = np.array(GAPS)
    extra = {"by_gap": nt.bg.by_gap(gaps, code, keys), "in_aisle_pass": nt.bg.in_aisle_pass(gaps, code)}
    fake_eval(tmp_path, "e_demo_aisle", code, extra=extra)
    none = np.full(6, np.nan)
    fake_eval(tmp_path, "e_demo_rand", code, extra={
        "by_gap": nt.bg.by_gap(none, code, keys), "in_aisle_pass": nt.bg.in_aisle_pass(none, code)})
    status(tmp_path, ["e_demo_aisle exit=0", "e_demo_rand exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    a, r = t["gap"]
    assert (a["cell"], a["aisle_episodes"], a["in_aisle_pass_pct"]) == ("aisle", 4, 50.0)
    assert a["in_aisle_pass_se_pp"] == pytest.approx(25.0)
    assert (a["gap_<0.13_n"], a["gap_<0.13_success_pct"]) == (0, "")
    assert (a["gap_0.13-0.16_n"], a["gap_0.13-0.16_success_pct"]) == (2, 50.0)
    assert (a["gap_0.19-0.22_n"], a["gap_0.19-0.22_success_pct"]) == (1, 0.0)
    assert (a["gap_>=0.25_n"], a["gap_>=0.25_success_pct"]) == (1, 100.0)
    assert "gap_no aisle box_n" not in a and "gap_noaislebox_n" not in a
    assert (r["aisle_episodes"], r["in_aisle_pass_pct"], r["in_aisle_pass_se_pp"]) == (0, "", "")
    md = (tmp_path / "out" / "tables.md").read_text(encoding="utf-8")
    assert "통로 안 통과 %" in md and "틈 0.13-0.16 m 성공 %" in md


def test_gap_paired_overall_and_by_bin(tmp_path):
    g = {"box_gap_m": np.array(GAPS)}
    fake_eval(tmp_path, "e_demo_aisle", [0, 0, 0, 0, 0, 0], npz=g)
    fake_eval(tmp_path, "e_ctrl_aisle", [0, 0, 1, 3, 0, 0], npz=g)
    status(tmp_path, ["e_demo_aisle exit=0", "e_ctrl_aisle exit=0"])
    rows = nt.main([str(tmp_path), str(tmp_path / "out")])["gap_paired"]
    by = {r["width_bin"]: r for r in rows}
    assert list(by) == ["all"] + nt.GAP_BINS
    a = by["all"]
    assert (a["policy"], a["vs_policy"], a["gap_mask"], a["n"]) == ("demo", "ctrl", "same", 4)
    assert a["success_diff_pp"] == pytest.approx(50.0)
    assert a["success_se_pp"] == pytest.approx(round(100 * np.std([0, 1, 1, 0], ddof=1) / 2, 4))
    assert a["collision_diff_pp"] == pytest.approx(-25.0)
    assert (by["0.13-0.16"]["n"], by["0.13-0.16"]["success_diff_pp"]) == (2, 50.0)
    assert (by["< 0.13"]["n"], by["< 0.13"]["success_diff_pp"], by["< 0.13"]["success_se_pp"]) == (
        0, "", "")
    assert (by["0.19-0.22"]["success_diff_pp"], by["0.19-0.22"]["success_se_pp"]) == (100.0, "")


def test_gap_paired_reports_unequal_masks(tmp_path):
    gb = np.array(GAPS)
    gb[4] = np.nan
    fake_eval(tmp_path, "e_demo_aisle", [0] * 6, npz={"box_gap_m": np.array(GAPS)})
    fake_eval(tmp_path, "e_ctrl_aisle", [0, 0, 1, 3, 0, 0], npz={"box_gap_m": gb})
    fake_eval(tmp_path, "e_acd3_aisle", [0] * 6)  # box_gap_m 없음: 그 쌍은 빠짐
    status(tmp_path, ["e_demo_aisle exit=0", "e_ctrl_aisle exit=0", "e_acd3_aisle exit=0"])
    rows = nt.main([str(tmp_path), str(tmp_path / "out")])["gap_paired"]
    assert {(r["policy"], r["vs_policy"]) for r in rows} == {("demo", "ctrl")}
    a = rows[0]
    assert a["gap_mask"] == "differs (a only 1, b only 0)"
    assert a["n"] == 3 and a["success_diff_pp"] == pytest.approx(round(200 / 3, 4))


def test_crossing_table(tmp_path):
    bc = {"cross_on_path": {"episodes": 10, "success": 0.5, "collision": 0.4, "oob": 0.0,
                            "timeout": 0.1},
          "cross_off_path": {"episodes": 20, "success": 0.9, "collision": 0.1, "oob": 0.0,
                             "timeout": 0.0},
          "no_cross": {"episodes": 0, "success": 0.0, "collision": 0.0, "oob": 0.0,
                       "timeout": 0.0}}
    fake_eval(tmp_path, "e_demo_demo1", [0, 1], extra={"by_crossing": bc})
    status(tmp_path, ["e_demo_demo1 exit=0"])
    (r,) = nt.main([str(tmp_path), str(tmp_path / "out")])["crossing"]
    assert (r["cross_on_path_n"], r["cross_on_path_collision_pct"]) == (10, 40.0)
    assert r["cross_off_path_success_pct"] == 90.0
    assert (r["no_cross_n"], r["no_cross_success_pct"]) == (0, "")
    assert r["coll_on_minus_off_pp"] == pytest.approx(30.0)
    assert r["coll_on_minus_off_se_pp"] == pytest.approx(
        round(100 * np.sqrt(0.4 * 0.6 / 10 + 0.1 * 0.9 / 20), 4))
    assert r["coll_on_minus_off_noise"] == "inside"


def test_hit_kind_without_kind_names_leaves_columns_empty(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [1, 0], npz={"hit_kind": np.array([3, 0])})
    fake_eval(tmp_path, "e_std_rand", [0, 0],
              npz={"hit_kind": np.array([0, 0]), "hit_kinds": KINDS})
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0"])
    (row,) = nt.main([str(tmp_path), str(tmp_path / "out")])["paired"]
    assert set(row) == OLD_PAIR_KEYS


def test_new_fields_follow_env_order(tmp_path):
    # ctrl은 env 순서를 뒤집어 저장: 짝은 env 번호로 맞춰져야 한다
    fake_eval(tmp_path, "e_demo_aisle", [0, 0, 0, 0],
              npz={"box_gap_m": np.array([np.nan, 0.14, 0.14, 0.30]),
                   "hit_kind": np.zeros(4, int), "hit_kinds": KINDS})
    d = tmp_path / "e_ctrl_aisle"
    fake_eval(tmp_path, "e_ctrl_aisle", [0, 0, 0, 0])
    np.savez(d / "episodes.npz", env=np.array([3, 2, 1, 0]), code=np.array([1, 0, 0, 0]),
             keys=KEYS, flips=np.zeros(4), length_s=np.full(4, 10.0),
             box_gap_m=np.array([0.30, 0.14, 0.14, np.nan]),
             hit_kind=np.array([3, 0, 0, 0]), hit_kinds=KINDS)
    status(tmp_path, ["e_demo_aisle exit=0", "e_ctrl_aisle exit=0"])
    t = nt.main([str(tmp_path), str(tmp_path / "out")])
    (row,) = t["paired"]
    assert row["coll_robot_diff_pp"] == pytest.approx(-25.0)
    by = {r["width_bin"]: r for r in t["gap_paired"]}
    assert by["all"]["gap_mask"] == "same"
    assert (by[">= 0.25"]["n"], by[">= 0.25"]["collision_diff_pp"]) == (1, -100.0)
    assert (by["0.13-0.16"]["n"], by["0.13-0.16"]["collision_diff_pp"]) == (2, 0.0)


def add_layout(runs, tag, obstacle_xy):
    """rollouts.npz에 play.py가 쓰는 경로 표의 상자 자리를 더한다."""
    p = runs / tag / "rollouts.npz"
    z = dict(np.load(p))
    np.savez(p, **z, obstacle_xy=np.array(obstacle_xy, float), obstacle_sizes=np.full(3, 0.1))


def test_different_box_layouts_are_reported_not_paired(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [1, 1, 0, 3])
    fake_eval(tmp_path, "e_std_rand", [0, 0, 0, 0])
    a = np.full((5, 3, 2), np.nan)
    a[0, 0] = (0.8, 0.6)
    b = a.copy()
    b[0, 0] = (0.9, 0.6)  # 시작 상태는 같고 상자 자리만 다르다
    add_layout(tmp_path, "e_ctrl_rand", a)
    add_layout(tmp_path, "e_std_rand", b)
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0"])
    (row,) = nt.main([str(tmp_path), str(tmp_path / "out")])["paired"]
    assert row["same_episodes"] == "different route table"
    assert "success_diff_pp" not in row


def test_equal_box_layouts_pair_and_one_sided_layout_is_not_checked(tmp_path):
    fake_eval(tmp_path, "e_ctrl_rand", [1, 1, 0, 3])
    fake_eval(tmp_path, "e_std_rand", [0, 0, 0, 0])
    fake_eval(tmp_path, "e_acd3_rand", [0, 0, 0, 0])  # 경로 표가 없는 예전 실행
    a = np.full((5, 3, 2), np.nan)  # 빈 칸은 NaN이고 NaN끼리는 같다고 본다
    a[0, 0] = (0.8, 0.6)
    add_layout(tmp_path, "e_ctrl_rand", a)
    add_layout(tmp_path, "e_std_rand", a.copy())
    status(tmp_path, ["e_ctrl_rand exit=0", "e_std_rand exit=0", "e_acd3_rand exit=0"])
    rows = nt.main([str(tmp_path), str(tmp_path / "out")])["paired"]
    assert {r["same_episodes"] for r in rows} == {"yes (4 tracked envs)"}
    assert len(rows) == 3


def test_cross_person_counts_person_hits_by_crossing_group_and_pairs_them(tmp_path):
    import cross_person as cp
    keys = np.array(["success", "collision", "oob", "timeout"])
    kinds = np.array(["wall_shelf", "box", "mover", "robot"])
    cross_path = np.array([1, 1, 1, 1, 0, 0, 0, 0], bool)
    cross = np.array([1, 1, 1, 1, 1, 1, 0, 0], bool)

    def save(name, code, hit):
        d = tmp_path / name
        d.mkdir()
        np.savez(d / "episodes.npz", keys=keys, hit_kinds=kinds, code=np.array(code), hit_kind=np.array(hit),
                 cross=cross, cross_path=cross_path, env=np.arange(8))
        return d

    # a: 경로 위 넷 가운데 사람 충돌 1 (성공으로 끝난 판의 hit_kind=mover는 세지 않는다)
    a = cp.load(save("a", [1, 0, 1, 0, 0, 1, 0, 0], [2, 2, 1, 0, 0, 3, 0, 0]))
    b = cp.load(save("b", [1, 1, 1, 0, 0, 0, 0, 0], [2, 2, 2, 0, 0, 0, 0, 0]))
    ra, rb = cp.rates(a), cp.rates(b)
    assert ra["cross_on_path"]["n"] == 4 and ra["cross_on_path"]["person_pct"] == 25.0
    assert rb["cross_on_path"]["person_pct"] == 75.0
    assert ra["cross_off_path"]["n"] == 2 and ra["no_cross"]["n"] == 2
    assert ra["cross_off_path"]["person_pct"] == 0.0
    d = cp.paired(a, b)["cross_on_path"]
    assert d["n"] == 4 and d["diff_pp"] == -50.0
    assert abs(d["se_pp"] - 100 * np.std([0, -1, -1, 0], ddof=1) / 2) < 1e-9
    # 건넘 무리가 다르면 짝을 짓지 않는다
    c = dict(b, cross_path=~cross_path)
    with pytest.raises(ValueError, match="not the same episodes"):
        cp.paired(a, c)


def test_cross_person_pairs_by_env_not_by_record_order_and_has_an_unpaired_fallback(tmp_path):
    import cross_person as cp
    keys = np.array(["success", "collision", "oob", "timeout"])
    kinds = np.array(["wall_shelf", "box", "mover", "robot"])

    def save(name, env, code, hit, on):
        d = tmp_path / name
        d.mkdir()
        on = np.array(on, bool)
        np.savez(d / "episodes.npz", keys=keys, hit_kinds=kinds, code=np.array(code), hit_kind=np.array(hit),
                 cross=on.copy(), cross_path=on, env=np.array(env))
        return d

    # 같은 env 넷, 끝난 순서만 다르다 (평가는 끝난 순서대로 적는다)
    a = cp.load(save("a", [0, 1, 2, 3], [1, 0, 1, 0], [2, 0, 2, 0], [1, 1, 1, 0]))
    b = cp.load(save("b", [3, 2, 1, 0], [0, 0, 1, 1], [0, 0, 2, 2], [0, 1, 1, 1]))
    assert a["env"].tolist() == [0, 1, 2, 3] and b["env"].tolist() == [0, 1, 2, 3]
    d = cp.paired(a, b)["cross_on_path"]
    # env 0: 1-1, env 1: 0-1, env 2: 1-0 → 평균 0
    assert d["n"] == 3 and d["diff_pp"] == 0.0
    # 짝을 지을 수 없을 때 쓰는 독립 표본 차이
    u = cp.unpaired(a, b)["cross_on_path"]
    pa, pb = 2 / 3, 2 / 3
    assert u["n_a"] == 3 and u["n_b"] == 3 and abs(u["diff_pp"]) < 1e-9
    assert abs(u["se_pp"] - 100 * (pa * (1 - pa) / 3 + pb * (1 - pb) / 3) ** 0.5) < 1e-9
    # env 묶음이 다르면 짝을 짓지 않는다
    c = cp.load(save("c", [0, 1, 2, 4], [1, 0, 1, 0], [2, 0, 2, 0], [1, 1, 1, 0]))
    with pytest.raises(ValueError, match="not the same episodes"):
        cp.paired(a, c)


def test_cross_person_compare_falls_back_and_labels_only_what_it_can_judge(tmp_path, monkeypatch):
    import cross_person as cp
    keys = np.array(["success", "collision", "oob", "timeout"])
    kinds = np.array(["wall_shelf", "box", "mover", "robot"])

    def save(name, env, code, hit, on):
        d = tmp_path / name
        d.mkdir()
        on = np.array(on, bool)
        np.savez(d / "episodes.npz", keys=keys, hit_kinds=kinds, code=np.array(code), hit_kind=np.array(hit),
                 cross=on.copy(), cross_path=on, env=np.array(env))
        return str(d)

    a = save("a", [0, 1, 2, 3], [1, 0, 1, 0], [2, 0, 2, 0], [1, 1, 1, 1])
    b = save("b", [0, 1, 2, 3], [0, 0, 1, 1], [0, 0, 2, 2], [1, 1, 1, 1])
    other = save("other", [0, 1, 2, 3], [0, 0, 1, 1], [0, 0, 2, 2], [1, 1, 0, 0])
    twice = save("twice", [0, 0, 1, 1], [1, 0, 1, 0], [2, 0, 2, 0], [1, 1, 1, 1])
    # 시작 상태를 확인할 수 없으면(rollouts.npz 없음) 독립 표본 차이
    kind, why, rows = cp.compare(a, b)
    assert kind == "unpaired" and "start not verified" in why
    # 빈 무리는 2 SE 표시를 하지 않는다
    assert rows["no_cross"]["n_a"] == 0 and rows["no_cross"]["two_se"] == ""
    assert rows["cross_on_path"]["two_se"] in ("inside 2 SE", "outside 2 SE")
    # 시작 상태가 같다고 나와도 건넘 무리가 다르면 독립 표본 차이로 간다
    monkeypatch.setattr(cp.nt, "check_same", lambda da, db: ("", 16))
    kind, why, rows = cp.compare(a, other)
    assert kind == "unpaired" and "not the same episodes" in why
    kind, why, rows = cp.compare(a, b)
    assert kind == "paired" and "16 tracked envs" in why and rows["cross_on_path"]["n"] == 4
    assert rows["cross_on_path"]["two_se"] == "inside 2 SE" and rows["no_cross"]["two_se"] == ""
    # env마다 에피소드가 여럿이면 뒤 에피소드가 같은지 알 수 없어 짝을 짓지 않는다
    kind, why, rows = cp.compare(twice, twice)
    assert kind == "unpaired" and "more than one episode per env" in why

"""real_scan_mask_check가 플러그인의 규칙(가림, 레이 고르기, 섹터 값)대로 셈하는지 본다."""
import importlib.util
import json
import math
import pathlib
import sys

import numpy as np
import pytest
import real_scan_mask_check as m

LID = {"rays": 360, "first_ray_deg": -180.0, "ray_step_deg": 1.0, "sectors": 36, "obs_max": 2.0}
NARROW = m.parse_mask(m.NARROW_MASK)


def grid(offset_deg=0.2, n=360, step_deg=1.0):
    """빔이 레이보다 offset만큼 돌아간 한 바퀴 격자(angle_min, inc)를 낸다."""
    return n, math.radians(-180.0 + offset_deg), math.radians(step_deg)


def scan_npz(tmp_path, ranges, n, amin, inc, yaw=0.0, rmin=0.05, rmax=12.0):
    """같은 격자의 scan 여러 장을 check가 읽는 npz로 쓴다."""
    ranges = np.asarray(ranges, dtype=np.float32)
    count = len(ranges)
    path = tmp_path / "scans.npz"
    np.savez(
        path, ranges=ranges, n=np.full(count, n),
        meta=np.tile([amin, amin + (n - 1) * inc, inc, rmin, rmax], (count, 1)),
        stamp=np.arange(count), yaw=np.array(yaw), frames=np.array("{}"))
    return path


# ---- 가림과 레이 고르기 ----
def test_parse_mask_refuses_bad_values():
    assert m.parse_mask("140,147,-150.5,-147.5") == [(140.0, 147.0), (-150.5, -147.5)]
    for bad in ("140", "147,140", "-181,0", "0,181", "1,1"):
        with pytest.raises(ValueError):
            m.parse_mask(bad)


def test_ray_picks_the_nearest_beam_and_wraps():
    n, amin, inc = grid()
    idx = m.ray_index(n, amin, inc, 0.0, LID)
    # 레이 d도는 d+0.2도의 빔을 고르고, -180도 레이는 한 바퀴를 돌아 0번 빔을 고른다
    assert list(idx[:3]) == [0, 1, 2] and idx[359] == 359


def test_ray_outside_a_partial_scan_is_not_covered():
    # 앞쪽 반 바퀴만 담은 scan: 뒤쪽 레이는 고를 빔이 없다
    idx = m.ray_index(181, math.radians(-90.0), math.radians(1.0), 0.0, LID)
    assert idx[180] == 90 and idx[0] == -1 and idx[359] == -1


def test_narrow_mask_leaves_rays_in_every_post_sector():
    n, amin, inc = grid()
    hit = m.masked_beams(m.beam_deg(n, amin, inc, 0.0), NARROW)
    idx = m.ray_index(n, amin, inc, 0.0, LID)
    left = (~hit[idx]).reshape(36, 10).sum(axis=1)
    # 빔이 0.2도 돌아간 격자에서 손으로 센 값: 가린 레이 21개, 기둥 섹터에 3, 9, 2, 5개가 남는다
    assert int(hit[idx].sum()) == 21
    assert [int(left[s]) for s in m.POST_SECTORS] == [3, 9, 2, 5]


def test_lidar_yaw_moves_the_mask_with_the_base_frame():
    # lidar가 뒤를 보고 달려 있으면(yaw 180도) 빔 번호는 달라져도 base 기준으로 가려지는 레이는 같다
    n, amin, inc = grid()
    idx = m.ray_index(n, amin, inc, math.pi, LID)
    hit = m.masked_beams(m.beam_deg(n, amin, inc, math.pi), NARROW)
    assert int(hit[idx].sum()) == 21 and idx[180] != 180


def test_straight_back_counts_as_both_ends():
    deg = np.array([-180.0, 180.0, 179.0])
    assert list(m.masked_beams(deg, [(179.5, 180.0)])) == [True, True, False]
    assert list(m.masked_beams(deg, [(-180.0, -179.5)])) == [True, True, False]


# ---- 섹터 값 ----
def test_masked_post_reads_as_the_ray_that_is_left():
    n, amin, inc = grid()
    ranges = np.full(n, 1.0, dtype=np.float32)
    deg = m.beam_deg(n, amin, inc, 0.0)
    ranges[(deg > 140.0) & (deg < 147.0)] = 0.06
    out = m.sector_values(ranges, n, amin, inc, 0.05, 12.0, 0.0, LID, NARROW)
    assert out["raw"][32] == pytest.approx(0.06) and out["masked"][32] == pytest.approx(1.0)


def test_whole_sector_mask_reads_as_no_return():
    n, amin, inc = grid()
    ranges = np.full(n, 1.0, dtype=np.float32)
    out = m.sector_values(ranges, n, amin, inc, 0.05, 12.0, 0.0, LID, [(138.0, 157.0)])
    assert out["left"][32] == 0 and out["masked"][32] == pytest.approx(2.0)
    assert out["masked"][31] == pytest.approx(1.0)


def test_returns_outside_the_range_limits_read_as_no_return():
    n, amin, inc = grid()
    ranges = np.full(n, 1.0, dtype=np.float32)
    deg = m.beam_deg(n, amin, inc, 0.0)
    ranges[(deg > 0.0) & (deg < 10.0)] = 0.03
    ranges[(deg > 10.0) & (deg < 20.0)] = np.inf
    out = m.sector_values(ranges, n, amin, inc, 0.05, 12.0, 0.0, LID, [])
    assert out["raw"][18] == pytest.approx(2.0) and out["raw"][19] == pytest.approx(2.0)
    assert bool(out["all_bad"][18]) and bool(out["all_bad"][19]) and not bool(out["all_bad"][20])


# ---- 지표와 판정 ----
def test_grade_uses_the_planned_thresholds():
    words = ("a", "b", "c")
    got = [m.grade(x, words) for x in (0.0, 0.01, 0.0101, 0.10, 0.1001)]
    assert got == ["a", "a", "b", "b", "c"]


def test_check_reports_a_leak_and_an_empty_sector(tmp_path):
    n, amin, inc = grid()
    deg = m.beam_deg(n, amin, inc, 0.0)
    clean = np.full(n, 1.0, dtype=np.float32)
    clean[(deg > 140.0) & (deg < 147.0)] = 0.06
    leak = clean.copy()
    leak[(deg > 147.0) & (deg < 149.0)] = 0.06
    empty = clean.copy()
    empty[(deg > 147.0) & (deg < 149.0)] = np.inf
    path = scan_npz(tmp_path, [clean, clean, leak, empty], n, amin, inc)
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    out = tmp_path / "out"
    res = m.check(path, spec, out, m.NARROW_MASK)
    assert res["scans"] == 4 and res["m1"]["verdict"].startswith("실물 scan 격자에서")
    assert res["m1"]["min_left"] == {"3": 3, "4": 9, "32": 2, "33": 5}
    assert res["m2"]["masked"] == pytest.approx(0.25) and res["m2"]["raw"] == pytest.approx(1.0)
    assert res["m3"]["masked"] == pytest.approx(0.25) and res["m3"]["raw"] == pytest.approx(0.0)
    assert res["m3"]["per_sector_masked"]["32"] == pytest.approx(0.25)
    assert res["m3"]["no_usable"] == pytest.approx(0.25)
    assert res["m3"]["per_sector_no_usable"] == {"3": 0.0, "4": 0.0, "32": 0.25, "33": 0.0}
    assert res["near"]["outside_mask"] == 2
    ray = {r["ray_deg"]: r for r in res["rays"]}
    assert ray[148.0]["masked"] is False and ray[148.0]["valid"] == pytest.approx(0.75)
    assert ray[148.0]["near"] == pytest.approx(1 / 3) and ray[149.0]["masked"] is True
    assert ray[148.0]["beam_deg"] == pytest.approx(148.2) and 0.0 not in ray
    saved = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert saved["m2"]["verdict"] == "자주 샌다" and (out / "summary.md").exists()


def test_check_splits_moving_and_stopped_scans(tmp_path):
    n, amin, inc = grid()
    deg = m.beam_deg(n, amin, inc, 0.0)
    full = np.full(n, 1.0, dtype=np.float32)
    empty = full.copy()
    empty[(deg > 147.0) & (deg < 149.0)] = np.inf
    path = scan_npz(tmp_path, [full, empty, empty, full], n, amin, inc)
    with np.load(path) as z:
        data = dict(z)
    # scan 시각 0, 1, 2, 3에 odom은 0.4, 1.4, 2.7에 있다: 가장 가까운 것이 0, 1, 1, 2번째다
    data.update(odom_t=np.array([0.4, 1.4, 2.7]), odom_v=np.array([0.0, 0.05, 0.0]),
                odom_w=np.array([0.0, 0.0, 0.2]))
    np.savez(path, **data)
    with np.load(path) as z:
        assert list(m.moving_scans(z)) == [False, True, True, True]
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    res = m.check(path, spec, tmp_path / "out", m.NARROW_MASK)
    assert res["motion"]["moving"]["scans"] == 3 and res["motion"]["stopped"]["scans"] == 1
    assert res["motion"]["moving"]["m3_masked"] == pytest.approx(2 / 3)
    assert res["motion"]["stopped"]["m3_masked"] == pytest.approx(0.0)
    assert "움직이는 동안" in (tmp_path / "out" / "summary.md").read_text(encoding="utf-8")


def test_far_valid_rays_read_as_the_limit_but_are_not_counted_as_no_return(tmp_path):
    # 남긴 레이가 2 m보다 먼 반환을 읽으면 섹터 값은 상한이지만 "반환이 없던 장"은 아니다
    n, amin, inc = grid()
    deg = m.beam_deg(n, amin, inc, 0.0)
    far = np.full(n, 1.0, dtype=np.float32)
    far[(deg > 147.0) & (deg < 149.0)] = 5.0
    path = scan_npz(tmp_path, [far], n, amin, inc)
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    res = m.check(path, spec, tmp_path / "out", m.NARROW_MASK)
    assert res["m3"]["masked"] == pytest.approx(1.0)
    assert res["m3"]["no_usable"] == pytest.approx(0.0)


def test_motion_split_with_one_odom_sample_and_an_empty_side(tmp_path):
    n, amin, inc = grid()
    path = scan_npz(tmp_path, [np.full(n, 1.0, dtype=np.float32)] * 3, n, amin, inc)
    with np.load(path) as z:
        data = dict(z)
    data.update(odom_t=np.array([1.2]), odom_v=np.array([0.0]), odom_w=np.array([0.0]))
    np.savez(path, **data)
    with np.load(path) as z:
        assert list(m.moving_scans(z)) == [False, False, False]
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    res = m.check(path, spec, tmp_path / "out", m.NARROW_MASK)
    assert res["motion"]["moving"] == {
        "scans": 0, "m2_masked": None, "m3_masked": None,
        "m3_per_sector_masked": {"3": None, "4": None, "32": None, "33": None}, "m2_raw": None}
    assert res["motion"]["stopped"]["scans"] == 3 and res["motion"]["stopped"]["m3_masked"] == 0.0
    assert "없음" in (tmp_path / "out" / "summary.md").read_text(encoding="utf-8")


def test_check_without_odom_has_no_motion_split(tmp_path):
    n, amin, inc = grid()
    path = scan_npz(tmp_path, [np.full(n, 1.0, dtype=np.float32)], n, amin, inc)
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    res = m.check(path, spec, tmp_path / "out", m.NARROW_MASK)
    with np.load(path) as z:
        assert res["motion"] == {} and m.moving_scans(z) is None


def test_check_flags_a_sector_with_no_ray_left(tmp_path):
    n, amin, inc = grid()
    path = scan_npz(tmp_path, [np.full(n, 1.0, dtype=np.float32)], n, amin, inc)
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    res = m.check(path, spec, tmp_path / "out", "138,157,-152.5,-137")
    assert res["m1"]["verdict"].startswith("남지 않는 섹터가 있다")
    assert res["m1"]["min_left"]["32"] == 0


# ---- 채우기 ----
WIDE = "138,157,-152.5,-137"


def test_fill_sectors_takes_the_nearer_side_and_skips_empty_neighbours():
    values = np.full(36, 0.5)
    values[[31, 34]] = [0.4, 0.7]
    values[[32, 33]] = 2.0
    touched = np.zeros(36, dtype=bool)
    touched[[31, 32, 33]] = True
    usable = np.ones(36, dtype=bool)
    usable[[32, 33]] = False
    out, empty = m.fill_sectors(values, usable, touched, 2.0)
    assert list(np.flatnonzero(empty)) == [32, 33]
    assert out[32] == 0.4 and out[33] == 0.4 and out[31] == 0.4 and out[34] == 0.7
    assert values[32] == 2.0  # 받은 배열은 바꾸지 않는다
    # 가림에 걸리지 않은 섹터는 쓸 수 있는 레이가 없어도 그대로다
    usable[18] = False
    values[18] = 2.0
    out, empty = m.fill_sectors(values, usable, touched, 2.0)
    assert out[18] == 2.0 and not empty[18]


def test_fill_sectors_gives_the_limit_when_no_neighbour_is_within_reach():
    values = np.full(36, 1.7)
    values[35], values[10] = 0.2, 0.9
    touched = np.zeros(36, dtype=bool)
    touched[:10] = True
    out, _ = m.fill_sectors(values, ~touched, touched, 2.0)
    assert list(out[:10]) == [0.2] * 4 + [2.0] * 2 + [0.9] * 4


def test_masked_beams_takes_a_beam_on_the_edge():
    deg = np.array([-150.0, -149.0, -141.0, -140.0])
    assert list(m.masked_beams(deg, [(-150.0, -141.0)])) == [True, True, True, False]
    assert list(m.masked_beams(deg, [(-149.9999995, -141.0000005)])) == [True, True, True, False]
    assert list(m.masked_beams(deg, [(-149.99999, -141.00001)])) == [False, True, False, False]


def test_masked_beams_matches_the_evaluation_code_on_whole_degree_edges():
    pytest.importorskip("torch")
    path = pathlib.Path(__file__).resolve().parents[2] / "learning/goal_reach/task_spec.py"
    spec = importlib.util.spec_from_file_location("task_spec_for_edges", path)
    ts = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = ts
    spec.loader.exec_module(ts)
    deg = m.beam_deg(360, -np.pi, 2.0 * np.pi / 360, 0.0)
    for lo, hi in [(138.0, 157.0), (-152.5, -137.0), (170.0, 180.0), (-180.0, -171.0),
                   (-150.0, -141.0), (0.0, 1.0)]:
        ours = m.masked_beams(deg, [(lo, hi)])
        theirs = ts.rays_within(360, lo, hi).numpy()
        assert np.array_equal(ours, theirs), (lo, hi)


def test_fill_sectors_matches_the_evaluation_code():
    torch = pytest.importorskip("torch")
    path = pathlib.Path(__file__).resolve().parents[2] / "learning/goal_reach/task_spec.py"
    spec = importlib.util.spec_from_file_location("task_spec_for_fill", path)
    ts = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = ts
    spec.loader.exec_module(ts)
    assert m.FILL_REACH == ts.FILL_REACH
    rng = np.random.default_rng(34)
    for _ in range(300):
        values = rng.uniform(0.05, 2.0, 36).astype(np.float32)
        touched = rng.random(36) < rng.uniform(0.05, 0.9)
        usable = rng.random(36) < rng.uniform(0.1, 0.9)
        ours, _ = m.fill_sectors(values, usable, touched, 2.0)
        theirs = ts.fill_masked_sectors(
            torch.from_numpy(values)[None], torch.from_numpy(usable)[None],
            torch.from_numpy(touched), 2.0)[0].numpy()
        assert np.array_equal(ours.astype(np.float32), theirs)


def test_check_with_fill_reads_a_fully_masked_sector_from_its_neighbours(tmp_path):
    n, amin, inc = grid()
    deg = m.beam_deg(n, amin, inc, 0.0)
    scan = np.full(n, 1.0, dtype=np.float32)
    scan[(deg > 140.0) & (deg < 157.0)] = 0.06  # 기둥: 넓은 가림 안
    scan[(deg > 130.0) & (deg < 138.0)] = 0.4  # 섹터 31 에 남는 레이
    scan[(deg > 157.0) & (deg < 160.0)] = 0.7  # 섹터 33 에 남는 레이
    path = scan_npz(tmp_path, [scan], n, amin, inc)
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    plain = m.check(path, spec, tmp_path / "plain", WIDE)
    assert plain["m3"]["per_sector_masked"]["32"] == pytest.approx(1.0)
    assert plain["m1"]["verdict"].startswith("남지 않는 섹터가 있다") and plain["fill"] == {}
    res = m.check(path, spec, tmp_path / "fill", WIDE, fill=True)
    assert res["m3"]["masked"] == pytest.approx(0.0) and res["m2"]["masked"] == pytest.approx(0.0)
    assert res["fill"]["per_sector_filled"] == {"3": 1.0, "4": 0.0, "32": 1.0, "33": 0.0}
    assert res["fill"]["filled_any"] == pytest.approx(1.0)
    assert res["post_value_masked_m"]["32"] == [0.4, 0.4, 0.4]
    assert res["post_value_masked_m"]["3"] == [1.0, 1.0, 1.0]
    assert res["fill"]["touched"] == [2, 3, 4, 31, 32, 33]
    assert res["m1"]["verdict"].startswith("채우기를 켰다")
    assert "채우기" in (tmp_path / "fill" / "summary.md").read_text(encoding="utf-8")


def test_check_with_fill_says_that_a_sector_with_no_neighbour_stays_at_the_limit(tmp_path):
    # 모두 가리면 채울 이웃이 없어 상한으로 남는다: M1 문장이 그 경우를 말하고 M3이 잡는다
    n, amin, inc = grid()
    path = scan_npz(tmp_path, [np.full(n, 1.0, dtype=np.float32)], n, amin, inc)
    spec = tmp_path / "policy_spec.json"
    spec.write_text(json.dumps({"lidar": LID}), encoding="utf-8")
    res = m.check(path, spec, tmp_path / "all", "-180,180", fill=True)
    assert res["m3"]["masked"] == pytest.approx(1.0)
    assert res["m3"]["verdict"] == m.grade(1.0, m.EMPTY_WORDS)
    assert res["fill"]["no_neighbour_any"] == pytest.approx(1.0)
    assert "상한" in res["m1"]["verdict"] and "100.00 %" in res["m1"]["verdict"]
    # 기둥 섹터가 아닌 곳에서 이웃이 없으면 M3에는 안 보이지만 따로 센 값에는 보인다
    far = m.check(path, spec, tmp_path / "far", "-80.5,29.5", fill=True)
    assert far["m3"]["masked"] == pytest.approx(0.0)
    assert far["fill"]["no_neighbour_any"] == pytest.approx(1.0)
    assert far["fill"]["no_neighbour_sectors"] == [14, 15, 16]
    wide = m.check(path, spec, tmp_path / "wide", WIDE, fill=True)
    assert wide["fill"]["no_neighbour_any"] == pytest.approx(0.0)
    assert wide["fill"]["no_neighbour_sectors"] == []


def test_no_neighbour_marks_only_filled_sectors_with_nothing_in_reach():
    empty = np.zeros(36, dtype=bool)
    empty[10:21] = True
    assert list(np.flatnonzero(m.no_neighbour(empty))) == [14, 15, 16]
    empty[:] = False
    empty[[0, 1, 2, 3, 35, 34, 33, 32, 31]] = True  # 한 바퀴를 넘어 이어진다
    assert list(np.flatnonzero(m.no_neighbour(empty))) == [35]


def test_module_file_has_a_usage_docstring():
    text = pathlib.Path(m.__file__).read_text(encoding="utf-8")
    assert "dump <bag.mcap>" in text and "check <scans.npz>" in text

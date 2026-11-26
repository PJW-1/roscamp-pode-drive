"""real_scan_mask_check가 플러그인의 규칙(가림, 레이 고르기, 섹터 값)대로 셈하는지 본다."""
import json
import math
import pathlib

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


def test_module_file_has_a_usage_docstring():
    text = pathlib.Path(m.__file__).read_text(encoding="utf-8")
    assert "dump <bag.mcap>" in text and "check <scans.npz>" in text

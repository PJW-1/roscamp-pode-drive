"""청소 커버리지 측정 도구(tools/coverage_report.py): 합성 bag으로 지표와 빈틈을 손 계산과 맞춘다."""
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

np = pytest.importorskip("numpy")
rosbag2_py = pytest.importorskip("rosbag2_py")
import coverage_report as cr  # noqa: E402

NS = "pode9"
DT = 0.1
T0 = 100.0
# 계획: 셀 하나에 x 0.2~1.0의 줄 셋(y 0.2 동, 0.3 서, 0.4 동), 줄 간격 0.10
PLAN = """params: {line_spacing_m: 0.1, turn_radius_m: 0.1}
lines:
- {cell: 0, start: [0.2, 0.2], end: [1.0, 0.2]}
- {cell: 0, start: [1.0, 0.3], end: [0.2, 0.3]}
- {cell: 0, start: [0.2, 0.4], end: [1.0, 0.4]}
"""


# ---- 합성 궤적 ----
def _rotate(pts, x, y, a0, a1, n=10):
    pts += [(x, y, a0 + (a1 - a0) * k / n) for k in range(1, n + 1)]


def _move(pts, x0, y0, x1, y1, yaw, n=10):
    pts += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, yaw) for k in range(1, n + 1)]


def _xs(a, b):
    """x = a, a±0.01, ..., b (0.005 끝자리)."""
    n = round(abs(b - a) / 0.01)
    step = 0.01 if b > a else -0.01
    return [round(a + step * k, 3) for k in range(n + 1)]


def actual_track():
    """줄 0은 y 0.19, 줄 1은 0.315(서로 멀어져 간격 0.125), 줄 2는 0.41/0.43 번갈아.
    줄마다 계획 끝에서 0.055 m 넘쳐 가고, 줄 끝에서 제자리 회전 - 직진 - 제자리 회전."""
    pts = [(x, 0.19, 0.0) for x in _xs(0.145, 1.055)]
    _rotate(pts, 1.055, 0.19, 0.0, math.pi / 2)
    _move(pts, 1.055, 0.19, 1.055, 0.315, math.pi / 2)
    _rotate(pts, 1.055, 0.315, math.pi / 2, math.pi)
    pts += [(x, 0.315, math.pi) for x in _xs(1.045, 0.145)]
    _rotate(pts, 0.145, 0.315, math.pi, math.pi / 2)
    _move(pts, 0.145, 0.315, 0.145, 0.41, math.pi / 2)
    _rotate(pts, 0.145, 0.41, math.pi / 2, 0.0)
    pts += [(x, 0.4 + (0.01 if round((x - 0.005) / 0.01) % 2 == 0 else 0.03), 0.0)
            for x in _xs(0.155, 1.055)]
    return pts


def plan_track(over=0.0):
    """계획 줄 위를 그대로 지나는 궤적(over만큼 넘쳐 감)."""
    lo, hi = 0.2 - over, 1.0 + over
    pts = [(lo + (hi - lo) * k / 80, 0.2, 0.0) for k in range(81)]
    _rotate(pts, hi, 0.2, 0.0, math.pi / 2)
    _move(pts, hi, 0.2, hi, 0.3, math.pi / 2)
    _rotate(pts, hi, 0.3, math.pi / 2, math.pi)
    pts += [(hi - (hi - lo) * k / 80, 0.3, math.pi) for k in range(1, 81)]
    _rotate(pts, lo, 0.3, math.pi, math.pi / 2)
    _move(pts, lo, 0.3, lo, 0.4, math.pi / 2)
    _rotate(pts, lo, 0.4, math.pi / 2, 0.0)
    pts += [(lo + (hi - lo) * k / 80, 0.4, 0.0) for k in range(1, 81)]
    return pts


# ---- 합성 bag ----
def write_bag(path, truth=None, amcl=None):
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = [(f"/{NS}/sim/truth", "nav_msgs/msg/Odometry", truth, Odometry),
              (f"/{NS}/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", amcl,
               PoseWithCovarianceStamped)]
    writer.create_topic(rosbag2_py.TopicMetadata(0, "/clock", "rosgraph_msgs/msg/Clock", "cdr"))
    for n, (name, kind, _, _) in enumerate(topics, 1):
        writer.create_topic(rosbag2_py.TopicMetadata(n, name, kind, "cdr"))
    count = max(len(truth or []), len(amcl or []))
    for i in range(count):
        t = T0 + i * DT
        ns = round(t * 1e9)
        stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
        writer.write("/clock", serialize_message(Clock(clock=stamp)), ns)
        for name, _, pts, cls in topics:
            if pts is None or i >= len(pts):
                continue
            x, y, yaw = pts[i]
            msg = cls()
            msg.header.stamp = stamp
            msg.header.frame_id = "map"
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            q = msg.pose.pose.orientation
            q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
            writer.write(name, serialize_message(msg), ns)
    del writer
    return path


@pytest.fixture
def plan_file(tmp_path):
    p = tmp_path / "plan.yaml"
    p.write_text(PLAN, encoding="utf-8")
    return p


def run(bag, plan_file, ctx=False):
    args = cr.build_parser().parse_args(["metrics", str(bag), "--ns", NS, "--plan", str(plan_file)])
    report, context = cr.analyse(args)
    return (report, context) if ctx else report


# ---- 줄 지표 ----
def test_lateral_spacing_turns_and_gap_match_hand_values(tmp_path, plan_file):
    truth = actual_track()
    bag = write_bag(tmp_path / "bag", truth=truth, amcl=plan_track(over=0.055))
    rep, ctx = run(bag, plan_file, ctx=True)
    m = rep["sources"]["truth"]
    assert m["samples"] == len(truth) == 334
    assert m["lap_time_s"] == pytest.approx(333 * DT)
    # 줄 0: 왼쪽 법선 +y, 0.01 오른쪽 / 줄 1: 서쪽이라 법선 -y, 0.015 / 줄 2: 0.01, 0.03 번갈아
    lines = m["lines"]
    assert [r["samples"] for r in lines] == [60, 60, 60]
    assert lines[0]["mean_m"] == pytest.approx(-0.01)
    assert lines[0]["rms_m"] == pytest.approx(0.01)
    assert lines[1]["mean_m"] == pytest.approx(-0.015)
    assert lines[1]["max_m"] == pytest.approx(0.015)
    assert lines[2]["mean_m"] == pytest.approx(0.02)
    assert lines[2]["rms_m"] == pytest.approx(math.sqrt(0.0005))
    assert lines[2]["p95_m"] == pytest.approx(0.03)
    assert lines[2]["max_m"] == pytest.approx(0.03)
    lat = m["lateral"]
    assert lat["samples"] == 180
    assert lat["mean_m"] == pytest.approx(-0.3 / 180)
    assert lat["rms_m"] == pytest.approx(math.sqrt(0.0495 / 180))
    # 실제 중심선 간격: 0.19 -> 0.315, 0.315 -> 0.42
    assert [s["planned_m"] for s in m["spacing"]] == pytest.approx([0.1, 0.1])
    assert [s["actual_m"] for s in m["spacing"]] == pytest.approx([0.125, 0.105])
    # 줄 끝 회전: 끝 지남(0.005 넘음) - 5칸 - 회전 10 - 직진 10 - 회전 10 = 35칸
    for turn in m["turns"]:
        assert turn["time_s"] == pytest.approx(35 * DT)
        assert turn["max_w_radps"] == pytest.approx((math.pi / 20) / DT)
        assert turn["in_place"] is True
        assert turn["overshoot_m"] == pytest.approx(0.055)
    # 빈틈: 줄 0과 1 사이 y 0.255 한 줄, x 0.175~0.995 = 83칸
    cov = m["coverage"]
    assert cov["gap_count"] == 1
    gap = cov["gaps"][0]
    assert gap["cells"] == 83
    assert gap["area_m2"] == pytest.approx(0.0083)
    assert gap["centroid"] == pytest.approx([0.585, 0.255])
    assert gap["box"] == pytest.approx([0.175, 0.255, 0.995, 0.255])
    # 계획 폭 넓이: 가운데 x 0.2~1.0은 칸 중심 y 0.145~0.455의 32줄 x 80칸,
    # 양 끝은 반원 셋(반지름 0.056, 이웃 겹침 반쪽 뺌) 0.014366 m²씩
    xs, ys = np.meshgrid(ctx["grid"].xc, ctx["grid"].yc)
    middle = ctx["target"] & (xs > 0.2) & (xs < 1.0)
    assert middle.sum() == 80 * 32
    assert (middle.any(axis=1) == ((ys[:, 0] > 0.14) & (ys[:, 0] < 0.46))).all()
    assert cov["target_m2"] == pytest.approx(0.256 + 2 * 0.014366, abs=0.002)
    assert cov["covered_ratio"] == pytest.approx(1 - 0.0083 / cov["target_m2"])
    # 줄 끝 덮지 못한 넓이: 빈틈 줄이 줄 0, 1의 양 끝 0.10 m 칸에 10칸씩
    assert [(r["start_uncovered_m2"], r["end_uncovered_m2"]) for r in lines] == pytest.approx(
        [(0.001, 0.001), (0.001, 0.001), (0.0, 0.0)])


def test_believed_poses_on_the_plan_show_the_hidden_gap(tmp_path, plan_file):
    bag = write_bag(tmp_path / "bag", truth=actual_track(), amcl=plan_track(over=0.055))
    rep = run(bag, plan_file)
    amcl = rep["sources"]["amcl"]
    assert amcl["coverage"]["gap_m2"] == 0.0
    assert amcl["coverage"]["covered_ratio"] == 1.0
    assert amcl["lateral"]["max_m"] == pytest.approx(0.0, abs=1e-12)
    diff = rep["believed_minus_actual"]
    assert diff["believed_not_actual_m2"] == pytest.approx(0.0083)
    assert diff["covered_ratio"] == pytest.approx(0.0083 / amcl["coverage"]["target_m2"])


def test_track_equal_to_plan_leaves_no_gap(tmp_path, plan_file):
    bag = write_bag(tmp_path / "bag", truth=plan_track())
    rep = run(bag, plan_file)
    m = rep["sources"]["truth"]
    assert set(rep["sources"]) == {"truth"}
    assert m["coverage"]["gap_m2"] == 0.0
    assert m["coverage"]["gap_count"] == 0
    assert m["coverage"]["covered_ratio"] == 1.0
    assert all(r["start_uncovered_m2"] == 0.0 and r["end_uncovered_m2"] == 0.0 for r in m["lines"])
    assert [t["overshoot_m"] for t in m["turns"]] == pytest.approx([0.0, 0.0], abs=1e-9)


# ---- 겹침 횟수 ----
def test_sweep_counts_a_return_pass_but_not_a_spin():
    grid = cr.Grid(np.array([0.0, 1.0]), np.array([0.0, 0.0]), 0.1)
    there_back = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [0.0, 0.0]])
    visits = cr.sweep(grid, there_back, 0.05)
    j = int((0.005 - grid.y0) / grid.res)
    i = int((0.505 - grid.x0) / grid.res)
    assert visits[j, i] == 2
    spin = np.array([[0.5, 0.0]] * 5)
    assert cr.sweep(grid, spin, 0.05).max() == 1


# ---- 명령줄 ----
def test_metrics_and_map_commands_write_table_json_and_png(tmp_path, plan_file):
    bag = write_bag(tmp_path / "bag", truth=actual_track(), amcl=plan_track(over=0.055))
    base = [str(bag), "--ns", NS, "--plan", str(plan_file)]
    md = tmp_path / "m.md"
    assert cr.main(["metrics", *base, "--out", str(md)]) == 0
    text = md.read_text(encoding="utf-8")
    assert "| gaps | 1 blobs, 83.0 cm² |" in text
    assert "## believed minus actual" in text
    js = tmp_path / "m.json"
    assert cr.main(["metrics", *base, "--format", "json", "--out", str(js)]) == 0
    import json
    data = json.loads(js.read_text(encoding="utf-8"))
    assert data["sources"]["truth"]["coverage"]["gaps"][0]["cells"] == 83
    png = tmp_path / "map.png"
    assert cr.main(["map", *base, "--out", str(png)]) == 0
    assert png.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert cr.main(["relocalize", *base]) == 2


def test_help_runs_from_checkout():
    result = subprocess.run([sys.executable, str(TOOLS / "coverage_report.py"), "-h"],
                            capture_output=True, text=True, cwd=TOOLS.parent)
    assert result.returncode == 0, result.stderr


# ---- 시뮬 녹화 명령 ----
def test_sim_record_adds_truth_and_clock_to_the_real_topics(tmp_path):
    real = (TOOLS / "real_pode" / "bag_record.sh").read_text(encoding="utf-8")
    real_topics = re.search(r'"\^/\$1/\(([^)]*)\)\$"', real).group(1).split("|")
    fake = tmp_path / "bin"
    fake.mkdir()
    (fake / "ros2").write_text('#!/bin/bash\nprintf "%s\\n" "$@" > "$FAKE_DIR/ros2.args"\n')
    (fake / "ros2").chmod(0o755)
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}", FAKE_DIR=str(tmp_path))
    out = tmp_path / "bag"
    result = subprocess.run(["bash", str(TOOLS / "sim_bag_record.sh"), NS, str(out)],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    args = (tmp_path / "ros2.args").read_text().splitlines()
    assert args[:4] == ["bag", "record", "-o", str(out)]
    assert "--use-sim-time" in args and args[args.index("-s") + 1] == "mcap"
    regex = re.compile(args[args.index("-e") + 1])
    for topic in [*real_topics, "sim/truth"]:
        assert regex.fullmatch(f"/{NS}/{topic}"), topic
    assert regex.fullmatch("/clock")
    assert not regex.fullmatch(f"/{NS}/local_costmap/costmap")
    out.mkdir()
    again = subprocess.run(["bash", str(TOOLS / "sim_bag_record.sh"), NS, str(out)],
                           env=env, capture_output=True, text=True)
    assert again.returncode == 1 and "exists" in again.stdout

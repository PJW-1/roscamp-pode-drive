"""ETA 오차 나누기(tools/eta_cause/eta_split.py): 작은 가짜 기록으로 끝 종류와 원인 몫을 확인한다."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS / "eta_cause"))
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

import eta_split  # noqa: E402
from fleet_core.common.commands import MoveOrderCmd, OrderRecord  # noqa: E402
from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_core.manager import FleetManager  # noqa: E402

ON = replace(load_params(), use_order_model=True)
SPEED = ON.nav_speed_mps
STAND_S = 1.0


# ---- 가짜 기록: 1 s 서 있다가 0.12 m/s로 곧게 가서 끝 노드 0.15 m 앞에서 넘어간다 ----

def record(tmp_path):
    now = [0.0]
    core = FleetManager(load_fleet_map("d2"), ON, ["pode1"], lambda: now[0])
    lines = []
    core.record_to(lines.append)
    start = core.site.node_pose("P-1")
    for _ in range(ON.release_confirm_samples):
        core.on_robot_state("pode1", *start, 0.0, "IDLE")
    core.on_order_board("RUN", [OrderRecord("O-001", {"A": 1})])
    order = None
    while order is None:
        assert now[0] < 5.0, "no order"
        now[0] += 0.1
        order = next((c for c in core.tick() if isinstance(c, MoveOrderCmd)), None)
    end = core.site.node_pose(order.base[0].end_node)
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    span = math.dist(start, end)
    t0 = now[0]
    step = 0
    while True:
        step += 1
        now[0] = t0 + 0.1 * step
        gone = max(0.0, now[0] - t0 - STAND_S) * SPEED
        if span - gone <= ON.nav_pass_m:
            break
        k = gone / span
        core.on_robot_state("pode1", start[0] + k * (end[0] - start[0]),
                            start[1] + k * (end[1] - start[1]), yaw, "MOVING")
        core.on_move_feedback("pode1", order.goal_id, 0, False, "")
        core.tick()
    core.on_move_feedback("pode1", order.goal_id, 1, False, "")
    core.tick()
    path = tmp_path / "fleet_record.jsonl"
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    return path, span


def test_a_pass_segment_splits_into_the_pass_and_the_standing_time(tmp_path):
    path, span = record(tmp_path)
    rows, poses, warnings, speed = eta_split.run([f"r={path}"])
    assert warnings == []
    assert speed == SPEED
    assert rows and {r["kind"] for r in rows} == {"pass"}
    first = eta_split.first_samples(rows)[0]
    assert first["adj"] == pytest.approx(-ON.nav_pass_m / SPEED)
    assert first["actual"] == pytest.approx(STAND_S + (span - ON.nav_pass_m) / SPEED, abs=0.25)
    p = eta_split.parts(first, poses["r"]["pode1"], speed)
    assert p["wait"] == pytest.approx(-STAND_S, abs=eta_split.BIN_S)
    assert p["length"] == pytest.approx(ON.nav_pass_m / SPEED, abs=eta_split.BIN_S)
    assert p["err"] == pytest.approx(-STAND_S, abs=0.3)
    assert p["mps"] == pytest.approx(SPEED, abs=0.01)


def test_the_report_and_a_folder_argument_find_the_same_record(tmp_path):
    path, _ = record(tmp_path)
    by_file, *_ = eta_split.run([f"r={path}"])
    by_folder, *_ = eta_split.run([str(tmp_path)])
    assert len(by_file) == len(by_folder)
    rows, poses, _, speed = eta_split.run([str(tmp_path)])
    text = "\n".join(eta_split.report(rows, poses, speed))
    assert "dispatch kind=pass: n=1" in text
    assert "pose at SEGMENT_DONE kind=pass" in text

"""ETA 오차 나누기(tools/eta_cause/eta_split.py): 작은 가짜 기록으로 끝 종류와 원인 몫을 확인한다."""
import importlib
import math
import shutil
import sys
from dataclasses import replace
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS / "eta_cause"))
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

eta_split = importlib.import_module("eta_split")
_commands = importlib.import_module("fleet_core.common.commands")
MoveOrderCmd, OrderRecord = _commands.MoveOrderCmd, _commands.OrderRecord
load_params = importlib.import_module("fleet_core.common.params").load_params
load_fleet_map = importlib.import_module("fleet_core.fleet_map").load_fleet_map
FleetManager = importlib.import_module("fleet_core.manager").FleetManager

ON = replace(load_params(), use_order_model=True)
SPEED = ON.nav_speed_mps
STAND_S = 1.0


# ---- 가짜 기록: 1 s 서 있다가 0.12 m/s로 곧게 가서 끝 노드 0.15 m 앞에서 넘어간다 ----

def record(tmp_path, name="fleet_record.jsonl"):
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
    path = tmp_path / name
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    return path, span


def test_a_pass_segment_splits_into_the_pass_and_the_standing_time(tmp_path):
    path, span = record(tmp_path)
    rows, poses, warnings, speeds = eta_split.run([f"r={path}"])
    assert warnings == []
    assert speeds == {SPEED}
    assert rows and {r["kind"] for r in rows} == {"pass"}
    first = eta_split.first_samples(rows)[0]
    assert first["adj"] == pytest.approx(-ON.nav_pass_m / SPEED)
    assert first["actual"] == pytest.approx(STAND_S + (span - ON.nav_pass_m) / SPEED, abs=0.25)
    p = eta_split.parts(first, poses["r"]["pode1"])
    assert p["wait"] == pytest.approx(-STAND_S, abs=eta_split.BIN_S)
    assert p["length"] == pytest.approx(ON.nav_pass_m / SPEED, abs=eta_split.BIN_S)
    assert p["err"] == pytest.approx(-STAND_S, abs=0.3)
    assert p["mps"] == pytest.approx(SPEED, abs=0.01)


def test_the_parts_add_up_to_the_error_for_every_sample(tmp_path):
    path, _ = record(tmp_path)
    rows, poses, _, _ = eta_split.run([f"r={path}"])
    for row in rows:
        p = eta_split.parts(row, poses["r"]["pode1"])
        total = p["adj"] + p["length"] + p["speed"] + p["turn"] + p["wait"] + p["unseen"]
        assert total == pytest.approx(p["err"], abs=1e-9)


def test_a_folder_finds_named_records_each_under_its_own_name(tmp_path):
    a, _ = record(tmp_path, "fleet_record_1530.jsonl")
    shutil.copy(a, tmp_path / "fms.jsonl")
    found = eta_split.records([f"real={tmp_path}"])
    assert sorted(name for name, _ in found) == ["real/fleet_record_1530.jsonl", "real/fms.jsonl"]
    rows, poses, _, _ = eta_split.run([f"real={tmp_path}"])
    assert set(poses) == {"real/fleet_record_1530.jsonl", "real/fms.jsonl"}
    text = "\n".join(eta_split.report(rows, poses))
    assert "dispatch kind=pass: n=2" in text
    assert "pose at SEGMENT_DONE kind=pass" in text


def test_a_path_with_an_equals_sign_is_read_as_a_path(tmp_path):
    folder = tmp_path / "a=b"
    folder.mkdir()
    path, _ = record(folder)
    assert eta_split.split_arg(str(path)) == ("", path)
    assert eta_split.split_arg(f"x={path}") == ("x", path)
    assert eta_split.records([f"x={folder}"])[0][1] == path

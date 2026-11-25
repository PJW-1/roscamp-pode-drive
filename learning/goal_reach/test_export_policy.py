"""export_policy.py 의 policy_spec 내용 테스트. 실행: python -m pytest learning/goal_reach"""

import dataclasses
import hashlib
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
pytest.importorskip("onnxruntime")
import export_policy as ep  # noqa: E402
import task_spec as ts  # noqa: E402

GOAL = {"rule": "path_end"}


def test_default_spec_is_format_2():
    out = ep.make_spec({}, GOAL)
    assert out["format"] == 2 and "obs_blocks" not in out and out["obs_dim"] == 41


def test_blocks_give_format_3_with_their_entries():
    spec = dataclasses.replace(
        ts.SPEC, prev_action_obs=1, lidar_frames=3, speed_cap_obs=1, speed_cap_range=(0.08, 0.16)
    )
    out = ep.make_spec({}, GOAL, spec)
    assert out["format"] == 3
    assert out["obs_blocks"] == ["prev_action", "lidar_history", "speed_cap"]
    assert out["prev_action"] == {"v_scale": 0.12, "w_scale": 1.0}
    assert out["lidar_history"] == {"frames": 3, "stride_steps": 4}
    assert out["speed_cap"] == {"min": 0.08, "max": 0.16, "obs_scale": 0.12}
    assert len(out["obs_layout"]) == out["obs_dim"] == 41 + 2 + 72 + 1


def test_a_cap_other_than_the_top_speed_without_its_block_is_refused():
    spec = dataclasses.replace(ts.SPEC, speed_cap_range=(0.08, 0.08))
    with pytest.raises(ValueError):
        ep.make_spec({}, GOAL, spec)


def _write(folder, name, data):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / name).write_text(json.dumps(data), encoding="utf-8")


def test_provenance_comes_from_run_config_with_its_sha256(tmp_path):
    cfg = {
        "format": 1,
        "max_iterations": 300,
        "seed": 5,
        "init": "D:/runs/a/model_1745.pt",
        "resume": "",
        "map": {"layout": "v7", "yaml": "x.yaml", "yaml_sha256": "ab"},
    }
    _write(tmp_path / "t", "run_config.json", cfg)
    _write(tmp_path / "t", "run.json", {"max_iterations": 1, "seed": 1})
    raw = (tmp_path / "t" / "run_config.json").read_bytes()
    out = ep.run_provenance(str(tmp_path / "t" / "model_2044.pt"), 2044)
    assert out == {
        "parent_checkpoint": "D:/runs/a/model_1745.pt",
        "iterations_this_run": 300,
        "iterations_total": 2044,
        "seed": 5,
        "map_layout": "v7",
        "run_config_sha256": hashlib.sha256(raw).hexdigest(),
    }


def test_provenance_follows_resume_chunks_back_to_the_first_init(tmp_path):
    c1, c2 = tmp_path / "c1", tmp_path / "c2"
    _write(c1, "run.json", {"max_iterations": 250, "seed": 3, "init": r"D:\r\init.pt", "resume": ""})
    _write(c2, "run.json", {"max_iterations": 250, "seed": 3, "init": "", "resume": str(c1 / "m.pt")})
    out = ep.run_provenance(str(c2 / "model_1496.pt"), 1496)
    assert out["parent_checkpoint"] == "D:/r/init.pt"
    assert out["iterations_this_run"] == 500 and out["seed"] == 3
    assert out["map_layout"] == "" and out["run_config_sha256"] == ""


def test_provenance_from_scratch_has_no_parent(tmp_path):
    _write(tmp_path, "run.json", {"max_iterations": 500, "seed": 42, "resume": ""})
    out = ep.run_provenance(str(tmp_path / "model_499.pt"), 499)
    assert out["parent_checkpoint"] == "" and out["iterations_this_run"] == 500


def test_provenance_stops_at_a_resume_whose_folder_has_no_record(tmp_path):
    _write(tmp_path / "c2", "run.json", {"max_iterations": 300, "seed": 1, "resume": "/gone/c1/m.pt"})
    out = ep.run_provenance(str(tmp_path / "c2" / "model_598.pt"), 598)
    assert out["parent_checkpoint"] == "/gone/c1/m.pt" and out["iterations_this_run"] == 300


def test_provenance_without_a_run_file_is_empty(tmp_path):
    out = ep.run_provenance(str(tmp_path / "model_1.pt"), 1)
    assert out == {
        "parent_checkpoint": "",
        "iterations_this_run": None,
        "iterations_total": 1,
        "seed": None,
        "map_layout": "",
        "run_config_sha256": "",
    }

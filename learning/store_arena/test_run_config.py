"""run_config.py 시험 (Isaac 불필요)."""

import dataclasses
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))

import arena_spec as arena  # noqa: E402
import run_config as rc  # noqa: E402
import task_spec as ts  # noqa: E402


def _config(**over):
    sets = ["prev_action_obs=1", "robot_max=2"]
    task_over, arena_over = arena.split_overrides(sets)
    kw = dict(
        argv=["train.py", "--seed", "3"],
        sets=sets,
        num_envs=64,
        max_iterations=2,
        seed=3,
        init="D:/x/model_1.pt",
        resume="",
        init_std="0.3",
        task_spec=ts.make_spec(task_over),
        arena_spec=arena.make_arena_spec(arena_over),
        agent={"algorithm": {"entropy_coef": 0.005}},
    )
    kw.update(over)
    return rc.build(**kw)


def test_the_snapshot_holds_every_spec_field_and_the_overrides():
    c = _config()
    assert c["format"] == rc.FORMAT
    assert c["task_spec"]["prev_action_obs"] == 1 and c["arena_spec"]["robot_max"] == 2
    assert set(c["task_spec"]) == {f.name for f in dataclasses.fields(ts.TaskSpec)}
    assert set(c["arena_spec"]) == {f.name for f in dataclasses.fields(arena.ArenaSpec)}
    assert c["init"] == "D:/x/model_1.pt" and c["init_std"] == [0.3, 0.3] and c["seed"] == 3
    assert set(c["git"]) == {"commit", "branch", "dirty"}


def test_the_map_entry_names_the_yaml_and_its_hash():
    m = rc.map_info("v7")
    assert m["yaml"] == "pc/fleet_core/fleet_core/data/fleet_map_v7.yaml"
    assert len(m["yaml_sha256"]) == 64
    assert rc.map_info(os.path.join(rc.REPO, m["yaml"]))["yaml_sha256"] == m["yaml_sha256"]
    assert rc.map_info("no_such_layout") == {
        "layout": "no_such_layout",
        "yaml": "",
        "yaml_sha256": "",
    }


def test_the_file_is_json_and_its_hash_is_returned(tmp_path):
    path = str(tmp_path / "run_config.json")
    sha = rc.write(path, _config())
    with open(path, encoding="utf-8") as f:
        back = json.load(f)
    assert back["task_spec"]["speed_cap_range"] == list(ts.SPEC.speed_cap_range)  # 튜플은 목록으로
    assert sha == rc.sha256_of(path) and len(sha) == 64


def test_init_std_takes_one_value_or_a_pair():
    assert rc.parse_init_std("0") == (0.0, 0.0)
    assert rc.parse_init_std("0.3") == (0.3, 0.3)
    assert rc.parse_init_std("0.3,0") == (0.3, 0.0)
    for bad in ("0.3,0.1,0.2", "-0.1", "a"):
        try:
            rc.parse_init_std(bad)
        except ValueError:
            continue
        raise AssertionError(bad)

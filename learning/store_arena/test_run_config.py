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


def test_symmetry_mode_is_recorded():
    assert _config()["symmetry"] == {"mode": "off"}
    c = _config(symmetry="both", mirror_loss_coeff=0.5)
    assert c["symmetry"] == {
        "mode": "both",
        "use_data_augmentation": True,
        "use_mirror_loss": True,
        "mirror_loss_coeff": 0.5,
    }


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


def test_init_std_refuses_values_that_are_not_finite():
    for bad in ("nan", "inf", "0.3,nan", "inf,0.1"):
        try:
            rc.parse_init_std(bad)
        except ValueError:
            continue
        raise AssertionError(bad)


def test_the_map_entry_resolves_a_layout_the_way_the_map_loader_does(tmp_path, monkeypatch):
    from fleet_core.fleet_map import fleet_map as fm

    assert os.path.samefile(os.path.join(rc.REPO, rc.MAP_DIR), fm.DATA)
    # 확장자 없는 이름은 작업 폴더에 같은 이름의 파일이 있어도 지도 이름이다
    (tmp_path / "v7").write_text("not a map", encoding="utf-8")
    monkeypatch.chdir(tmp_path)
    assert rc.map_info("v7")["yaml"] == "pc/fleet_core/fleet_core/data/fleet_map_v7.yaml"
    # 저장소 밖의 yaml 은 절대 경로로 적는다 (다른 드라이브여도 예외 없이)
    other = tmp_path / "other.yaml"
    other.write_text("a: 1", encoding="utf-8")
    m = rc.map_info(str(other))
    assert os.path.isabs(m["yaml"]) and m["yaml"].endswith("other.yaml")
    assert m["yaml_sha256"] == rc.sha256_of(str(other))
    monkeypatch.setattr(rc.os.path, "relpath", lambda *a, **k: (_ for _ in ()).throw(ValueError()))
    assert os.path.isabs(rc.map_info("v7")["yaml"])

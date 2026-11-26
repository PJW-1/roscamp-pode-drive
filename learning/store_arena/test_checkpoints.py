"""checkpoints/ 의 백업이 index.json, 정책 폴더의 policy_spec.json 과 맞는지 (Isaac 불필요)."""

import hashlib
import json
import os

import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "checkpoints")
POLICIES = os.path.join(HERE, "..", "..", "pode", "pode_rl_controller", "policies")

with open(os.path.join(ROOT, "index.json"), encoding="utf-8") as f:
    INDEX = json.load(f)


def test_every_checkpoint_file_is_listed_once():
    files = sorted(
        f"{d}/{name}"
        for d in os.listdir(ROOT)
        if os.path.isdir(os.path.join(ROOT, d))
        for name in os.listdir(os.path.join(ROOT, d))
        if name.endswith(".pt")
    )
    assert files == sorted(e["file"] for e in INDEX)


@pytest.mark.parametrize("entry", INDEX, ids=[e["policy"] for e in INDEX])
def test_the_file_matches_its_index_line_and_the_policy_spec(entry):
    path = os.path.join(ROOT, entry["file"])
    with open(path, "rb") as f:
        data = f.read()
    assert hashlib.sha256(data).hexdigest() == entry["sha256"] and len(data) == entry["bytes"]
    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    assert int(ckpt["iter"]) == entry["iter"]
    assert ckpt["actor_state_dict"]["mlp.0.weight"].shape[1] == entry["obs_dim"]
    with open(os.path.join(POLICIES, entry["policy"], "policy_spec.json"), encoding="utf-8") as f:
        spec = json.load(f)
    assert spec["obs_dim"] == entry["obs_dim"]
    assert spec["source"]["iterations_total"] == entry["iter"]
    assert spec["source"]["checkpoint_path"] == entry["copied_from"]
    assert os.path.isfile(os.path.join(ROOT, entry["run_json"]))


@pytest.mark.parametrize("entry", INDEX, ids=[e["policy"] for e in INDEX])
def test_a_run_config_hash_in_run_json_points_at_a_file_kept_beside_it(entry):
    """run.json 이 run_config_sha256 을 적으면 같은 폴더의 run_config.json 해시가 그 값이다."""
    run_json = os.path.join(ROOT, entry["run_json"])
    with open(run_json, encoding="utf-8") as f:
        want = json.load(f).get("run_config_sha256")
    if want is None:
        pytest.skip("run_config.json 이 생기기 전의 실행")
    with open(os.path.join(os.path.dirname(run_json), "run_config.json"), "rb") as f:
        assert hashlib.sha256(f.read()).hexdigest() == want

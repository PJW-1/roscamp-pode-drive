"""RL 정책이 저장소만으로 도는지 확인한다: 명세와 onnx가 함께 있고, onnx가 git에 올라 있고,
parity 목록과 시뮬 덮어쓰기 파일이 정책 폴더와 맞는다."""
import json
import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
POLICIES = REPO / "pode" / "pode_rl_controller" / "policies"
OVERRIDES = REPO / "sim" / "fleet_gz_sim" / "nav_overrides"
DEFAULT_POLICY = "store_arena_4b"


def _names():
    return sorted(p.name for p in POLICIES.iterdir() if p.is_dir())


def test_every_policy_folder_has_its_spec_and_onnx():
    """정책 폴더마다 policy_spec.json과 policy.onnx가 둘 다 있다. 하나만 있으면 다른 PC에서 못 돈다."""
    assert DEFAULT_POLICY in _names()
    for name in _names():
        assert (POLICIES / name / "policy_spec.json").is_file(), name
        assert (POLICIES / name / "policy.onnx").stat().st_size > 1000, name


def test_every_policy_onnx_is_tracked_by_git():
    """.gitignore의 *.onnx 예외가 정책 폴더 전부에 걸려 onnx가 커밋된다."""
    if not (REPO / ".git").exists():
        pytest.skip("git 작업본이 아니다(복사본)")
    out = subprocess.run(["git", "ls-files", "pode/pode_rl_controller/policies"], cwd=REPO,
                         capture_output=True, text=True, check=True).stdout.split()
    for name in _names():
        assert f"pode/pode_rl_controller/policies/{name}/policy.onnx" in out, name


def test_the_parity_list_names_every_policy_folder():
    """CMake의 parity 목록이 정책 폴더와 같아야 정책마다 C++ 추론 비교가 돈다."""
    text = (REPO / "pode" / "pode_rl_controller" / "CMakeLists.txt").read_text(encoding="utf-8")
    listed = re.search(r"foreach\(policy ([^)]*)\)", text).group(1).split()
    assert sorted(listed) == _names()
    assert "set(onnx_required 1)" in text


def test_the_onnx_input_and_output_match_the_spec():
    """onnx의 입력 칸 수와 출력 칸 수가 명세의 obs_dim, act_dim과 같다."""
    onnx = pytest.importorskip("onnx")
    for name in _names():
        spec = json.loads((POLICIES / name / "policy_spec.json").read_text(encoding="utf-8"))
        graph = onnx.load(str(POLICIES / name / "policy.onnx")).graph
        dims = [[d.dim_value for d in v.type.tensor_type.shape.dim]
                for v in (graph.input[0], graph.output[0])]
        assert dims == [[1, spec["obs_dim"]], [1, spec["act_dim"]]], name


def test_a_policy_override_file_differs_from_the_default_only_in_its_policy():
    """follow_path_rl_<이름>.yaml은 follow_path_rl.yaml과 정책 이름만 다르고, 그 정책 폴더가 있다."""
    base = yaml.safe_load((OVERRIDES / "follow_path_rl.yaml").read_text(encoding="utf-8"))
    others = sorted(OVERRIDES.glob("follow_path_rl_*.yaml"))
    assert [p.name for p in others] == ["follow_path_rl_acd3.yaml", "follow_path_rl_demohist.yaml",
                                        "follow_path_rl_demohist2.yaml", "follow_path_rl_std.yaml"]
    for path in others:
        policy = "store_arena_" + path.stem.removeprefix("follow_path_rl_")
        assert policy in _names()
        want = json.loads(json.dumps(base).replace(DEFAULT_POLICY, policy))
        assert yaml.safe_load(path.read_text(encoding="utf-8")) == want

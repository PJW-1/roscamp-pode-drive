"""cases.yaml이 저장소 설정과 맞는지 본다(ROS 없이)."""
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]
CASES = yaml.safe_load((PKG / "config" / "cases.yaml").read_text(encoding="utf-8"))
BENCH = CASES["compute_bench"]["ros__parameters"]
NAV2 = yaml.safe_load(
    (REPO / "pode" / "pode_nav_config" / "params" / "nav2_params.yaml").read_text(encoding="utf-8"))
SERVER = NAV2["controller_server"]["ros__parameters"]
RL_FILE = REPO / "sim" / "fleet_gz_sim" / "nav_overrides" / "follow_path_rl.yaml"
MPPI = [c for c in BENCH["cases"] if c.startswith("mppi_")]


def test_every_case_is_defined():
    assert set(BENCH["cases"]) == {k for k in BENCH if k != "cases"}


def test_rpp_is_the_real_follow_path():
    assert BENCH["rpp"] == {"bench_source": "FollowPath",
                            "bench_rate_hz": SERVER["controller_frequency"]}
    assert "FollowPath" in SERVER["controller_plugins"]
    assert SERVER["FollowPath"]["plugin"] == \
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"


def test_mppi_l0_is_the_sim_setting():
    src = SERVER["FollowPathMPPI"]
    assert src["primary_controller"] == "nav2_mppi_controller::MPPIController"
    l0 = BENCH["mppi_l0"]
    for key in ("batch_size", "time_steps", "model_dt"):
        assert l0[key] == src[key], key
    assert l0["bench_rate_hz"] == SERVER["controller_frequency"]


@pytest.mark.parametrize("case", MPPI)
def test_mppi_level_rate_matches_model_dt(case):
    c = BENCH[case]
    assert c["bench_source"] == "FollowPathMPPI"
    assert c["plugin"] == "nav2_mppi_controller::MPPIController"
    assert 1.0 / c["bench_rate_hz"] == pytest.approx(c["model_dt"])


def test_mppi_levels_get_lighter():
    work = [BENCH[c]["batch_size"] * BENCH[c]["time_steps"] for c in MPPI]
    assert len(MPPI) in (4, 5)
    assert work == sorted(work, reverse=True) and len(set(work)) == len(work)


@pytest.mark.skipif(not RL_FILE.exists(), reason="sim overrides not in this tree")
def test_rl_matches_sim_follow_path_rl_alone():
    sim = yaml.safe_load(RL_FILE.read_text(encoding="utf-8"))
    alone = sim["controller_server"]["ros__parameters"]["FollowPathRLAlone"]
    ours = {k: v for k, v in BENCH["rl"].items() if not k.startswith("bench_")}
    assert ours.pop("scan_topic") == "bench_scan"
    alone = {k: v for k, v in alone.items() if k != "scan_topic"}
    assert ours == alone

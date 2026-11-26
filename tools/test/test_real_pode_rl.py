"""실물 RL 선택(make_params.py --rl-policy, start.sh RL_POLICY, deploy.sh WITH_RL)을 ROS 없이 본다."""
import copy
import os
import re
import shutil
import subprocess
import sys
import time
import types
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
KIT = ROOT / "tools" / "real_pode"
SIM_RL = ROOT / "sim" / "fleet_gz_sim" / "nav_overrides" / "follow_path_rl.yaml"
POLICIES = ROOT / "pode" / "pode_rl_controller" / "policies"
# RL 선택을 더하기 전 커밋(기본 출력 비교의 기준)
BASE_COMMIT = "951d71bb"
RL_NAMES = ("FollowPathRL", "FollowPathRLAlone")
# RL 선택 전 deploy.sh가 보내던 패키지(기본 배포 비교의 기준)
OLD_PACKAGES = ("interfaces/fleet_interfaces", "pode/pode_fsm", "pode/pode_nav_config",
                "pode/pode_avoidance", "pode/pode_peer_obstacles", "pode/pode_scan_filter",
                "pode/pode_dock", "pode/pode_plan_keep")

sys.path.insert(0, str(KIT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

needs_bash = pytest.mark.skipif(os.name == "nt", reason="bash 스크립트 실행은 Linux(WSL)에서 본다")


def _mp():
    import make_params
    return make_params


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _ctl(data):
    return data["controller_server"]["ros__parameters"]


# ---- 기본 출력은 RL 선택 전과 바이트까지 같다 ----
def _base_module():
    run = subprocess.run(["git", "-C", str(ROOT), "show",
                          f"{BASE_COMMIT}:tools/real_pode/make_params.py"],
                         capture_output=True, text=True, encoding="utf-8")
    if run.returncode != 0:
        pytest.skip(f"기준 커밋 {BASE_COMMIT}이 이 작업본에 없다")
    mod = types.ModuleType("make_params_base")
    mod.__file__ = str(KIT / "make_params.py")
    exec(compile(run.stdout, "make_params_base", "exec"), mod.__dict__)
    return mod


@pytest.mark.parametrize("speed", [0.12, 0.05, 0.1])
@pytest.mark.parametrize("fixed", [False, True])
def test_default_output_is_byte_identical_to_before(tmp_path, speed, fixed):
    base = _base_module()
    base.write_params(speed, tmp_path / "old", fixed_line_check=fixed)
    _mp().write_params(speed, tmp_path / "new", fixed_line_check=fixed)
    for name in ("nav2_params.yaml", "avoidance.yaml"):
        assert (tmp_path / "new" / name).read_bytes() == (tmp_path / "old" / name).read_bytes()


def test_default_cli_has_no_rl(tmp_path):
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.12",
                          "--out", str(tmp_path / "p")], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    assert "rl controllers" not in run.stdout
    ctl = _ctl(_load(tmp_path / "p" / "nav2_params.yaml"))
    assert not set(RL_NAMES) & set(ctl) and not set(RL_NAMES) & set(ctl["controller_plugins"])


# ---- --rl-policy: 시뮬 파일의 두 블록, policy만 바뀜 ----
def _policies():
    return sorted(p.name for p in POLICIES.iterdir() if p.is_dir())


@pytest.mark.parametrize("policy", ["store_arena_acd3", *_policies()])
def test_rl_output_is_the_sim_blocks_with_the_policy(tmp_path, policy):
    mp = _mp()
    mp.write_params(0.12, tmp_path / "d")
    mp.write_params(0.12, tmp_path / "r", rl_policy=policy)
    default, rl, sim = (_load(tmp_path / "d" / "nav2_params.yaml"),
                        _load(tmp_path / "r" / "nav2_params.yaml"), _load(SIM_RL))
    plugins = _ctl(rl)["controller_plugins"]
    assert plugins == _ctl(default)["controller_plugins"] + list(RL_NAMES)
    assert tuple(plugins[-2:]) == RL_NAMES and "FollowPathMPPI" not in plugins
    for name in RL_NAMES:
        want = copy.deepcopy(_ctl(sim)[name])
        want["policy"] = policy
        assert _ctl(rl)[name] == want
    # 그 밖에는 기본 출력과 같다(avoidance 사본 포함)
    rest = copy.deepcopy(rl)
    for name in RL_NAMES:
        del _ctl(rest)[name]
    _ctl(rest)["controller_plugins"] = _ctl(default)["controller_plugins"]
    assert rest == default
    assert ((tmp_path / "r" / "avoidance.yaml").read_bytes()
            == (tmp_path / "d" / "avoidance.yaml").read_bytes())


def test_rl_text_only_adds_lines_and_changes_the_plugin_line(tmp_path):
    mp = _mp()
    mp.write_params(0.12, tmp_path / "d")
    mp.write_params(0.12, tmp_path / "r", rl_policy="store_arena_acd3")
    old = (tmp_path / "d" / "nav2_params.yaml").read_text(encoding="utf-8").splitlines()
    new = (tmp_path / "r" / "nav2_params.yaml").read_text(encoding="utf-8").splitlines()
    at = next(i for i, line in enumerate(old) if line.startswith("    controller_plugins:"))
    added = len(new) - len(old)
    assert added > 0
    assert new[:at] == old[:at] and new[at + 1 + added:] == old[at + 1:]


def test_rl_follows_the_sim_file_values(tmp_path):
    # 시뮬 파일 값을 바꾸면 실물 사본도 따라간다(출처는 하나)
    src = tmp_path / "follow_path_rl.yaml"
    src.write_text(SIM_RL.read_text(encoding="utf-8").replace("scan_timeout: 0.5",
                                                              "scan_timeout: 0.7"),
                   encoding="utf-8")
    _mp().write_params(0.12, tmp_path / "r", rl_policy="store_arena_acd3", rl_src=src)
    ctl = _ctl(_load(tmp_path / "r" / "nav2_params.yaml"))
    assert all(ctl[name]["scan_timeout"] == 0.7 for name in RL_NAMES)


def test_rl_cli(tmp_path):
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.12",
                          "--out", str(tmp_path / "p"), "--rl-policy", "store_arena_acd3"],
                         capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    assert "policy store_arena_acd3" in run.stdout and "WARNING" not in run.stdout
    ctl = _ctl(_load(tmp_path / "p" / "nav2_params.yaml"))
    assert ctl["FollowPathRL"]["policy"] == "store_arena_acd3"
    # 쓰는 controller 이름은 행동 트리가 정한다: 사본은 RPP용 블록을 그대로 둔다
    assert "FollowPathGoal" in ctl["controller_plugins"]


def test_rl_cli_warns_that_speed_does_not_cap_rl(tmp_path):
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.06",
                          "--out", str(tmp_path / "p"), "--rl-policy", "store_arena_acd3"],
                         capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    assert "WARNING rl controllers ignore --speed" in run.stdout


@pytest.mark.parametrize("name", ["nope", "", "../store_arena_4b", "store_arena_4b/",
                                  "/abs/store_arena_4b", "store arena", "store_arena_4b;ls"])
def test_rl_bad_policy_name_writes_nothing(tmp_path, name):
    with pytest.raises(ValueError):
        _mp().write_params(0.12, tmp_path / "o", rl_policy=name)
    assert not (tmp_path / "o").exists()
    run = subprocess.run([sys.executable, str(KIT / "make_params.py"), "--speed", "0.12",
                          "--out", str(tmp_path / "c"), "--rl-policy", name],
                         capture_output=True, text=True)
    assert run.returncode == 2 and "rl policy" in run.stderr
    assert not (tmp_path / "c").exists()


@pytest.mark.parametrize("missing", ["policy.onnx", "policy_spec.json"])
def test_rl_policy_folder_without_a_file_writes_nothing(tmp_path, missing):
    folder = tmp_path / "policies" / "half"
    folder.mkdir(parents=True)
    for name in ("policy.onnx", "policy_spec.json"):
        if name != missing:
            (folder / name).write_bytes(b"x")
    with pytest.raises(FileNotFoundError, match=missing):
        _mp().write_params(0.12, tmp_path / "o", rl_policy="half",
                           policies_dir=tmp_path / "policies")
    assert not (tmp_path / "o").exists()


def test_rl_unknown_name_lists_the_known_ones(tmp_path):
    with pytest.raises(ValueError, match="store_arena_acd3"):
        _mp().write_params(0.12, tmp_path / "o", rl_policy="store_arena_acd9")


# ---- 스크립트: RL 줄은 선택했을 때만 닿는다(정적 검사) ----
def _guarded(text, guard):
    """guard로 여는 if 블록 안의 줄 번호."""
    inside, depth, rows = False, 0, set()
    for i, line in enumerate(text.splitlines()):
        if not inside and line.strip() == guard:
            inside, depth = True, 0
        if inside:
            rows.add(i)
            depth += len(re.findall(r"^\s*if\b", line)) - (1 if line.strip() == "fi" else 0)
            if depth == 0:
                inside = False
    return rows


def test_deploy_rl_lines_are_behind_with_rl():
    text = (KIT / "deploy.sh").read_text(encoding="utf-8")
    rows = _guarded(text, 'if [ -n "$with_rl" ]; then')
    assert rows
    defs = {"rl_pkg=pode/pode_rl_controller",
            "rl_yaml=sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml"}
    for i, line in enumerate(text.splitlines()):
        if line.lstrip().startswith("#") or line in defs:
            continue
        if re.search(r"rl_pkg|rl_yaml|onnxruntime|pode_rl_controller|--packages-select", line):
            assert i in rows, f"deploy.sh:{i + 1} reaches RL outside the WITH_RL block: {line}"
    # 기본 패키지 목록에는 RL이 없다
    assert "pode_rl_controller" not in re.search(r"^packages=\(([^)]*)\)", text, re.M).group(1)


def test_start_rl_lines_are_behind_rl_policy():
    text = (KIT / "start.sh").read_text(encoding="utf-8")
    for i, line in enumerate(text.splitlines()):
        if line.lstrip().startswith("#"):
            continue
        if "--rl-policy" in line:
            assert line.startswith('[ -z "$rlp" ] || rl=('), f"start.sh:{i + 1}: {line}"
        if "log_level" in line:
            assert line.strip().startswith('[ -z "$9" ] || extra+=(log_level:=info)'), \
                f"start.sh:{i + 1}: {line}"
    assert "rl=()" in text.splitlines()


# ---- 스크립트 실행(대역 ssh, rsync, ros2, make_params) ----
def _fake_bin(tmp_path, scripts):
    fake = tmp_path / "bin"
    fake.mkdir(exist_ok=True)
    for name, body in scripts.items():
        (fake / name).write_text("#!/bin/bash\n" + body)
        (fake / name).chmod(0o755)
    return fake


def _deploy(tmp_path, ort_ok, shim_ok=True, **extra):
    calls = tmp_path / "calls"
    ort, shim = ("0" if ort_ok else "1"), ("0" if shim_ok else "1")
    fake = _fake_bin(tmp_path, {
        "ssh": f'printf "ssh %s\\n" "$*" >> "{calls}"\n'
               f'case "$*" in *onnxruntime_vendor*) exit {ort} ;;'
               f' *rotation_shim*) exit {shim} ;; esac\nexit 0\n',
        "rsync": f'printf "rsync %s\\n" "$*" >> "{calls}"\n',
    })
    env = {k: v for k, v in os.environ.items() if k not in ("WITH_RL", "TMPDIR")}
    env.update(PATH=f"{fake}:{os.environ['PATH']}", **extra)
    run = subprocess.run(["bash", str(KIT / "deploy.sh"), "pinky@robot"], env=env,
                         capture_output=True, text=True, timeout=120)
    return run, calls.read_text().splitlines() if calls.exists() else []


@needs_bash
@pytest.mark.parametrize("extra", [{}, {"WITH_RL": "0"}, {"WITH_RL": ""}])
def test_deploy_default_touches_no_rl(tmp_path, extra):
    run, calls = _deploy(tmp_path, ort_ok=False, **extra)
    assert run.returncode == 0, run.stdout + run.stderr
    # 임시 폴더 이름만 빼고 예전 deploy.sh와 같은 호출이어야 한다
    got = [re.sub(r"/tmp/tmp\.[A-Za-z0-9]+/", "TMP/", c) for c in calls]
    want = ["ssh pinky@robot mkdir -p ~/nav2fleet_ws/src/interfaces ~/nav2fleet_ws/src/pode "
            "~/nav2fleet_real"]
    want += [f"rsync -a --delete TMP/{pkg}/ pinky@robot:~/nav2fleet_ws/src/{pkg}/"
             for pkg in OLD_PACKAGES]
    want.append("rsync -a TMP/tools/real_pode/ pinky@robot:~/nav2fleet_real/")
    assert got == want


@needs_bash
def test_deploy_rl_without_onnxruntime_changes_nothing(tmp_path):
    run, calls = _deploy(tmp_path, ort_ok=False, WITH_RL="1")
    assert run.returncode == 1
    assert "sudo apt install ros-jazzy-onnxruntime-vendor" in run.stdout
    assert len(calls) == 1 and "onnxruntime_vendor" in calls[0]


@needs_bash
def test_deploy_rl_without_rotation_shim_changes_nothing(tmp_path):
    run, calls = _deploy(tmp_path, ort_ok=True, shim_ok=False, WITH_RL="1")
    assert run.returncode == 1
    assert "ros-jazzy-nav2-rotation-shim-controller" in run.stdout
    assert len(calls) == 2 and "rotation_shim" in calls[1]


@needs_bash
def test_deploy_rl_ships_and_builds_the_package(tmp_path):
    run, calls = _deploy(tmp_path, ort_ok=True, WITH_RL="1")
    assert run.returncode == 0, run.stdout + run.stderr
    assert "onnxruntime_vendor" in calls[0]
    rsyncs = [c for c in calls if c.startswith("rsync")]
    assert any("pode/pode_rl_controller/ pinky@robot:~/nav2fleet_ws/src/pode/pode_rl_controller/"
               in c for c in rsyncs)
    assert any(c.endswith("pinky@robot:~/nav2fleet_ws/src/sim/fleet_gz_sim/nav_overrides/"
                          "follow_path_rl.yaml") for c in rsyncs)
    build = [c for c in calls if "colcon build" in c]
    assert len(build) == 1 and "--packages-select pode_rl_controller" in build[0]


@needs_bash
def test_deploy_refuses_bad_with_rl(tmp_path):
    run, calls = _deploy(tmp_path, ort_ok=True, WITH_RL="yes")
    assert run.returncode == 2 and "bad WITH_RL" in run.stdout and calls == []


def _start(tmp_path, **extra):
    """start.sh는 진짜, make_params와 ros2는 받은 인자를 적는 대역."""
    from test_real_pode import _robot_home
    kit = tmp_path / "kit"
    kit.mkdir()
    for name in ("start.sh", "env.sh", "net.sh"):
        shutil.copy(KIT / name, kit)
    mp_args, ros_args = tmp_path / "mp.args", tmp_path / "ros2.args"
    (kit / "make_params.py").write_text(
        f"import sys\nopen({str(mp_args)!r}, 'w').write('\\n'.join(sys.argv[1:]))\n")
    fake = _fake_bin(tmp_path, {"ros2": f'printf "%s\\n" "$@" > "{ros_args}"\n'})
    env = {"PATH": f"{fake}:/usr/bin:/bin", **_robot_home(tmp_path), **extra}
    run = subprocess.run(["bash", str(kit / "start.sh"), "pode1", "0.12", "0.10", "1.5708"],
                         env=env, capture_output=True, text=True, timeout=30)
    for _ in range(50):
        if ros_args.exists() and ros_args.read_text().strip():
            break
        time.sleep(0.1)
    read = (lambda p: p.read_text().split("\n") if p.exists() else None)
    return run, read(mp_args), read(ros_args)


@needs_bash
@pytest.mark.parametrize("extra", [{}, {"RL_POLICY": ""}])
def test_start_default_passes_no_rl(tmp_path, extra):
    run, mp_args, ros_args = _start(tmp_path, **extra)
    assert run.returncode == 0, run.stdout + run.stderr
    assert mp_args == ["--speed", "0.12", "--out", "params"]
    assert ros_args == ["launch", "./real_pode.launch.py", "namespace:=pode1", "initial_x:=0.12",
                        "initial_y:=0.10", "initial_yaw:=1.5708", "plan_keep:=true", ""]
    assert "rl policy" not in run.stdout


@needs_bash
def test_start_rl_policy_passes_the_name_and_info_log(tmp_path):
    run, mp_args, ros_args = _start(tmp_path, RL_POLICY="store_arena_acd3")
    assert run.returncode == 0, run.stdout + run.stderr
    assert mp_args == ["--speed", "0.12", "--out", "params", "--rl-policy", "store_arena_acd3"]
    assert ros_args[-2:] == ["log_level:=info", ""]
    assert "rl policy store_arena_acd3" in run.stdout


@needs_bash
def test_no_motion_refuses_rl_policy(tmp_path):
    from test_real_pode import _no_motion_start
    run, argv = _no_motion_start(tmp_path, ["pode1"], NO_MOTION="1", RL_POLICY="store_arena_acd3")
    assert run.returncode == 2 and argv == []
    assert "RL_POLICY" in run.stdout + run.stderr

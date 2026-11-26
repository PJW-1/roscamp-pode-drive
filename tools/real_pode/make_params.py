#!/usr/bin/env python3
"""실물 Pode용 Nav2 설정 사본을 속도 S로 만든다. ROS 없이 돈다.

    python3 make_params.py --speed 0.12 --out params

앞으로 가는 컨트롤러의 desired_linear_vel을 S로 바꾼다.
고정 경로 컨트롤러 셋의 충돌 검사는 기본으로 끈다(--fixed-line-check로 켬).
--rl-policy NAME을 주면 RL controller 둘(FollowPathRL, FollowPathRLAlone)을 더한다.
값은 시뮬 sim/fleet_gz_sim/nav_overrides/follow_path_rl.yaml에서 그대로 가져오고 policy만 NAME으로 바꾼다.
--rl-scan-mask "lo,hi,..."를 함께 주면 두 블록의 policy 줄 다음에
scan_mask_deg 한 줄씩을 더한다.
"""
import argparse
import copy
import json
import math
import re
import sys
from decimal import Decimal
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
NAV2_NAME = "nav2_params.yaml"
AVOID_NAME = "avoidance.yaml"
NAV2_REL = Path("pode") / "pode_nav_config" / "params" / NAV2_NAME
AVOID_REL = Path("pode") / "pode_avoidance" / "params" / AVOID_NAME
SPEED_REL = Path("pode") / "pode_nav_config" / "params" / "nav_speed.yaml"
# 저장소 안(PC)과 로봇(~/nav2fleet_ws/src, deploy.sh가 놓는 곳) 순서로 찾는다
SOURCE_ROOTS = (HERE.parents[1], Path.home() / "nav2fleet_ws" / "src")
NAV2_TARGETS = (("controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"),
                ("controller_server", "ros__parameters", "FollowPathThrough",
                 "desired_linear_vel"))
COLLISION_TARGETS = tuple(("controller_server", "ros__parameters", name, "use_collision_detection")
                          for name in ("FollowPath", "FollowPathThrough", "FollowPathReverse"))
AVOID_TARGETS = (("follow_path_goal", "desired_linear_vel"),)
FORWARD = ("FollowPath", "FollowPathThrough")
# RL controller(선택): 시뮬 덮어쓰기 파일 하나가 값의 출처다
RL_REL = Path("sim") / "fleet_gz_sim" / "nav_overrides" / "follow_path_rl.yaml"
POLICIES_REL = Path("pode") / "pode_rl_controller" / "policies"
RL_NAMES = ("FollowPathRL", "FollowPathRLAlone")
CTL = ("controller_server", "ros__parameters")
POLICY_NAME = re.compile(r"^[A-Za-z0-9_]+$")
# scan 가림 각의 한 항목: 부호, 소수, 지수만 받는다(nan, inf, 1_0은 거절)
MASK_NUMBER = re.compile(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$")


def find_source(rel: Path, given=None) -> Path:
    if given:
        return Path(given)
    for root in SOURCE_ROOTS:
        if (root / rel).is_file():
            return root / rel
    raise FileNotFoundError(f"{rel} not found under {', '.join(map(str, SOURCE_ROOTS))}")


def repo_speed(path=None) -> float:
    text = find_source(SPEED_REL, path).read_text(encoding="utf-8")
    return float(yaml.safe_load(text)["nav_speed_mps"])


MAX_SPEED = repo_speed()


# ---- 속도 확인 ----
def checked_speed(speed) -> float:
    value = float(speed)
    if not math.isfinite(value):
        raise ValueError(f"speed must be a finite number, got {speed!r}")
    # 반올림 전에 범위를 본다(0.25001이 0.25로 빠져나가지 않게)
    if not 0.0 < value <= MAX_SPEED or round(value, 4) <= 0.0:
        raise ValueError(f"speed must be above 0 and at most {MAX_SPEED} m/s, got {speed!r}")
    value = round(value, 4)
    if not 0.0 < value <= MAX_SPEED:
        raise ValueError(f"speed must be above 0 and at most {MAX_SPEED} m/s, got {speed!r}")
    return value


def _text(value: float) -> str:
    out = f"{value:.4f}".rstrip("0")
    return out + "0" if out.endswith(".") else out


# ---- 줄 단위 바꾸기(주석 보존) ----
def replace_values(text: str, targets, value, note=None) -> str:
    """targets 경로의 스칼라 값 줄만 바꾼다. 경로마다 정확히 한 번 바뀌어야 한다."""
    if note is None:
        note = f"make_params.py --speed (저장소 {MAX_SPEED})"
    shown = _text(value) if isinstance(value, float) else str(value).lower()
    lines = text.splitlines(keepends=True)
    stack = []  # (들여쓰기, 키)
    hits = {t: 0 for t in targets}
    for i, line in enumerate(lines):
        body = line.split("#", 1)[0].rstrip()
        stripped = body.lstrip()
        if not stripped or stripped.startswith("-") or ":" not in stripped:
            continue
        indent = len(body) - len(stripped)
        key = stripped.split(":", 1)[0].strip().strip("\"'")
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, key))
        path = tuple(k for _, k in stack)
        if path in hits:
            end = "\n" if line.endswith("\n") else ""
            lines[i] = " " * indent + f"{key}: {shown}  # {note}{end}"
            hits[path] += 1
    bad = {".".join(t): n for t, n in hits.items() if n != 1}
    if bad:
        raise RuntimeError(f"expected each target once, found {bad}")
    return "".join(lines)


def _set(data: dict, path, value) -> None:
    for key in path[:-1]:
        data = data[key]
    data[path[-1]] = value


def _converted(src: Path, targets, value: float, collision=(), collision_on=False) -> str:
    text = src.read_text(encoding="utf-8")
    out = replace_values(text, targets, value)
    if collision:
        out = replace_values(out, collision, collision_on,
                             f"make_params.py 고정 경로 충돌 검사 {'켬' if collision_on else '끔'}")
    want = copy.deepcopy(yaml.safe_load(text))
    for path in targets:
        _set(want, path, value)
    for path in collision:
        _set(want, path, collision_on)
    if yaml.safe_load(out) != want:
        raise RuntimeError(f"{src}: the copy differs from the source beyond the changed values")
    return out


# ---- RL controller 더하기(--rl-policy) ----
def checked_policy(name, policies_dir=None) -> str:
    """정책 이름을 확인한다. 정책 폴더에 policy.onnx와 policy_spec.json이 있어야 한다."""
    name = str(name)
    if not POLICY_NAME.match(name):
        raise ValueError(f"rl policy must be a folder name under {POLICIES_REL} "
                         f"(letters, digits, _), got {name!r}")
    if policies_dir is None:
        found = [root / POLICIES_REL for root in SOURCE_ROOTS if (root / POLICIES_REL).is_dir()]
        if not found:
            raise FileNotFoundError(f"{POLICIES_REL} not found under "
                                    f"{', '.join(map(str, SOURCE_ROOTS))} (deploy with WITH_RL=1)")
        policies_dir = found[0]
    policies_dir = Path(policies_dir)
    folder = policies_dir / name
    if not folder.is_dir():
        known = sorted(p.name for p in policies_dir.iterdir() if p.is_dir())
        raise ValueError(f"unknown rl policy {name!r}: no folder {folder} "
                         f"(known: {', '.join(known)})")
    for need in ("policy.onnx", "policy_spec.json"):
        if not (folder / need).is_file():
            raise FileNotFoundError(f"rl policy {name!r}: {folder / need} is missing")
    return name


def checked_scan_mask(text) -> list:
    """scan 가림 각 "lo,hi,lo,hi,..."(도)를 확인해 float 목록으로 돌려준다.

    규칙은 플러그인 parse_scan_mask와 같다: 짝수 개, 모두 유한,
    쌍마다 -180 <= lo < hi <= 180.
    여기서는 빈 목록도 거절한다(가리지 않으려면 옵션을 주지 않는다).
    """
    items = str(text).split(",")
    if not all(MASK_NUMBER.match(item.strip()) for item in items):
        raise ValueError(f"rl scan mask must be comma-separated numbers in degrees, got {text!r}")
    values = [float(item) for item in items]
    if len(values) % 2:
        raise ValueError(f"rl scan mask must list pairs lo,hi,lo,hi,... in degrees, "
                         f"got {len(values)} numbers")
    for i in range(0, len(values), 2):
        lo, hi = values[i], values[i + 1]
        if not (math.isfinite(lo) and math.isfinite(hi) and -180.0 <= lo < hi <= 180.0):
            raise ValueError(f"rl scan mask range {i // 2} [{items[i].strip()}, "
                             f"{items[i + 1].strip()}] must satisfy -180 <= lo < hi <= 180 "
                             f"(split a range across 180 into two)")
    return values


def _mask_text(values) -> str:
    """YAML이 double 배열로 읽게 늘 소수점을 붙여 적는다(지수 표기 없이)."""
    shown = []
    for value in values:
        out = format(Decimal(repr(value)), "f")
        shown.append(out if "." in out else out + ".0")
    return "[" + ", ".join(shown) + "]"


def _rl_blocks(rl_src: Path, policy: str, scan_mask=None) -> tuple:
    """시뮬 파일에서 RL controller 두 블록의 줄을 꺼내 policy만 바꾼다. (줄 문자열, 기대 dict).

    scan_mask가 있으면 policy 줄마다 바로 다음에 같은 들여쓰기로
    scan_mask_deg 줄을 넣는다.
    """
    text = rl_src.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    ctl = data[CTL[0]][CTL[1]]
    if set(ctl) != {"controller_plugins", *RL_NAMES} or \
            tuple(ctl["controller_plugins"][-len(RL_NAMES):]) != RL_NAMES:
        raise RuntimeError(f"{rl_src}: expected controller_plugins ending with {RL_NAMES} "
                           f"and only those two blocks")
    lines = text.splitlines(keepends=True)
    head = re.compile(r"^    (" + "|".join(RL_NAMES) + r"):\s*(#.*)?$")
    starts = [i for i, line in enumerate(lines) if head.match(line.rstrip("\r\n"))]
    if len(starts) != len(RL_NAMES):
        raise RuntimeError(f"{rl_src}: expected one line per block {RL_NAMES}, found {len(starts)}")
    # 두 블록은 파일 끝에 붙어 있다(첫 블록 머리부터 끝까지)
    block = lines[starts[0]:]
    if any(line.strip() and not line.startswith("    ") for line in block):
        raise RuntimeError(f"{rl_src}: the RL blocks must be the last part of the file")
    hits = 0
    for i, line in enumerate(block):
        new, n = re.subn(r'^(\s+policy:\s*)"[^"]*"', rf'\g<1>"{policy}"', line)
        block[i], hits = new, hits + n
        if n and scan_mask is not None:
            indent = line[:len(line) - len(line.lstrip())]
            block[i] += (f"{indent}scan_mask_deg: {_mask_text(scan_mask)}"
                         f"  # make_params.py --rl-scan-mask\n")
    if hits != len(RL_NAMES):
        raise RuntimeError(f"{rl_src}: expected {len(RL_NAMES)} quoted policy lines, found {hits}")
    want = {name: copy.deepcopy(ctl[name]) for name in RL_NAMES}
    for name in RL_NAMES:
        want[name]["policy"] = policy
        if scan_mask is not None:
            if "scan_mask_deg" in want[name]:
                raise RuntimeError(f"{rl_src}: {name} already has scan_mask_deg")
            want[name]["scan_mask_deg"] = list(scan_mask)
    if not block[-1].endswith("\n"):
        block[-1] += "\n"
    return "".join(block), want


def with_rl(text: str, policy: str, rl_src: Path, scan_mask=None) -> str:
    """실물 사본에 RL controller 둘을 더한다. 다른 줄은 그대로 둔다."""
    blocks, want_blocks = _rl_blocks(rl_src, policy, scan_mask)
    base = yaml.safe_load(text)
    plugins = base[CTL[0]][CTL[1]]["controller_plugins"]
    if "FollowPathMPPI" in plugins or any(name in base[CTL[0]][CTL[1]] for name in RL_NAMES):
        raise RuntimeError(f"real controller list already has MPPI or RL: {plugins}")
    lines = text.splitlines(keepends=True)
    at = [i for i, line in enumerate(lines) if line.startswith("    controller_plugins:")]
    if len(at) != 1 or not lines[at[0]].rstrip().endswith("]"):
        raise RuntimeError("expected one single-line controller_plugins under controller_server")
    shown = json.dumps(plugins + list(RL_NAMES))
    note = f"    # make_params.py --rl-policy {policy}: {RL_REL.as_posix()}의 두 블록(policy만 바꿈)\n"
    lines[at[0]] = f"    controller_plugins: {shown}  # make_params.py --rl-policy\n"
    out = "".join(lines[:at[0] + 1]) + note + blocks + "".join(lines[at[0] + 1:])
    want = copy.deepcopy(base)
    want[CTL[0]][CTL[1]]["controller_plugins"] = plugins + list(RL_NAMES)
    want[CTL[0]][CTL[1]].update(want_blocks)
    if yaml.safe_load(out) != want:
        raise RuntimeError("the RL copy differs from the real copy plus the two sim blocks")
    return out


def write_params(speed, out_dir, nav2_src=None, avoid_src=None, fixed_line_check=False,
                 rl_policy=None, rl_src=None, policies_dir=None, rl_scan_mask=None) -> tuple:
    """두 사본을 out_dir에 쓴다. 확인을 모두 통과해야 쓴다."""
    value = checked_speed(speed)
    # scan 가림은 RL 블록에만 들어가므로 정책 없이는 받지 않는다
    mask = None
    if rl_scan_mask is not None:
        if rl_policy is None:
            raise ValueError("--rl-scan-mask needs --rl-policy (it only goes into the RL blocks)")
        mask = checked_scan_mask(rl_scan_mask)
    nav2 = _converted(find_source(NAV2_REL, nav2_src), NAV2_TARGETS, value, COLLISION_TARGETS,
                      bool(fixed_line_check))
    if rl_policy is not None:
        nav2 = with_rl(nav2, checked_policy(rl_policy, policies_dir), find_source(RL_REL, rl_src),
                       mask)
    avoid = _converted(find_source(AVOID_REL, avoid_src), AVOID_TARGETS, value)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / NAV2_NAME).write_text(nav2, encoding="utf-8")
    (out / AVOID_NAME).write_text(avoid, encoding="utf-8")
    return out / NAV2_NAME, out / AVOID_NAME


# ---- launch가 쓰는 도우미 ----
def speed_of(params_file) -> float:
    """만든 nav2_params.yaml의 앞으로 가는 속도(FollowPath, FollowPathThrough는 같아야 한다)."""
    path = Path(params_file)
    if not path.is_file():
        raise FileNotFoundError(f"{path} does not exist: run make_params.py --speed S "
                                f"--out {path.parent} (start.sh does it)")
    ctl = yaml.safe_load(path.read_text(encoding="utf-8"))["controller_server"]["ros__parameters"]
    speeds = {float(ctl[name]["desired_linear_vel"]) for name in FORWARD}
    if len(speeds) != 1:
        raise ValueError(f"{path}: FollowPath and FollowPathThrough speeds differ: {speeds}")
    return speeds.pop()


def horizon_cap(given: str, params_file) -> str:
    """pode_fsm horizon_cap_mps. 비면 컨트롤러 속도, 주면 그 속도 이하여야 한다."""
    speed = speed_of(params_file)
    if not str(given).strip():
        return _text(speed)
    try:
        cap = float(given)
    except ValueError:
        raise ValueError(f"horizon_cap_mps must be a number, got {given!r}") from None
    if not math.isfinite(cap) or not 0.0 < cap <= speed + 1e-9:
        raise ValueError(f"horizon_cap_mps must be above 0 and at most the controller "
                         f"speed {speed} in {params_file}, got {given!r}")
    return _text(cap)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--speed", required=True, help=f"m/s, 0 초과 {MAX_SPEED} 이하")
    parser.add_argument("--out", required=True, type=Path, help="사본을 쓸 폴더")
    parser.add_argument("--nav2-src", help="원본 nav2_params.yaml(기본: 저장소 또는 로봇 src)")
    parser.add_argument("--avoidance-src", help="원본 avoidance.yaml(기본: 같은 곳)")
    parser.add_argument("--fixed-line-check", action="store_true",
                        help="고정 경로 충돌 검사를 켜 둔다(기본: 끔)")
    parser.add_argument("--rl-policy", metavar="NAME",
                        help="RL controller 둘(FollowPathRL, FollowPathRLAlone)을 이 정책으로 더한다"
                             "(기본: 더하지 않음). 쓰는 controller는 그대로 FollowPathGoal")
    parser.add_argument("--rl-scan-mask", metavar="LO,HI,...",
                        help="RL 블록 둘의 scan_mask_deg 가림 각(도, base 기준, "
                             "-180 <= lo < hi <= 180 쌍). --rl-policy가 있어야 한다"
                             "(기본: 없음)")
    # 가림 각 값은 -로 시작할 수 있어 argparse가 옵션으로 읽지 않게 "옵션=값" 한 낱말로 붙인다.
    # --로 시작하는 다음 낱말은 값이 아니라 옵션이므로 붙이지 않는다(argparse가 값 없음으로 거절)
    given, argv = list(sys.argv[1:] if argv is None else argv), []
    while given:
        word = given.pop(0)
        if word == "--rl-scan-mask" and given and not given[0].startswith("--"):
            word = f"--rl-scan-mask={given.pop(0)}"
        argv.append(word)
    args = parser.parse_args(argv)
    try:
        nav2, avoid = write_params(args.speed, args.out, args.nav2_src, args.avoidance_src,
                                  args.fixed_line_check, args.rl_policy,
                                  rl_scan_mask=args.rl_scan_mask)
    except (ValueError, FileNotFoundError, RuntimeError) as problem:
        print(f"make_params: {problem}", file=sys.stderr)
        return 2
    print(f"make_params: speed {_text(checked_speed(args.speed))} m/s -> {nav2}, {avoid}")
    print(f"make_params: fixed-line check {'on' if args.fixed_line_check else 'off'}")
    if args.rl_policy is not None:
        print(f"make_params: rl controllers {', '.join(RL_NAMES)} with policy {args.rl_policy}")
        # RL 블록은 --speed를 따르지 않는다(max_linear_vel 0 = 정책 v_max)
        if checked_speed(args.speed) < MAX_SPEED:
            print(f"make_params: WARNING rl controllers ignore --speed and drive up to the "
                  f"policy v_max (at most {MAX_SPEED} m/s)")
    if args.rl_scan_mask is not None:
        mask = checked_scan_mask(args.rl_scan_mask)
        print(f"make_params: rl scan mask {len(mask) // 2} ranges: {_mask_text(mask)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

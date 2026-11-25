"""학습 실행의 설정 스냅샷(run_config.json)을 만든다 (Isaac 불필요).

학습을 시작할 때 실행 폴더에 한 번 쓴다: 실제 --set 목록, TaskSpec·ArenaSpec 전체 값, agent 설정,
git 커밋, 부모 체크포인트, 지도 yaml 의 sha256. 정책 계보(POLICIES.md)와 policy_spec.json 의
source 가 이 파일을 근거로 삼는다.
"""

from __future__ import annotations

import dataclasses
import datetime
import hashlib
import json
import math
import os
import subprocess

FORMAT = 1
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
MAP_DIR = os.path.join("pc", "fleet_core", "fleet_core", "data")
# --symmetry: off 는 symmetry_cfg 없음 (기존 실행과 같음), aug 는 거울 사본 추가, loss 는 거울 손실
SYMMETRY_MODES = ("off", "aug", "loss", "both")
# 거울 손실 = 행동 평균의 MSE (행동 단위), value_loss_coef 1.0 과 같은 크기로 더함
MIRROR_LOSS_COEFF = 1.0
# --caps_*: 계수가 하나라도 0 보다 크면 이 PPO 하위 클래스 (goal_reach/caps_ppo.py) 로 학습
CAPS_CLASS_NAME = "caps_ppo:CapsPPO"


def _git(*args: str) -> str:
    try:
        out = subprocess.run(("git", "-C", REPO, *args), capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.SubprocessError):
        return ""
    return out.stdout.strip() if out.returncode == 0 else ""


def git_info() -> dict:
    """커밋, 브랜치, 작업본에 커밋 안 된 변경이 있는지. git 이 없으면 빈 값."""
    return {
        "commit": _git("rev-parse", "HEAD"),
        "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(_git("status", "--porcelain", "--untracked-files=no")),
    }


def map_info(layout: str) -> dict:
    """지도 이름(v7)이나 yaml 경로를 경로와 sha256 으로. 이름과 경로는 지도 로더(load_fleet_map)와
    같은 규칙으로 가른다: 확장자가 없으면 이름. 저장소 안이면 저장소 기준 경로, 밖이면 절대 경로."""
    named = os.path.join(REPO, MAP_DIR, f"fleet_map_{layout}.yaml")
    path = layout if os.path.splitext(layout)[1] else named
    if not os.path.isfile(path):
        return {"layout": layout, "yaml": "", "yaml_sha256": ""}
    full = os.path.abspath(path)
    try:
        rel = os.path.relpath(full, REPO)
    except ValueError:  # Windows 에서 다른 드라이브
        rel = ".."
    shown = full if rel.startswith("..") else rel
    return {"layout": layout, "yaml": shown.replace(os.sep, "/"), "yaml_sha256": sha256_of(path)}


def build(
    *,
    argv,
    sets,
    num_envs,
    max_iterations,
    seed,
    init,
    resume,
    init_std,
    task_spec,
    arena_spec,
    agent,
    symmetry: str = "off",
    mirror_loss_coeff: float = MIRROR_LOSS_COEFF,
    caps_temporal: float = 0.0,
    caps_spatial: float = 0.0,
    caps_sigma: float = 0.0,
) -> dict:
    """run_config.json 내용. task_spec, arena_spec 은 dataclass, agent 는 dict."""
    return {
        "format": FORMAT,
        "started": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "argv": list(argv),
        "set": list(sets),
        "num_envs": int(num_envs),
        "max_iterations": int(max_iterations),
        "seed": int(seed),
        "init": init,
        "resume": resume,
        "init_std": list(parse_init_std(init_std)),
        "symmetry": {"mode": symmetry, **(symmetry_settings(symmetry, mirror_loss_coeff) or {})},
        "caps": caps_record(caps_temporal, caps_spatial, caps_sigma),
        "task_spec": dataclasses.asdict(task_spec),
        "arena_spec": dataclasses.asdict(arena_spec),
        "agent": agent,
        "map": map_info(arena_spec.layout),
        "git": git_info(),
    }


def parse_init_std(text: str) -> tuple[float, float]:
    """--init_std 값 "s" 또는 "sv,sw" 를 (v, w) 행동 std 로. 0 은 체크포인트 값을 그대로 둔다."""
    parts = [float(x) for x in str(text).split(",")] if str(text).strip() else [0.0]
    if len(parts) == 1:
        parts = parts * 2
    if len(parts) != 2 or not all(math.isfinite(x) and x >= 0 for x in parts):
        raise ValueError(f"--init_std takes one value or v,w (each finite and >= 0), got {text!r}")
    return parts[0], parts[1]


def symmetry_settings(mode: str, mirror_loss_coeff: float = MIRROR_LOSS_COEFF) -> dict | None:
    """--symmetry 값을 RslRlSymmetryCfg 인자로. off 는 None (symmetry_cfg 를 두지 않음)."""
    if mode not in SYMMETRY_MODES:
        raise ValueError(f"--symmetry takes one of {SYMMETRY_MODES}, got {mode!r}")
    if mode == "off":
        return None
    loss = mode in ("loss", "both")
    if loss and not mirror_loss_coeff > 0:
        raise ValueError(f"--symmetry {mode} needs --mirror_loss_coeff above 0")
    return {
        "use_data_augmentation": mode in ("aug", "both"),
        "use_mirror_loss": loss,
        "mirror_loss_coeff": float(mirror_loss_coeff) if loss else 0.0,
    }


def caps_settings(temporal: float, spatial: float, sigma: float) -> dict | None:
    """--caps_* 값을 algorithm 설정 항목으로. 모두 0 이면 None (rsl_rl PPO 그대로, 추가 난수 없음)."""
    values = (temporal, spatial, sigma)
    if not all(math.isfinite(x) and x >= 0 for x in values):
        raise ValueError(f"--caps_temporal/--caps_spatial/--caps_sigma must be finite and >= 0, got {values}")
    if spatial > 0 and not sigma > 0:
        raise ValueError("--caps_spatial needs --caps_sigma above 0")
    if sigma > 0 and not spatial > 0:
        raise ValueError("--caps_sigma only acts with --caps_spatial above 0")
    if not (temporal > 0 or spatial > 0):
        return None
    return {
        "class_name": CAPS_CLASS_NAME,
        "caps_temporal": float(temporal),
        "caps_spatial": float(spatial),
        "caps_sigma": float(sigma),
    }


def caps_record(temporal: float, spatial: float, sigma: float) -> dict:
    """run_config.json / run.json 의 caps 항목 (꺼져 있어도 0 값과 enabled=False 로 남김)."""
    on = caps_settings(temporal, spatial, sigma) is not None
    return {"enabled": on, "temporal": float(temporal), "spatial": float(spatial), "sigma": float(sigma)}


def sha256_of(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def write(path: str, config: dict) -> str:
    """파일로 쓰고 그 파일의 sha256 을 돌려준다."""
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(config, f, indent=1, ensure_ascii=False, default=str)
        f.write("\n")
    return sha256_of(path)

"""pode_rl_controller 의 가린 섹터 채우기 (scan_mask_fill, C++) 가 Python 과 같은 섹터 값을 내는지
견줄 파일을 쓴다: 경우마다 scan 격자, 가림, 빔 거리, 채우기 전과 뒤의 섹터 값 (m), 섹터마다
가림에 걸렸는지 (touched) 와 쓸 수 있는 레이가 있는지 (usable).

    python make_mask_fill_cases.py <out.json>

두 종류를 담는다.
- grid360: 빔 360 개, angle_min -pi, 간격 1 deg, lidar yaw 0 (레이 k 와 빔 k 가 일대일).
  기대값은 task_spec 의 own_body_rays, lidar_sectors (lidar_mask_fill=1) 로 낸다.
- grid720: 빔 720 개, lidar yaw 180 deg, 정수 도에서 어긋난 격자. 기대값은
  docs/rl_tracks_2026-11-26/real_scan_mask_check.py 의 sector_values, fill_sectors 로 낸다.
가림 경계에 너무 가까운 빔이 있으면 두 구현이 그 빔을 다르게 볼 수 있어 멈춘다.
JSON 에 inf/nan 이 없어 돌아오지 않은 빔은 YAML 문자열 (".inf", ".nan") 로 쓴다.
"""

import argparse
import importlib.util
import json
import math
import os
import random
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import task_spec as ts  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(HERE))
CHECK_PY = os.path.join(ROOT, "docs", "rl_tracks_2026-11-26", "real_scan_mask_check.py")
SPEC_JSON = os.path.join(
    ROOT, "pode", "pode_rl_controller", "policies", "store_arena_4b", "policy_spec.json"
)
SEED = 20261127
RANGE_MIN, RANGE_MAX = 0.05, 12.0
EDGE_MARGIN = 1e-6  # deg; 빔 각과 가림 경계의 최소 차이
# 반 도 경계: 정수 도 경계는 반올림에 따라 두 구현이 경계 빔 하나를 다르게 볼 수 있다
MASKS_360 = (
    "137.5,156.5,-152.5,-137.5",
    "-179.5,-140.5",
    "170.5,180,-180,-175.5",
)
MASKS_720 = ("138,157,-152.5,-137", "-60,-31")
GRID_720 = {"angle_min": -3.1241, "angle_increment": 2.0 * math.pi / 719, "yaw": math.pi}
CASES_PER_MASK = 4


def enc(x: float):
    """make_closing_cases.enc 와 같다 (-inf 는 넣지 않는다)."""
    if math.isnan(x):
        return ".nan"
    if math.isinf(x):
        return ".inf"
    return x


def load_check():
    """real_scan_mask_check.py 를 경로로 불러온다."""
    spec = importlib.util.spec_from_file_location("real_scan_mask_check", CHECK_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---- 시험용 빔 거리 ----
def beam_value(rng: random.Random, dark: bool, odd: bool) -> float:
    """빔 하나의 거리. dark 이면 반환 없음 (inf 나 nan), odd 이면 range 밖 값도 섞는다."""
    if dark:
        return math.inf if rng.random() < 0.6 else math.nan
    u = rng.random()
    if odd and u < 0.05:
        return 0.01  # range_min 아래
    if odd and u < 0.10:
        return 15.0  # range_max 위
    if u < 0.50:
        return round(rng.uniform(0.08, 1.95), 4)
    if u < 0.75:
        return round(rng.uniform(2.05, 8.0), 4)  # 관측 상한보다 먼 유효 반환
    return math.inf if u < 0.90 else math.nan


def make_ranges(rng: random.Random, n: int, block: int, odd: bool) -> list[float]:
    """block 빔마다 하나의 뭉치로 보아, 뭉치의 35 % 는 반환이 하나도 없게 만든다."""
    out = []
    for b in range(0, n, block):
        dark = rng.random() < 0.35
        out += [beam_value(rng, dark, odd) for _ in range(min(block, n - b))]
    return out


def edge_check(name, deg, mask):
    """가림 경계에 EDGE_MARGIN 보다 가까운 빔이 있으면 멈춘다. ±180 경계는 MASKS_360 처럼
    반대쪽 범위와 짝을 지어 쓰므로 보지 않는다."""
    for lo, hi in mask:
        for edge in (lo, hi):
            if abs(edge) != 180.0 and float(np.abs(deg - edge).min()) < EDGE_MARGIN:
                raise SystemExit(f"{name}: a beam is within {EDGE_MARGIN} deg of edge {edge}")


# ---- 기대값 ----
def case_360(chk, rng, mask_text, i):
    """빔 360 개 정수 도 격자: task_spec 으로 기대값을 낸다."""
    pairs = chk.parse_mask(mask_text)
    n, amin, inc = 360, -math.pi, 2.0 * math.pi / 360
    edge_check(f"grid360 {mask_text}", chk.beam_deg(n, amin, inc, 0.0), pairs)
    ranges = make_ranges(rng, n, 10, odd=False)
    plain = ts.make_spec(ts.parse_overrides([f"lidar_mask_deg={mask_text}"]))
    fill = ts.make_spec(ts.parse_overrides([f"lidar_mask_deg={mask_text}", "lidar_mask_fill=1"]))
    r = torch.tensor([ranges], dtype=torch.float32)
    before = ts.lidar_sectors(ts.own_body_rays(r, plain), plain)[0]
    marked = ts.own_body_rays(r, fill)
    after = ts.lidar_sectors(marked, fill)[0]
    touched = ts.masked_sectors(fill, n)
    usable = torch.isfinite(marked[0]).view(fill.lidar_sectors, -1).any(dim=-1)
    return {
        "name": f"grid360_{i}",
        "angle_min": amin,
        "angle_increment": inc,
        "yaw": 0.0,
        "mask": [float(x) for x in mask_text.split(",")],
        "ranges": ranges,
        "before": before.tolist(),
        "after": after.tolist(),
        "touched": touched.tolist(),
        "usable": usable.tolist(),
    }


def case_720(chk, lid, rng, mask_text, i):
    """빔 720 개, yaw 180 deg, 어긋난 격자: real_scan_mask_check 로 기대값을 낸다."""
    pairs = chk.parse_mask(mask_text)
    g, n = GRID_720, 720
    deg = chk.beam_deg(n, g["angle_min"], g["angle_increment"], g["yaw"])
    edge_check(f"grid720 {mask_text}", deg, pairs)
    ranges = make_ranges(rng, n, 20, odd=True)
    r32 = np.asarray(ranges, dtype=np.float32)
    out = chk.sector_values(
        r32, n, g["angle_min"], g["angle_increment"], RANGE_MIN, RANGE_MAX, g["yaw"], lid, pairs
    )
    after, _ = chk.fill_sectors(out["masked"], out["usable"], out["touched"], lid["obs_max"])
    return {
        "name": f"grid720_{i}",
        "angle_min": g["angle_min"],
        "angle_increment": g["angle_increment"],
        "yaw": g["yaw"],
        "mask": [float(x) for x in mask_text.split(",")],
        "ranges": [float(x) for x in r32],
        "before": [float(x) for x in out["masked"]],
        "after": [float(x) for x in after],
        "touched": [bool(x) for x in out["touched"]],
        "usable": [bool(x) for x in out["usable"]],
    }


def summary(c):
    """채워진 섹터 수와 값이 바뀐 섹터 수."""
    empty = [t and not u for t, u in zip(c["touched"], c["usable"], strict=True)]
    changed = sum(a != b for a, b in zip(c["before"], c["after"], strict=True))
    return f"{c['name']}: touched {sum(c['touched'])}, empty {sum(empty)}, changed {changed}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    args = ap.parse_args()
    chk = load_check()
    with open(SPEC_JSON, encoding="utf-8") as f:
        lid = json.load(f)["lidar"]
    if (lid["sectors"], lid["obs_max"]) != (ts.SPEC.lidar_sectors, ts.SPEC.lidar_obs_max):
        raise SystemExit(f"policy lidar {lid} differs from task_spec")
    if chk.FILL_REACH != ts.FILL_REACH:
        raise SystemExit("FILL_REACH differs between task_spec and real_scan_mask_check")
    rng = random.Random(SEED)
    cases = []
    for mask_text in MASKS_360:
        for _ in range(CASES_PER_MASK):
            cases.append(case_360(chk, rng, mask_text, len(cases)))
    for mask_text in MASKS_720:
        for _ in range(CASES_PER_MASK):
            cases.append(case_720(chk, lid, rng, mask_text, len(cases)))
    for c in cases:
        c["ranges"] = [enc(x) for x in c["ranges"]]
        print(f"[mask_fill] {summary(c)}")
    data = {
        "range_min": RANGE_MIN,
        "range_max": RANGE_MAX,
        "obs_max": lid["obs_max"],
        "fill_reach": ts.FILL_REACH,
        "cases": cases,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(data, separators=(",", ":")) + "\n")
    print(f"[mask_fill] wrote {args.out} ({os.path.getsize(args.out)} bytes)")


if __name__ == "__main__":
    main()

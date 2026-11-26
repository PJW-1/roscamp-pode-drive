"""시간을 넘긴 에피소드의 끝 동작을 센다 (작업 기록 20절).

평가(play.py)가 쓴 rollouts.npz에서 추적 env의 첫 에피소드 가운데 시간을 넘긴 것을 고르고, 끝 3 s의 움직임을
learning/store_arena/timeouts.py의 motion으로 stopped, dithering, moving 셋으로 나눈다. 정면 거리는 motion에 주지 않는다:
주면 앞이 가까운 에피소드가 돌고 있었는지와 무관하게 facing_wall 한 갈래로 묶인다. 앞이 가까웠는지는 따로 센다.

두 번째 폴더(같은 정책의 기존 평가)를 주면 결과 비율과 추적 env 안의 시간 초과 수가 같은지도 본다.

사용: python timeout_motion.py <실행 폴더> [<같은 정책의 기존 평가 폴더>]
"""
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "learning" / "store_arena"))
import timeouts  # noqa: E402

CLASSES = ("stopped", "dithering", "moving")
TAIL_S = 3.0  # 끝 동작을 보는 구간
FRONT_M = 0.2  # 이보다 가까우면 앞이 막힌 것으로 센다
MIN_N = 20  # 이보다 적으면 판정하지 않는다
TIMEOUT = 3  # play.py의 결과 순서 (success, collision, oob, timeout)


def box_distance(xy, centers, sizes) -> float:
    """점에서 가장 가까운 상자 변까지의 거리. 상자 안이면 0, 쓰는 상자가 없으면 NaN."""
    best = float("nan")
    for c, side in zip(centers, sizes, strict=True):
        if np.isnan(c[0]):
            continue
        d = np.maximum(np.abs(np.asarray(xy) - c) - 0.5 * side, 0.0)
        best = float(np.nanmin([best, np.hypot(d[0], d[1])]))
    return best


def rows(run) -> list:
    """추적 env의 첫 에피소드가 시간 초과면 한 줄: 끝 3 s의 동작, 앞이 가까웠는지, 끝 위치에서 상자까지의 거리."""
    with np.load(pathlib.Path(run) / "rollouts.npz", allow_pickle=True) as f:
        z = {k: f[k] for k in f.files}
    tail = max(int(round(TAIL_S / float(z["dt"]))), 2)
    out = []
    for e in range(z["xy"].shape[1]):
        for s, t in timeouts.episodes(z["done"][:, e])[:1]:
            if z["outcome"][t, e] != TIMEOUT:
                continue
            # 끝난 스텝 t의 위치는 이미 다음 에피소드의 리셋 자세라서 뺀다. 경로 번호는 스텝 전의 것이 기록되므로
            # t에서도 이 에피소드의 번호다
            b = max(s, t - tail)
            us = z["xy"][b:t, e]
            route = int(z["route"][t, e])
            out.append({
                "env": e,
                "motion": timeouts.motion(us, None, z["cmd"][b:t, e]),
                "front_near": bool(z["front_m"][b:t, e].min() < FRONT_M),
                "box_m": box_distance(us[-1], z["obstacle_xy"][route], z["obstacle_sizes"]),
                "net_m": float(np.linalg.norm(us[-1] - us[0])),
            })
    return out


def verdict(counts: dict) -> str:
    """갈래 하나가 절반을 넘으면 그 갈래. 시간 초과가 MIN_N보다 적으면 판정하지 않는다."""
    n = sum(counts.values())
    if n < MIN_N:
        return f"not judged (fewer than {MIN_N} tracked timeouts)"
    for c in CLASSES:
        if counts[c] > n / 2:
            return f"mostly {c}"
    return "no majority"


def check(run, base) -> dict:
    """같은 정책의 기존 평가와 결과 비율이 같은지, 추적 env 안의 시간 초과 수가 얼마씩인지."""
    def load(d):
        d = pathlib.Path(d)
        rates = json.loads((d / "eval.json").read_text(encoding="utf-8"))["rates"]
        with np.load(d / "episodes.npz", allow_pickle=True) as f:
            keys = [str(k) for k in f["keys"]]
            return rates, f["env"], f["code"] == keys.index("timeout")
    with np.load(pathlib.Path(run) / "rollouts.npz", allow_pickle=True) as f:
        tracked = f["xy"].shape[1]
    (ra, ea, ta), (rb, eb, tb) = load(run), load(base)
    return {"rates_equal": ra == rb,
            "timeouts_in_tracked_envs": [int((ta & (ea < tracked)).sum()), int((tb & (eb < tracked)).sum())]}


def main():
    run = sys.argv[1]
    rs = rows(run)
    counts = {c: sum(r["motion"] == c for r in rs) for c in CLASSES}
    n = len(rs)
    print(run)
    print(f"  tracked timeouts {n}: " + ", ".join(f"{c} {counts[c]}" for c in CLASSES) + f"  -> {verdict(counts)}")
    for c in CLASSES:
        sel = [r for r in rs if r["motion"] == c]
        if not sel:
            continue
        box = np.array([r["box_m"] for r in sel])
        print(f"  {c:9s} front nearer than {FRONT_M} m: {sum(r['front_near'] for r in sel)} of {len(sel)},"
              f" box edge distance median {np.nanmedian(box):.3f} m, within 0.15 m: {int((box < 0.15).sum())},"
              f" net move in the last {TAIL_S:.0f} s median {np.median([r['net_m'] for r in sel]):.3f} m")
    if len(sys.argv) > 2:
        print("  against", sys.argv[2], json.dumps(check(run, sys.argv[2])))


if __name__ == "__main__":
    main()

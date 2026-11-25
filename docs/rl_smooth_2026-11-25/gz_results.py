"""출력 평활 Gazebo 표: acd3+shim 평활 없음, ema_alpha_w 0.5, 0.25와 RPP, MPPI의 칸 값과 쌍별 차이.
  python3 gz_results.py <gz_run.sh OUT> [dest]   (WSL, ROS install을 source한 셸)
구간 값(성공, 접촉, 시간, 분당 w 부호 반전, 몸 최소 간격)과 통계는 rl_acd_2026-11-24/gz_results.py를 쓰고,
명령 w의 RMS 각가속도와 RMS 각저크를 더한다.
"""
import importlib.util
import json
import math
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DT = 0.05  # controller_frequency 20 Hz
GAP = 1.5  # 이웃 명령 간격이 이 배수 * DT를 넘으면 계열을 끊는다

NAMES = {"FollowPathRLShimACD3": "acd3 raw", "FollowPathRLShimACD3W50": "acd3 w0.5",
         "FollowPathRLShimACD3W25": "acd3 w0.25", "FollowPathGoal": "RPP",
         "FollowPathMPPI": "MPPI"}
# 비교 쌍 (기준, 후보): 차이는 후보 - 기준
PAIRS = (("FollowPathRLShimACD3", "FollowPathRLShimACD3W50"),
         ("FollowPathRLShimACD3", "FollowPathRLShimACD3W25"),
         ("FollowPathGoal", "FollowPathRLShimACD3"),
         ("FollowPathGoal", "FollowPathRLShimACD3W50"),
         ("FollowPathGoal", "FollowPathRLShimACD3W25"))


# ---- 명령 w의 미분: 20 Hz 계열로 보고 간격이 벌어진 곳에서 끊는다 ----
def w_rates(cmd, dt=DT, gap=GAP):
    """cmd [(t, v, w), ...] -> (RMS 각가속도 rad/s^2, RMS 각저크 rad/s^3). 차분이 없으면 None."""
    segments, cur = [], []
    for c in cmd:
        if cur and c[0] - cur[-1][0] > gap * dt:
            segments.append(cur)
            cur = []
        cur.append(c)
    if cur:
        segments.append(cur)
    acc_sq, jerk_sq = [], []
    for seg in segments:
        acc = [(b[2] - a[2]) / dt for a, b in zip(seg, seg[1:])]
        acc_sq += [a * a for a in acc]
        jerk_sq += [((y - x) / dt) ** 2 for x, y in zip(acc, acc[1:])]
    rms = (lambda sq: math.sqrt(statistics.fmean(sq)) if sq else None)
    return rms(acc_sq), rms(jerk_sq)


def _base():
    spec = importlib.util.spec_from_file_location(
        "gz_results_acd2", HERE.parent / "rl_acd_2026-11-24" / "gz_results.py")
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    return g


# ---- 구간 표: 기준 스크립트의 행에 각가속도·각저크를 붙인다 ----
def legs(g, out):
    rows = g.legs(out)
    rates = {}
    for batch, _ in g.BATCHES:
        for p in sorted((out / batch / "legs").glob("*.json")):
            rec = json.loads(p.read_text())
            sc = "q2e3" if batch == "q2e3" else rec["scenario"]
            cmd = g.mt.within(rec["cmd"], rec["t_start"], rec["t_end"])
            dts = sorted(b[0] - a[0] for a, b in zip(cmd, cmd[1:]))
            acc, jerk = w_rates(cmd)
            rates[(sc, rec["controller"], rec["repeat"])] = {
                "w_acc_rms": None if acc is None else round(acc, 3),
                "w_jerk_rms": None if jerk is None else round(jerk, 2),
                "cmd_dt_p95_ms": round(1000 * dts[int(0.95 * (len(dts) - 1))], 1) if dts else None}
    return [{**r, **rates[(r["scenario"], r["controller"], r["repeat"])]} for r in rows]


def sd(v):
    v = [x for x in v if x is not None]
    return round(statistics.stdev(v), 3) if len(v) > 1 else None


def summarize(g, rows):
    cells, comps = [], []
    for sc in sorted({r["scenario"] for r in rows}):
        by = {}
        for ctl in NAMES:
            grp = [r for r in rows if r["scenario"] == sc and r["controller"] == ctl]
            if not grp:
                continue
            by[ctl] = grp
            ok = [r for r in grp if r["ok"]]
            mins = [r["body_min_m"] for r in grp if r["body_min_m"] is not None]
            cells.append({
                "scenario": sc, "name": NAMES[ctl], "runs": len(grp), "successes": len(ok),
                "collision_runs": sum(1 for r in grp
                                      if r["wall_obstacle_contacts"] + r["robot_contacts"] > 0),
                "timeouts": sum(1 for r in grp if r["status"] == "TIMEOUT"),
                "mean_time_s": g.mean([r["time_s"] for r in ok]),
                "sd_time_s": sd([r["time_s"] for r in ok]),
                "flips_per_min": g.mean([r["flips_per_min"] for r in grp]),
                "sd_flips_per_min": sd([r["flips_per_min"] for r in grp]),
                "w_acc_rms": g.mean([r["w_acc_rms"] for r in grp]),
                "w_jerk_rms": g.mean([r["w_jerk_rms"] for r in grp]),
                "min_clearance_m": min(mins) if mins else None,
                "median_leg_clearance_m": round(statistics.median(mins), 4) if mins else None,
                "cmd_dt_p95_ms": max((r["cmd_dt_p95_ms"] for r in grp
                                      if r["cmd_dt_p95_ms"] is not None), default=None)})
        for base, cand in PAIRS:
            if base not in by or cand not in by:
                continue
            a, b = by[base], by[cand]
            pair = f"{NAMES[cand]} - {NAMES[base]}"
            ka, kb = sum(r["ok"] for r in a), sum(r["ok"] for r in b)
            pa, pb = ka / len(a), kb / len(b)
            se = math.sqrt(pa * (1 - pa) / len(a) + pb * (1 - pb) / len(b))
            comps.append({"pair": pair, "scenario": sc, "metric": "success_pp",
                          "diff": round(100 * (pb - pa), 1), "se": round(100 * se, 1),
                          "fisher_p": round(g.fisher_p(kb, len(b), ka, len(a)), 3)})
            for metric, key, sel in (("time_s (successful)", "time_s", True),
                                     ("flips_per_min", "flips_per_min", False),
                                     ("w_acc_rms", "w_acc_rms", False),
                                     ("w_jerk_rms", "w_jerk_rms", False),
                                     ("leg_min_clearance_m", "body_min_m", False)):
                xa = [r[key] for r in a if r["ok"] or not sel]
                xb = [r[key] for r in b if r["ok"] or not sel]
                d, s = g.welch(xb, xa)
                comps.append({"pair": pair, "scenario": sc, "metric": metric, "diff": d, "se": s,
                              "fisher_p": ""})
    return cells, comps


def main():
    g = _base()
    out = Path(sys.argv[1])
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE
    rows = legs(g, out)
    cells, comps = summarize(g, rows)
    g.write(dest / "gz_legs.csv", rows)
    g.write(dest / "gz_cells.csv", cells)
    g.write(dest / "gz_compare.csv", comps)
    print(g.table(cells))
    print()
    print(g.table(comps))


if __name__ == "__main__":
    main()

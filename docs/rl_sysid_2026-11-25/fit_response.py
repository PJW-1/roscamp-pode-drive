#!/usr/bin/env python3
"""계단 응답에 지연 + 1차 지연 모델을 맞추고, 왼쪽·오른쪽 차이와 Isaac 계단 값을 낸다.

사용: python docs/rl_sysid_2026-11-25/fit_response.py  (worktree 루트에서)
"""
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "real_pode"))
import step_report as sr  # noqa: E402

WHEEL_SEPARATION = 0.0961  # m, learning/goal_reach/task_spec.py RobotGeometry
FIT_WINDOW_S = 1.0


# ---- 지연 d, 시간 상수 tau 격자 맞춤: y = A (1 - exp(-(t - d) / tau)), t > d ----
def fit_delay_lag(t, y, a):
    best = (math.inf, math.nan, math.nan)
    for d in np.arange(0.0, 0.1505, 0.0025):
        for tau in np.arange(0.0025, 0.2005, 0.0025):
            m = np.where(t > d, a * (1.0 - np.exp(-(t - d) / tau)), 0.0)
            sse = float(np.sum((y - m) ** 2))
            if sse < best[0]:
                best = (sse, d, tau)
    return best[1], best[2]


def odom_stamp_lag(paths):
    """odom 을 받은 시각 t_ros 와 헤더 시각 t_stamp 차이의 중앙값 (전달 지연)."""
    lags = []
    for p in paths:
        with open(p, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                if r["kind"] == "odom" and r["t_stamp"]:
                    lags.append(float(r["t_ros"]) - float(r["t_stamp"]))
    return float(np.median(lags)) if lags else math.nan


# ---- probe CSV 한 묶음 ----
def probe_table(name, paths):
    segs = sr.segments(sr.load_rows([str(p) for p in paths]))
    print(f"\n## {name}")
    print(f"odom 수신 - 헤더 시각 중앙값: {odom_stamp_lag(paths) * 1000:.1f} ms")
    fits = defaultdict(list)
    lr = defaultdict(list)
    counts = defaultdict(int)
    for s in segs:
        if s["set"] not in ("linear", "angular"):
            counts[(s["set"], s["label"])] += 1
            continue
        target, y = sr.response_of(s)
        sel = (s["t"] >= s["t0"]) & (s["t"] <= min(s["t1"], s["t0"] + FIT_WINDOW_S))
        d, tau = fit_delay_lag(s["t"][sel] - s["t0"], y[sel], target)
        fits[(s["set"], s["label"])].append((d, tau))
        if s["set"] == "linear":
            tail = (s["t"] >= s["t0"] + 0.5 * (s["t1"] - s["t0"])) & (s["t"] <= s["t1"])
            # (오른 - 왼) / 평균 바퀴 속도 = w b / v
            lr[s["label"]].append(float(np.mean(s["ow"][tail]) * WHEEL_SEPARATION / s["v"]))
    print("| set | label | n | 지연 d s (평균, 범위) | tau s (평균, 범위) |")
    print("|---|---|---|---|---|")
    for (st, lab), v in fits.items():
        ds, ts_ = np.array([x[0] for x in v]), np.array([x[1] for x in v])
        print(f"| {st} | {lab} | {len(v)} | {ds.mean():.3f} ({ds.min():.3f}-{ds.max():.3f}) "
              f"| {ts_.mean():.3f} ({ts_.min():.3f}-{ts_.max():.3f}) |")
    if lr:
        print("\n| linear label | (v_R - v_L) / v (평균, 범위) |")
        print("|---|---|")
        for lab, v in lr.items():
            a = np.array(v)
            print(f"| {lab} | {a.mean():+.4f} ({a.min():+.4f}..{a.max():+.4f}) |")
    if counts:
        print("\n| 기타 set | label | 끝난 계단 수 |")
        print("|---|---|---|")
        for (st, lab), n in counts.items():
            print(f"| {st} | {lab} | {n} |")


# ---- Isaac newton_pinky_check 50 Hz 기록: 단계 시작에서 바로 바뀐 바퀴 목표에 대한 응답 ----
def isaac_table(path):
    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    print(f"\n## Isaac {path.name}")
    print("| phase | 목표 | 10% s | 90% s | 상승 s | 정상 비 |")
    print("|---|---|---|---|---|---|")
    by = defaultdict(list)
    for r in rows:
        by[r["phase"]].append(r)
    for ph, key, cmd in (("straight_0.10", "vx_b", 0.10), ("spin_1.0", "wz_b", 1.0)):
        rr = by.get(ph, [])
        if not rr:
            continue
        t = np.array([float(r["t"]) for r in rr])
        y = np.array([float(r[key]) for r in rr])
        t0 = t[0] - 0.02
        t10 = sr.crossing(t, y, 0.1 * cmd, t0, t[-1])
        t90 = sr.crossing(t, y, 0.9 * cmd, t0, t[-1])
        tail = t >= t[-1] - 2.0
        print(f"| {ph} | {cmd} | {t10 - t0:.3f} | {t90 - t0:.3f} | {t90 - t10:.3f} "
              f"| {np.mean(y[tail]) / cmd:.3f} |")


# ---- 학습 모델(지연 k 스텝, 20 Hz 1차 지연 tau)에 Newton 바퀴 응답 tn 을 더한 계단 응답 ----
def model_step(k, tau, tn, dt=0.05, h=0.0005, horizon=1.0):
    t = np.arange(0.0, horizon, h)
    beta = math.exp(-dt / tau) if tau > 0 else 0.0
    n = np.floor(t / dt)
    u = np.where(n >= k, 1.0 - beta ** np.maximum(n - k + 1, 0), 0.0)
    y = np.zeros_like(t)
    a = 1.0 - math.exp(-h / tn)
    for i in range(1, t.size):
        y[i] = y[i - 1] + (u[i - 1] - y[i - 1]) * a
    t10 = t[np.argmax(y >= 0.1)]
    return t10, t[np.argmax(y >= 0.9)] - t10


def model_table():
    # Newton 바퀴 시간 상수: Isaac 계단의 상승 시간 / 2.2 (선 0.051 s, 회전 0.012 s)
    print("\n## 학습 모델 계단 응답 (변화율 제한 없음, 10% 시각 / 10→90% 상승 s)")
    print("| 축 | 지연 k | " + " | ".join(f"tau {x:.3f}" for x in (0.0, 0.04, 0.065, 0.08, 0.12))
          + " |")
    print("|---|---|---|---|---|---|---|")
    for name, tn in (("선", 0.023), ("회전", 0.005)):
        for k in (0, 1, 2):
            cells = [model_step(k, tau, tn) for tau in (0.0, 0.04, 0.065, 0.08, 0.12)]
            print(f"| {name} | {k} | " + " | ".join(f"{a:.3f} / {b:.3f}" for a, b in cells)
                  + " |")


def main():
    real = ROOT / "docs" / "rl_sysid_2026-11-25" / "real"
    gz = ROOT / "results" / "2026-11-21" / "step_probe_gz"
    probe_table("real pode2", sorted(real.glob("step_*_pode2.csv")))
    probe_table("Gazebo", [gz / "step_linear.csv", gz / "step_angular.csv",
                           gz / "step_small_rot.csv"])
    nk = ROOT / "results" / "2026-11-21" / "newton_pinky_check"
    isaac_table(nk / "step_env0_50hz.csv")
    isaac_table(nk / "kd0.5_step_env0_50hz.csv")
    model_table()


if __name__ == "__main__":
    main()

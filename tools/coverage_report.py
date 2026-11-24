#!/usr/bin/env python3
"""청소 커버리지 측정(계획 6-2, 6-4): bag의 자세로 줄 품질 지표와 커버율 지도를 만든다.

    python3 tools/coverage_report.py metrics BAG --ns pode3 [--layout v7 | --plan FILE]
        [--format md|json]
    python3 tools/coverage_report.py map BAG --ns pode3 --out coverage.png
    python3 tools/coverage_report.py relocalize BAG --ns pode3   (다음 단계)

자세는 시뮬 참값(/<ns>/sim/truth, 실제)과 AMCL(/<ns>/amcl_pose, 관제가 믿는 값) 두 가지다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc" / "fleet_core"))

# ---- 기본 파라미터 ----
GRID_M = 0.01
CLEAN_WIDTH_M = 0.112
END_PAD_M = 0.10
ALIGN_RAD = 0.1
LINE_HEADING_RAD = 0.5
SPIN_RADPS = 0.3
SPIN_MPS = 0.01
STILL_M = 0.001
SOURCES = ("truth", "amcl")
ROLE = {"truth": "actual (sim truth)", "amcl": "believed (AMCL)"}
_EPS = 1e-9


# ---- 자세 기록 ----
@dataclass
class Track:
    topic: str
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    def __len__(self) -> int:
        return len(self.t)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def default_topics(ns: str) -> dict[str, str]:
    return {"truth": f"/{ns}/sim/truth", "amcl": f"/{ns}/amcl_pose"}


def read_tracks(bag: Path, topics: dict[str, str]) -> dict[str, Track]:
    """bag에서 자세 토픽을 읽는다. 시각은 header stamp, 비었으면 녹화 시각."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage = "mcap" if str(bag).endswith(".mcap") else ""
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {topic: key for key, topic in topics.items() if topic in types}
    rows: dict[str, list] = {key: [] for key in wanted.values()}
    if wanted:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    while wanted and reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        msg = deserialize_message(raw, classes[topic])
        # Odometry, PoseWithCovarianceStamped는 pose.pose, PoseStamped는 pose
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9 if (st.sec or st.nanosec) else stamp_ns * 1e-9
        rows[wanted[topic]].append((t, pose.position.x, pose.position.y, _yaw(pose.orientation)))
    out = {}
    for key, data in rows.items():
        if data:
            # 같은 시각은 bag 순서를 지킨다
            arr = np.array(sorted(data, key=lambda r: r[0]))
            out[key] = Track(topics[key], arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3])
    return out


# ---- 계획 줄 ----
@dataclass
class Line:
    start: tuple[float, float]
    end: tuple[float, float]
    cell: int

    @property
    def length(self) -> float:
        return math.dist(self.start, self.end)

    @property
    def u(self) -> np.ndarray:
        return (np.array(self.end) - np.array(self.start)) / self.length

    @property
    def n(self) -> np.ndarray:
        u = self.u
        return np.array([-u[1], u[0]])

    @property
    def heading(self) -> float:
        return math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

    def along(self, x, y):
        """(줄 방향 거리, 왼쪽 옆 거리)."""
        dx, dy = x - self.start[0], y - self.start[1]
        u, n = self.u, self.n
        return dx * u[0] + dy * u[1], dx * n[0] + dy * n[1]


@dataclass
class Plan:
    source: str
    lines: list[Line]
    transits: list[list[tuple[float, float]]]
    line_spacing_m: float
    turn_radius_m: float


def load_plan(path: Path) -> Plan:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    params = data.get("params", {})
    lines = [Line(tuple(e["start"]), tuple(e["end"]), int(e.get("cell", 0))) for e in data["lines"]]
    transits = [[tuple(p) for p in e["points"]] for e in data.get("transits", [])]
    return Plan(str(path), lines, transits, float(params.get("line_spacing_m", 0.10)),
                float(params.get("turn_radius_m", 0.0)))


def plan_path(args) -> Path:
    if args.plan:
        return Path(args.plan)
    from fleet_core import coverage
    return coverage.data_path(args.layout)


# ---- 1 cm 격자와 쓸기 ----
class Grid:
    def __init__(self, xs: np.ndarray, ys: np.ndarray, pad: float, res: float = GRID_M):
        self.res = res
        self.x0 = math.floor((xs.min() - pad) / res) * res
        self.y0 = math.floor((ys.min() - pad) / res) * res
        self.w = math.ceil((xs.max() + pad - self.x0) / res) + 1
        self.h = math.ceil((ys.max() + pad - self.y0) / res) + 1
        self.xc = self.x0 + (np.arange(self.w) + 0.5) * res
        self.yc = self.y0 + (np.arange(self.h) + 0.5) * res

    @property
    def cell_m2(self) -> float:
        return self.res * self.res

    def extent(self):
        return (self.x0, self.x0 + self.w * self.res, self.y0, self.y0 + self.h * self.res)


def sweep(grid: Grid, pts: np.ndarray, half: float) -> np.ndarray:
    """꺾은선을 폭(반지름 half)으로 쓸어 칸마다 덮은 횟수를 센다.
    같은 칸을 다시 덮어도 그 사이 지나간 길이가 청소 폭 이하이면 한 번으로 본다."""
    visits = np.zeros((grid.h, grid.w), dtype=np.int32)
    last = np.full((grid.h, grid.w), -np.inf)
    if len(pts) == 0:
        return visits
    if len(pts) == 1:
        pts = np.vstack([pts, pts])
    s = 0.0
    for a, b in zip(pts[:-1], pts[1:], strict=True):
        seg = float(math.hypot(b[0] - a[0], b[1] - a[1]))
        # 선분 둘레 창만 계산한다
        i0 = max(0, int((min(a[0], b[0]) - half - grid.x0) / grid.res) - 1)
        i1 = min(grid.w, int((max(a[0], b[0]) + half - grid.x0) / grid.res) + 2)
        j0 = max(0, int((min(a[1], b[1]) - half - grid.y0) / grid.res) - 1)
        j1 = min(grid.h, int((max(a[1], b[1]) + half - grid.y0) / grid.res) + 2)
        xs, ys = np.meshgrid(grid.xc[i0:i1], grid.yc[j0:j1])
        ux, uy = b[0] - a[0], b[1] - a[1]
        span = ux * ux + uy * uy
        t = np.zeros(xs.shape) if span <= _EPS else np.clip(
            ((xs - a[0]) * ux + (ys - a[1]) * uy) / span, 0.0, 1.0)
        hit = np.hypot(xs - a[0] - t * ux, ys - a[1] - t * uy) <= half + _EPS
        # 칸에 가장 가까운 점의 누적 길이로 다시 지나감을 가린다
        at = s + t * seg
        win_last = last[j0:j1, i0:i1]
        visits[j0:j1, i0:i1] += hit & (at - win_last > 2.0 * half)
        win_last[hit] = at[hit]
        s += seg
    return visits


def target_mask(grid: Grid, plan: Plan, half: float) -> np.ndarray:
    """계획 줄과 (둥글게 바꾼) 이동 구간을 청소 폭으로 쓴 칸."""
    mask = np.zeros((grid.h, grid.w), dtype=bool)
    paths = [[ln.start, ln.end] for ln in plan.lines]
    if plan.transits:
        from fleet_core import coverage
        paths += [coverage.rounded(p, plan.turn_radius_m) for p in plan.transits]
    for p in paths:
        mask |= sweep(grid, np.array(p, dtype=float), half) > 0
    return mask


def blobs(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """8방향으로 이어진 칸 덩어리."""
    seen = np.zeros(mask.shape, dtype=bool)
    out = []
    for j, i in zip(*np.nonzero(mask), strict=True):
        if seen[j, i]:
            continue
        seen[j, i] = True
        queue, cells = deque([(j, i)]), []
        while queue:
            cj, ci = queue.popleft()
            cells.append((cj, ci))
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    nj, ni = cj + dj, ci + di
                    if (0 <= nj < mask.shape[0] and 0 <= ni < mask.shape[1]
                            and mask[nj, ni] and not seen[nj, ni]):
                        seen[nj, ni] = True
                        queue.append((nj, ni))
        out.append(cells)
    return out


# ---- 줄별 지표 ----
def assign(track: Track, plan: Plan) -> np.ndarray:
    """표본마다 가장 가까운 줄 번호. 방향이 다르거나 반 줄 간격보다 멀면 -1."""
    near = plan.line_spacing_m / 2.0
    best = np.full(len(track), -1)
    best_d = np.full(len(track), np.inf)
    for k, ln in enumerate(plan.lines):
        s, lat = ln.along(track.x, track.y)
        off = np.maximum(np.maximum(-s, s - ln.length), 0.0)
        d = np.hypot(off, lat)
        aligned = np.abs(_wrap(track.yaw - ln.heading)) < LINE_HEADING_RAD
        ok = aligned & (d <= near + _EPS) & (d < best_d)
        best[ok], best_d[ok] = k, d[ok]
    return best


def inner(track: Track, ln: Line, own: np.ndarray) -> np.ndarray:
    s, _ = ln.along(track.x, track.y)
    return np.nonzero(own & (s >= END_PAD_M) & (s <= ln.length - END_PAD_M))[0]


def lateral_stats(lat: np.ndarray) -> dict:
    if len(lat) == 0:
        return {"samples": 0}
    a = np.abs(lat)
    return {"samples": int(len(lat)), "mean_m": float(lat.mean()),
            "rms_m": float(np.sqrt((lat * lat).mean())), "p95_m": float(np.percentile(a, 95)),
            "max_m": float(a.max())}


def rates(track: Track) -> tuple[np.ndarray, np.ndarray]:
    """표본 사이 (선속도, 각속도). 시각이 같으면 0."""
    dt = np.diff(track.t)
    dist = np.hypot(np.diff(track.x), np.diff(track.y))
    dyaw = np.abs(_wrap(np.diff(track.yaw)))
    good = dt > _EPS
    v = np.where(good, dist / np.where(good, dt, 1.0), 0.0)
    w = np.where(good, dyaw / np.where(good, dt, 1.0), 0.0)
    return v, w


def turns(track: Track, plan: Plan, own: np.ndarray, idx: list[np.ndarray]) -> list[dict]:
    """줄 끝 회전: 줄 끝을 지난 때부터 다음 줄 방향 오차가 ALIGN_RAD 안에 들 때까지.
    시작 기준은 곧은 부분의 마지막 표본, 짧은 줄이라 없으면 그 줄에 붙은 첫 표본."""
    v, w = rates(track)
    near = plan.line_spacing_m / 2.0
    out = []
    for k in range(len(plan.lines) - 1):
        a, b = plan.lines[k], plan.lines[k + 1]
        mine = np.nonzero(own == k)[0]
        if len(mine) == 0:
            continue
        lo = int(idx[k][-1]) if len(idx[k]) else int(mine[0])
        nxt = np.nonzero(own == k + 1)[0]
        nxt = nxt[nxt > lo]
        if len(nxt) == 0:
            continue
        hi = int(nxt[-1])
        s_a, _ = a.along(track.x[lo:hi + 1], track.y[lo:hi + 1])
        first = int(nxt[0]) - lo
        past = np.nonzero(s_a[:first + 1] >= a.length - _EPS)[0]
        leave = lo + (int(past[0]) if len(past) else int(np.argmax(s_a[:first + 1])))
        # 다음 줄 선분(연장선이 아님)에서 반 줄 간격 안이어야 한다
        s_b, lat_b = b.along(track.x[leave:hi + 1], track.y[leave:hi + 1])
        d_b = np.hypot(np.maximum(np.maximum(-s_b, s_b - b.length), 0.0), lat_b)
        aligned = np.abs(_wrap(track.yaw[leave:hi + 1] - b.heading)) < ALIGN_RAD
        ok = aligned & (d_b <= near + _EPS)
        hits = np.nonzero(ok)[0]
        row = {"from": k, "to": k + 1,
               "overshoot_m": float(s_a[:first + 1].max() - a.length)}
        if len(hits):
            enter = leave + int(hits[0])
            seg = slice(leave, enter)
            row.update({"time_s": float(track.t[enter] - track.t[leave]),
                        "max_w_radps": float(w[seg].max()) if enter > leave else 0.0,
                        "in_place": bool(np.any((w[seg] > SPIN_RADPS) & (v[seg] < SPIN_MPS)))})
        out.append(row)
    return out


def end_uncovered(grid: Grid, ln: Line, missing: np.ndarray, half: float) -> tuple[float, float]:
    """줄 양 끝 END_PAD_M 구간의 청소 폭 안에서 덮지 못한 넓이(시작, 끝)."""
    xs, ys = np.meshgrid(grid.xc, grid.yc)
    s, lat = ln.along(xs, ys)
    band = np.abs(lat) <= half + _EPS
    head = band & (s >= 0.0) & (s <= END_PAD_M)
    tail = band & (s >= ln.length - END_PAD_M) & (s <= ln.length)
    return (float((head & missing).sum() * grid.cell_m2),
            float((tail & missing).sum() * grid.cell_m2))


def moving_span(track: Track) -> slice:
    """앞뒤로 제자리에 서 있던 표본을 뺀 구간."""
    d0 = np.hypot(track.x - track.x[0], track.y - track.y[0])
    d1 = np.hypot(track.x - track.x[-1], track.y - track.y[-1])
    moved0 = np.nonzero(d0 > STILL_M)[0]
    moved1 = np.nonzero(d1 > STILL_M)[0]
    if len(moved0) == 0:
        return slice(0, 1)
    return slice(max(int(moved0[0]) - 1, 0), int(moved1[-1]) + 2)


# ---- 한 자세 출처의 지표 ----
def source_metrics(track: Track, plan: Plan, grid: Grid, target: np.ndarray, half: float) -> dict:
    own = assign(track, plan)
    idx, rows, lat_all = [], [], []
    pts = np.column_stack([track.x, track.y])
    visits = sweep(grid, pts, half)
    covered = visits > 0
    missing = target & ~covered
    for k, ln in enumerate(plan.lines):
        i = inner(track, ln, own == k)
        idx.append(i)
        _, lat = ln.along(track.x[i], track.y[i])
        lat_all.append(lat)
        head, tail = end_uncovered(grid, ln, missing, half)
        rows.append({"line": k, "cell": ln.cell, **lateral_stats(lat),
                     "start_uncovered_m2": head, "end_uncovered_m2": tail})
    spacing = []
    for k in range(len(plan.lines) - 1):
        a, b = plan.lines[k], plan.lines[k + 1]
        if a.cell != b.cell or len(idx[k]) == 0 or len(idx[k + 1]) == 0:
            continue
        n = a.n
        c_a = ((pts[idx[k]] - a.start) @ n).mean()
        c_b = ((pts[idx[k + 1]] - a.start) @ n).mean()
        planned = float(abs((np.array(b.start) - a.start) @ n))
        spacing.append({"lines": [k, k + 1], "planned_m": planned,
                        "actual_m": float(abs(c_b - c_a))})
    span = moving_span(track)
    t, x, y = track.t[span], track.x[span], track.y[span]
    length = float(np.hypot(np.diff(x), np.diff(y)).sum())
    lap = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    gaps = []
    for cells in blobs(missing):
        jj = np.array([c[0] for c in cells])
        ii = np.array([c[1] for c in cells])
        gaps.append({"cells": len(cells), "area_m2": float(len(cells) * grid.cell_m2),
                     "centroid": [float(grid.xc[ii].mean()), float(grid.yc[jj].mean())],
                     "box": [float(grid.xc[ii].min()), float(grid.yc[jj].min()),
                             float(grid.xc[ii].max()), float(grid.yc[jj].max())]})
    gaps.sort(key=lambda g: -g["cells"])
    done = target & covered
    return {
        "topic": track.topic, "samples": len(track),
        "lap_time_s": lap, "path_length_m": length,
        "mean_speed_mps": length / lap if lap > 0 else 0.0,
        "lateral": lateral_stats(np.concatenate(lat_all) if lat_all else np.array([])),
        "lines": rows, "spacing": spacing, "turns": turns(track, plan, own, idx),
        "coverage": {
            "target_m2": float(target.sum() * grid.cell_m2),
            "covered_ratio": float(done.sum() / target.sum()) if target.any() else 0.0,
            "overlap_ratio": (float((target & (visits >= 2)).sum() / done.sum())
                              if done.any() else 0.0),
            "gap_m2": float(missing.sum() * grid.cell_m2), "gap_count": len(gaps), "gaps": gaps},
        "_covered": covered, "_visits": visits,
    }


def analyse(args) -> tuple[dict, dict]:
    topics = default_topics(args.ns)
    if args.truth_topic:
        topics["truth"] = args.truth_topic
    if args.amcl_topic:
        topics["amcl"] = args.amcl_topic
    tracks = read_tracks(Path(args.bag), topics)
    if not tracks:
        raise SystemExit(f"no pose topic in {args.bag}: looked for {sorted(topics.values())}")
    plan = load_plan(plan_path(args))
    half = args.clean_width / 2.0
    xs = np.concatenate([[p for ln in plan.lines for p in (ln.start[0], ln.end[0])]]
                        + [tr.x for tr in tracks.values()])
    ys = np.concatenate([[p for ln in plan.lines for p in (ln.start[1], ln.end[1])]]
                        + [tr.y for tr in tracks.values()])
    if plan.transits:
        pts = np.array([p for tr in plan.transits for p in tr])
        xs, ys = np.concatenate([xs, pts[:, 0]]), np.concatenate([ys, pts[:, 1]])
    grid = Grid(xs, ys, half + args.grid, args.grid)
    target = target_mask(grid, plan, half)
    report = {"bag": str(args.bag), "ns": args.ns, "plan": plan.source,
              "clean_width_m": args.clean_width, "grid_m": args.grid, "sources": {}}
    for key in SOURCES:
        if key in tracks:
            report["sources"][key] = source_metrics(tracks[key], plan, grid, target, half)
    src = report["sources"]
    if "truth" in src and "amcl" in src:
        only = target & src["amcl"]["_covered"] & ~src["truth"]["_covered"]
        report["believed_minus_actual"] = {
            "covered_ratio": (src["amcl"]["coverage"]["covered_ratio"]
                              - src["truth"]["coverage"]["covered_ratio"]),
            "believed_not_actual_m2": float(only.sum() * grid.cell_m2)}
    ctx = {"plan": plan, "grid": grid, "target": target, "tracks": tracks}
    return report, ctx


def public(report: dict) -> dict:
    out = dict(report)
    out["sources"] = {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")}
                      for k, v in report["sources"].items()}
    return out


# ---- 표 ----
def _cm(v) -> str:
    return "-" if v is None else f"{v * 100:.2f}"


def to_markdown(report: dict, top: int = 10) -> str:
    out = [f"# Coverage metrics: {report['bag']}", "",
           f"- namespace `{report['ns']}`, plan `{report['plan']}`",
           f"- clean width {report['clean_width_m']} m, grid {report['grid_m']} m", ""]
    for key, m in report["sources"].items():
        cov, lat = m["coverage"], m["lateral"]
        out += [f"## {ROLE[key]}: `{m['topic']}` ({m['samples']} samples)", "",
                "| item | value |", "|---|---|",
                f"| lap time | {m['lap_time_s']:.1f} s |",
                f"| path length | {m['path_length_m']:.3f} m |",
                f"| mean speed | {m['mean_speed_mps']:.3f} m/s |",
                f"| lateral mean / rms / p95 / max | {_cm(lat.get('mean_m'))} / "
                f"{_cm(lat.get('rms_m'))} / {_cm(lat.get('p95_m'))} / {_cm(lat.get('max_m'))} cm "
                f"({lat['samples']} samples) |",
                f"| coverage of planned swath | {cov['covered_ratio'] * 100:.2f} % of "
                f"{cov['target_m2']:.4f} m² |",
                f"| covered twice or more | {cov['overlap_ratio'] * 100:.2f} % |",
                f"| gaps | {cov['gap_count']} blobs, {cov['gap_m2'] * 1e4:.1f} cm² |", "",
                "| line | cell | samples | mean cm | rms cm | p95 cm | max cm | start gap cm² "
                "| end gap cm² |",
                "|---|---|---|---|---|---|---|---|---|"]
        for r in m["lines"]:
            out.append(f"| {r['line']} | {r['cell']} | {r['samples']} | {_cm(r.get('mean_m'))} | "
                       f"{_cm(r.get('rms_m'))} | {_cm(r.get('p95_m'))} | {_cm(r.get('max_m'))} | "
                       f"{r['start_uncovered_m2'] * 1e4:.1f} | {r['end_uncovered_m2'] * 1e4:.1f} |")
        if m["spacing"]:
            out += ["", "| lines | planned cm | actual cm |", "|---|---|---|"]
            out += [f"| {s['lines'][0]}-{s['lines'][1]} | {_cm(s['planned_m'])} | "
                    f"{_cm(s['actual_m'])} |"
                    for s in m["spacing"]]
        if m["turns"]:
            out += ["", "| turn | time s | max w rad/s | in place | overshoot cm |",
                    "|---|---|---|---|---|"]
            for t in m["turns"]:
                time_s = f"{t['time_s']:.2f}" if "time_s" in t else "not aligned"
                w = f"{t['max_w_radps']:.2f}" if "max_w_radps" in t else "-"
                out.append(f"| {t['from']}-{t['to']} | {time_s} | {w} | {t.get('in_place', '-')} | "
                           f"{_cm(t['overshoot_m'])} |")
        if cov["gaps"]:
            out += ["", f"| gap (largest {top}) | area cm² | centroid x, y | box |",
                    "|---|---|---|---|"]
            for n, g in enumerate(cov["gaps"][:top]):
                box = ", ".join(f"{v:.3f}" for v in g["box"])
                out.append(f"| {n} | {g['area_m2'] * 1e4:.1f} | {g['centroid'][0]:.3f}, "
                           f"{g['centroid'][1]:.3f} | {box} |")
        out.append("")
    if "believed_minus_actual" in report:
        d = report["believed_minus_actual"]
        out += ["## believed minus actual", "",
                f"- coverage difference {d['covered_ratio'] * 100:+.2f} points",
                f"- believed covered but not actually covered "
                f"{d['believed_not_actual_m2'] * 1e4:.1f} cm²", ""]
    return "\n".join(out)


# ---- 하위 명령 ----
def cmd_metrics(args) -> int:
    report, _ = analyse(args)
    text = (json.dumps(public(report), indent=2) if args.format == "json"
            else to_markdown(public(report)))
    if args.out:
        Path(args.out).write_text(text + "\n", encoding="utf-8")
        print(f"wrote {args.out}")
    else:
        print(text)
    return 0


def cmd_map(args) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    report, ctx = analyse(args)
    src = report["sources"]
    key = args.source or ("truth" if "truth" in src else "amcl")
    if key not in src:
        raise SystemExit(f"no {key} poses in {args.bag}")
    m, grid, target, plan = src[key], ctx["grid"], ctx["target"], ctx["plan"]
    # 0 바깥, 1 목표 안 미덮음(빈틈), 2 한 번, 3 두 번 이상, 4 목표 밖 덮음
    visits = m["_visits"]
    img = np.zeros(visits.shape, dtype=int)
    img[(visits > 0) & ~target] = 4
    img[target & (visits == 0)] = 1
    img[target & (visits == 1)] = 2
    img[target & (visits >= 2)] = 3
    cmap = ListedColormap(["#ffffff", "#d62728", "#a6dba0", "#1b7837", "#e0e0e0"])
    x0, x1, y0, y1 = grid.extent()
    fig, ax = plt.subplots(figsize=(10, max(3.0, 10 * (y1 - y0) / (x1 - x0) + 1.0)))
    ax.imshow(img, origin="lower", extent=grid.extent(), cmap=cmap, vmin=0, vmax=4,
              interpolation="nearest")
    for k, ln in enumerate(plan.lines):
        ax.plot([ln.start[0], ln.end[0]], [ln.start[1], ln.end[1]], "k--", lw=0.8,
                label="planned line" if k == 0 else None)
    for name, style in (("amcl", dict(color="tab:orange", ls=":", lw=0.9)),
                        ("truth", dict(color="tab:blue", lw=0.9))):
        if name in ctx["tracks"]:
            tr = ctx["tracks"][name]
            ax.plot(tr.x, tr.y, label=ROLE[name], **style)
    for g in m["coverage"]["gaps"][:args.top]:
        ax.annotate(f"{g['area_m2'] * 1e4:.0f} cm²", g["centroid"], color="#d62728", fontsize=7,
                    textcoords="offset points", xytext=(3, 3))
    cov = m["coverage"]
    ax.set_title(f"{ROLE[key]}: coverage {cov['covered_ratio'] * 100:.1f} %, "
                 f"gaps {cov['gap_count']} ({cov['gap_m2'] * 1e4:.0f} cm²), "
                 f"twice+ {cov['overlap_ratio'] * 100:.1f} %, width {args.clean_width} m",
                 fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(args.out, dpi=args.dpi)
    plt.close(fig)
    print(f"wrote {args.out}")
    return 0


def cmd_relocalize(args) -> int:
    print("relocalize (scan matching on the localization map, plan 6-3/6-4) is the next step; "
          "not implemented yet")
    return 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Cleaner coverage quality from a bag (plan 6-2, 6-4)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("bag", help="rosbag2 directory or .mcap file")
    common.add_argument("--ns", required=True, help="robot namespace, e.g. pode3")
    common.add_argument("--layout", default="v7", help="use fleet_core data coverage_<layout>.yaml")
    common.add_argument("--plan", help="coverage plan yaml (overrides --layout)")
    common.add_argument("--clean-width", type=float, default=CLEAN_WIDTH_M)
    common.add_argument("--grid", type=float, default=GRID_M)
    common.add_argument("--truth-topic")
    common.add_argument("--amcl-topic")
    p = sub.add_parser("metrics", parents=[common], help="line quality and coverage table")
    p.add_argument("--format", choices=("md", "json"), default="md")
    p.add_argument("--out")
    p.set_defaults(func=cmd_metrics)
    p = sub.add_parser("map", parents=[common], help="coverage map PNG")
    p.add_argument("--out", required=True)
    p.add_argument("--source", choices=SOURCES)
    p.add_argument("--top", type=int, default=10, help="label this many largest gaps")
    p.add_argument("--dpi", type=int, default=150)
    p.set_defaults(func=cmd_map)
    p = sub.add_parser("relocalize", parents=[common], help="next step: scan re-localization")
    p.set_defaults(func=cmd_relocalize)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())

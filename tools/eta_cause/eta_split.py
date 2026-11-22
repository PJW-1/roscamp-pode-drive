#!/usr/bin/env python3
"""넓은 구역(nav_goal) ETA 오차를 끝 종류와 원인별로 나눈다. 관제 기록을 이 checkout의 코어로 재생한다.

    python3 tools/eta_cause/eta_split.py 이름=기록.jsonl [이름=폴더 ...] [--every=0.5] [--rows=표본.json]

폴더를 주면 그 아래의 fleet_record.jsonl, fms.jsonl을 모두 읽는다(이름은 폴더 기준 상대 경로).
오차 = ETA(_left_s) - 실제(그 Segment의 SEGMENT_DONE 시각 - 표본 시각). -면 일찍 닿는다고 본 것이다.
끝 종류: pass(다음 조각이 nav_goal, follow_path라 넘김 거리를 뺌), turn(후진이나 Route 끝이라 끝 회전을 더함).
원인 나누기(위치 보고를 BIN_S마다 평균):
    오차 = 끝 보정 + 길이((ETA 거리 - 달린 거리)/v) + 속도(달린 거리/v - 움직인 시간) - 제자리 회전 - 서 있음
         - 보고 없음(위치 보고 칸이 덮지 못한 시간). v는 그 기록의 nav_speed_mps다.
읽기만 한다. 기록 형식은 tools/replay.py 머리에 있다.
"""
from __future__ import annotations

import importlib.util
import json
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))
_spec = importlib.util.spec_from_file_location("replay", REPO / "tools" / "replay.py")
replay = importlib.util.module_from_spec(_spec)
sys.modules["replay"] = replay
_spec.loader.exec_module(replay)

NAV_GOAL = importlib.import_module("fleet_core.fleet_map").NAV_GOAL
_routes = importlib.import_module("fleet_core.traffic.zone.routes")
PASS, nav_end_s, nav_stretch = _routes.PASS, _routes.nav_end_s, _routes.nav_stretch

BIN_S = 0.5
STAND_MPS = 0.03
TURN_RPS = 0.3
FAR_M = 0.3
NEAR_M = (0.04, 0.16)
BINS = ((0.0, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0), (8.0, math.inf))
RECORD_GLOBS = ("fleet_record*.jsonl", "fms*.jsonl")
KINDS = ("pass", "turn", "none")


# ---- 기록 찾기 ----

def split_arg(arg: str) -> tuple[str, Path]:
    if Path(arg).exists() or "=" not in arg:
        return "", Path(arg)
    name, _, path = arg.partition("=")
    return name, Path(path)


def records(args: list[str]) -> list[tuple[str, Path]]:
    out = []
    for arg in args:
        name, path = split_arg(arg)
        if path.is_dir():
            found = sorted({p for g in RECORD_GLOBS for p in path.rglob(g)})
            out += [(f"{name or path.name}/{p.relative_to(path).as_posix()}", p) for p in found]
        else:
            out.append((name or path.name, path))
    return out


# ---- 재생하며 표본 모으기 ----

def current_nav(m):
    mo, route = m.move_order, m.route
    if mo is None or route is None or m.goal is None:
        return None
    seq = mo.done_seq + 1
    cut = next((c for c in mo.cuts[:mo.given] if c[0] == seq), None)
    if cut is None or cut[1] != route.done or route.edges[cut[1]].mode != NAV_GOAL:
        return None
    return mo.id, seq, cut


def sample(traffic, m, cut) -> dict:
    route = m.route
    edges = route.edges
    chain = route.done + nav_stretch(edges[route.done:cut[2]])
    adj = nav_end_s(traffic.params, traffic.site, edges, chain, m.x, m.y, m.yaw, PASS)
    after = edges[chain] if chain < len(edges) else None
    nx, ny = traffic.site.node_pose(edges[chain - 1].b)[:2]
    return {"used": traffic._left_s(m, route, cut[2]), "adj": adj,
            "kind": "pass" if adj < 0 else ("turn" if adj > 0 else "none"),
            "next": after.mode if after is not None else "end",
            "x": m.x, "y": m.y, "nx": nx, "ny": ny,
            "from_distance": traffic._reported_end(m, route) is not None}


def replay_rows(path: Path, every: float = 0.5) -> tuple[list[dict], float | None, dict, float]:
    lines, entries, _ = replay._read(path)
    head, *rest = entries
    now = [0.0]
    core = replay.start_core(head, lambda: now[0])
    produced: list[str] = []
    core.record_to(produced.append)
    taken, last, done_at = [], {}, {}
    poses = defaultdict(lambda: defaultdict(list))
    for entry in rest:
        if "in" not in entry:
            continue
        now[0] = entry["t"]
        before = len(produced)
        getattr(core, entry["in"])(*replay._arguments(entry))
        for line in produced[before:]:
            out = json.loads(line)
            if out.get("out") == "EventOut" and out.get("code") == "SEGMENT_DONE":
                order, _, number = out["detail"].split(" ")[:3]
                done_at.setdefault((order, int(number)), out["t"])
        if entry["in"] != "on_robot_state":
            continue
        robot, x, y, yaw = entry["args"][:4]
        poses[robot][int(now[0] // BIN_S)].append((x, y, yaw))
        m = core.traffic._robots.get(robot)
        found = current_nav(m) if m is not None else None
        if found is None or now[0] - last.get(robot, -math.inf) < every - 1e-9:
            continue
        last[robot] = now[0]
        order, seq, cut = found
        taken.append({"seg": f"{order}#{seq}", "key": (order, seq), "t": now[0], "robot": robot,
                      **sample(core.traffic, m, cut)})
    differ = next((json.loads(new).get("t") for old, new in zip(lines[1:], produced[1:],
                                                                strict=False) if old != new),
                  None)
    rows = []
    for row in taken:
        end = done_at.get(row.pop("key"))
        if end is not None and end >= row["t"]:
            rows.append({**row, "end_t": end, "actual": end - row["t"],
                         "err": row["used"] - (end - row["t"])})
    return rows, differ, averaged(poses), core.traffic.params.nav_speed_mps


def averaged(poses) -> dict:
    out = {}
    for robot, bins in poses.items():
        seq = []
        for k in sorted(bins):
            pts = bins[k]
            yaw = math.atan2(st.mean(math.sin(p[2]) for p in pts),
                             st.mean(math.cos(p[2]) for p in pts))
            seq.append(((k + 0.5) * BIN_S, st.mean(p[0] for p in pts),
                        st.mean(p[1] for p in pts), yaw))
        out[robot] = seq
    return out


# ---- 원인 나누기 ----

def parts(row: dict, seq: list) -> dict:
    speed = row["v"]
    t0, t1 = row["t"], row["end_t"]
    driven = moving = turning = waiting = 0.0
    for a, b in zip(seq, seq[1:], strict=False):
        lo, hi = max(a[0], t0), min(b[0], t1)
        if hi <= lo:
            continue
        dt = b[0] - a[0]
        d = math.dist(a[1:3], b[1:3])
        if dt > 1.5 * BIN_S:
            continue
        if d / dt >= STAND_MPS:
            driven += d * (hi - lo) / dt
            moving += hi - lo
        elif abs(math.remainder(b[3] - a[3], math.tau)) / dt >= TURN_RPS:
            turning += hi - lo
        else:
            waiting += hi - lo
    geo = (row["used"] - row["adj"]) * speed
    return {"err": row["err"], "adj": row["adj"], "length": (geo - driven) / speed,
            "speed": driven / speed - moving, "turn": -turning, "wait": -waiting,
            "unseen": -(row["actual"] - moving - turning - waiting),
            "actual": row["actual"], "mps": driven / moving if moving > 0 else math.nan}


def at_done(row: dict, seq: list) -> tuple[float, float]:
    near = min(seq, key=lambda p: abs(p[0] - row["end_t"]))
    ax, ay = row["nx"] - row["x"], row["ny"] - row["y"]
    span = math.hypot(ax, ay) or 1.0
    along = ((near[1] - row["nx"]) * ax + (near[2] - row["ny"]) * ay) / span
    return along, math.dist(near[1:3], (row["nx"], row["ny"]))


def reach_to_done(row: dict, seq: list, near_m: float) -> float | None:
    reach = next((p[0] for p in seq if row["t"] <= p[0] <= row["end_t"] + BIN_S / 2
                  and math.dist(p[1:3], (row["nx"], row["ny"])) <= near_m), None)
    return None if reach is None else row["end_t"] - reach


# ---- 표 ----

def line_of(title: str, rows: list[dict]) -> str:
    if not rows:
        return f"{title}: n=0"
    e = [r["err"] for r in rows]
    return (f"{title}: n={len(rows)} segs={len({(r['run'], r['seg']) for r in rows})} "
            f"mean={st.mean(e):+.2f} median={st.median(e):+.2f} "
            f"mean_abs={st.mean(map(abs, e)):.2f} adj_mean={st.mean(r['adj'] for r in rows):+.2f}")


def parts_line(title: str, rows: list[dict], poses: dict) -> str:
    if not rows:
        return f"{title}: n=0"
    p = [parts(r, poses[r["run"]][r["robot"]]) for r in rows]
    keys = ("err", "adj", "length", "speed", "turn", "wait", "unseen", "actual")
    mps = [q["mps"] for q in p if not math.isnan(q["mps"])]
    return (f"{title}: n={len(p)} " + " ".join(f"{k}={st.mean(q[k] for q in p):+.2f}" for k in keys)
            + (f" moving_mps={st.mean(mps):.3f}" if mps else ""))


def first_samples(rows: list[dict]) -> list[dict]:
    first = {}
    for r in rows:
        first.setdefault((r["run"], r["seg"]), r)
    return list(first.values())


def report(rows: list[dict], poses: dict) -> list[str]:
    kinds = [k for k in KINDS if any(r["kind"] == k for r in rows)]
    out = ["## split", line_of("ALL", rows)]
    out += [line_of(f" kind={k}", [r for r in rows if r["kind"] == k]) for k in kinds]
    out += [line_of(f" next={k}", [r for r in rows if r["next"] == k])
            for k in sorted({r["next"] for r in rows})]
    for robot in sorted({r["robot"] for r in rows}):
        out.append(line_of(f" robot={robot}", [r for r in rows if r["robot"] == robot]))
    for lo, hi in BINS:
        for k in kinds:
            out.append(line_of(f" actual {lo:g}-{hi:g} kind={k}",
                               [r for r in rows if lo <= r["actual"] < hi and r["kind"] == k]))
    first = first_samples(rows)
    out.append(line_of("dispatch (first sample per segment)", first))
    out += [line_of(f" dispatch kind={k}", [r for r in first if r["kind"] == k]) for k in kinds]
    out += ["## causes", parts_line("all samples", rows, poses)]
    out += [parts_line(f" kind={k}", [r for r in rows if r["kind"] == k], poses) for k in kinds]
    out.append(parts_line("dispatch", first, poses))
    for k in kinds:
        out.append(parts_line(f" dispatch kind={k}", [r for r in first if r["kind"] == k], poses))
        for robot in sorted({r["robot"] for r in rows}):
            out.append(parts_line(f"   dispatch kind={k} {robot}",
                                  [r for r in first if r["kind"] == k and r["robot"] == robot],
                                  poses))
    out.append("## end of segment")
    far = [r for r in first if math.dist((r["x"], r["y"]), (r["nx"], r["ny"])) > FAR_M]
    for k in kinds:
        part = [r for r in far if r["kind"] == k]
        if not part:
            continue
        ends = [at_done(r, poses[r["run"]][r["robot"]]) for r in part]
        out.append(f"pose at SEGMENT_DONE kind={k}: n={len(ends)} "
                   f"along median={st.median(e[0] for e in ends):+.3f} m "
                   f"dist median={st.median(e[1] for e in ends):.3f} m")
        for near in NEAR_M:
            gaps = [g for g in (reach_to_done(r, poses[r["run"]][r["robot"]], near) for r in part)
                    if g is not None]
            if gaps:
                out.append(f"  done - reached {near:g} m kind={k}: reached {len(gaps)}/{len(part)} "
                           f"median={st.median(gaps):+.2f} s mean={st.mean(gaps):+.2f} s")
    used = sum(r["from_distance"] for r in rows)
    out.append(f"samples using distance_remaining: {used}/{len(rows)}")
    return out


# ---- 실행 ----

def run(args: list[str], every: float = 0.5) -> tuple[list[dict], dict, list[str], set]:
    rows, poses, warnings, speeds = [], {}, [], set()
    for name, path in records(args):
        got, differ, run_poses, speed = replay_rows(path, every)
        if differ is not None:
            warnings.append(f"warning: {name}: this code decides differently from t {differ} s on")
        for r in got:
            r["run"], r["v"] = name, speed
        rows += got
        poses[name] = run_poses
        speeds.add(speed)
    return rows, poses, warnings, speeds


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    opts = dict(a[2:].split("=", 1) for a in argv if a.startswith("--") and "=" in a)
    paths = [a for a in argv if not a.startswith("--")]
    if not paths:
        print(__doc__)
        return 2
    rows, poses, warnings, speeds = run(paths, float(opts.get("every", 0.5)))
    print("\n".join([*warnings, f"records: {len({r['run'] for r in rows})} with samples, "
                     f"nav_speed_mps {sorted(speeds)}", *report(rows, poses)]))
    if "rows" in opts:
        out = Path(opts["rows"])
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(rows), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())

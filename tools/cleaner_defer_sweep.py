"""청소 로봇 겹침 판정 조건 x seed 판을 머리 없는 세계에서 돌려 미룸 원인 표를 찍는다.

    python3 tools/cleaner_defer_sweep.py                      # 모든 조건, seed 1~5
    python3 tools/cleaner_defer_sweep.py -c default ahead05 --seeds 1 2 --jobs 4
    python3 tools/cleaner_defer_sweep.py --json out.json --md out.md

판: 운반 pode1(P-2)·pode2(P-3) + 청소 pode3(P-1), cleaners pode3. 운반 주문이 모두 끝나고 청소
첫 바퀴가 끝나면(청소 없음 판은 주문만, 청소 혼자 판은 바퀴만) 멈춘다. 상한 900 s.
지연은 같은 배치·seed의 청소 없음 판과 비교한다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for sub in ("pc/fleet_core", "sim/fake_robots"):
    if str(REPO / sub) not in sys.path:
        sys.path.insert(0, str(REPO / sub))

CLEANER = "pode3"
TRANSPORTS = {"pode1": "P-2", "pode2": "P-3"}
LIMIT_S = 900.0
STALL_S = 90.0  # 이만큼 어떤 관제 사건도 없으면 서로 막힌 것으로 보고 판을 끝낸다
TRAIL_STEP_M = 0.02  # 받은 위치는 이만큼 움직일 때마다 한 점씩 이어 덮은 칸을 잰다

# ---- 조건: (이름, 배치, 판 종류, 파라미터 바꿈) ----
NO_CLEANER, ALONE, FLEET = "no_cleaner", "alone", "fleet"
AHEAD = {"clean_hold_check": "ahead", "clean_base_check": "ahead"}
R3_ONLY = {"clean_hold_check": "off", "clean_base_check": "off"}
CONDITIONS = [
    ("no_cleaner", "v7", NO_CLEANER, {}),
    ("alone", "v7", ALONE, {}),
    ("default", "v7", FLEET, {}),
    ("no_tuck", "v7", FLEET, {"clean_tuck_rows": 0}),
    ("reach015", "v7", FLEET, {"clean_conflict_reach_m": 0.15}),
    ("r2_off", "v7", FLEET, {"clean_hold_check": "off"}),
    ("r4_off", "v7", FLEET, {"clean_base_check": "off"}),
    ("ahead05", "v7", FLEET, {**AHEAD, "clean_conflict_ahead_m": 0.5}),
    ("ahead10", "v7", FLEET, {**AHEAD, "clean_conflict_ahead_m": 1.0}),
    ("r3_only", "v7", FLEET, R3_ONLY),
    ("reach015_ahead05", "v7", FLEET,
     {**AHEAD, "clean_conflict_ahead_m": 0.5, "clean_conflict_reach_m": 0.15}),
    ("reach015_r3_only", "v7", FLEET, {**R3_ONLY, "clean_conflict_reach_m": 0.15}),
    ("d2_no_cleaner", "d2", NO_CLEANER, {}),
    ("d2_alone", "d2", ALONE, {}),
    ("d2_default", "d2", FLEET, {}),
    ("d2_no_tuck", "d2", FLEET, {"clean_tuck_rows": 0}),
    ("d2_reach015", "d2", FLEET, {"clean_conflict_reach_m": 0.15}),
    ("d2_r3_only", "d2", FLEET, R3_ONLY),
]
NAMES = [c[0] for c in CONDITIONS]

# ---- 미룸 이유 문자열 → 원인 ----
CAUSES = [("R1", "not a shared zone"), ("R2", " is held by "), ("R3", " stands near"),
          ("R4", "'s base passes near"), ("take", "cannot take"),
          ("path", "the joined path is not clear"), ("blocked", "blocked for"),
          ("meet", "meets "), ("wait", " waits for ")]
CAUSE_NAMES = [c for c, _ in CAUSES] + ["other"]
TRANSPORT_BLOCK = {"BLOCKED", "BLOCKED_WAIT", "BLOCK_CONFIRMED", "ZONE_MARKED_BLOCKED"}
TRANSPORT_YIELD = {"HALL_YIELD", "LOCAL_YIELD", "ZONE_WAIT", "RETREAT_WAIT", "BASE_END_WAIT"}
# 청소 로봇 비키기(4-6)와 줄 앞 기다림 사건: 판 전체에서 센다. BACKUP은 후진으로 비킨 CLEAN_YIELD
CLEAN_YIELD_CODES = ("CLEAN_YIELD", "BACKUP", "CLEAN_HOME", "CLEAN_STUCK", "CLEAN_LINE_WAIT")
LAP = re.compile(r"lap 1 in ([\d.]+) s: (\d+)/(\d+) lines, (\d+) deferrals, skipped \[([^\]]*)\]"
                 r".*covered ([\d.]+)")


def cause(detail: str) -> str:
    why = detail.split(": ", 1)[-1]
    return next((name for name, key in CAUSES if key in why), "other")


# ---- 첫 바퀴 사건: 시각이 아니라 사건 순서로 자른다(같은 걸음에 다음 바퀴가 시작될 수 있다) ----
def first_lap(ev):
    start = next((i for i, (_, e) in enumerate(ev)
                  if e.code == "CLEAN_LAP_START" and "lap 1," in e.detail), None)
    if start is None:
        return [], None
    end = next((i for i in range(start, len(ev)) if ev[i][1].code == "CLEAN_LAP_DONE"
                and ev[i][1].detail.startswith("lap 1 ")), None)
    if end is None:
        return ev[start:], None
    return ev[start:end + 1], ev[end]


# ---- 판 하나 ----
def run_one(name: str, layout: str, kind: str, extra: dict, seed: int, maps=None) -> dict:
    from fake_robots.world.world import ZoneWorld
    from fleet_core.coverage import covered_by

    robots = {} if kind == ALONE else dict(TRANSPORTS)
    if kind != NO_CLEANER:
        robots[CLEANER] = "P-1"
    kwargs = {"orders": []} if kind == ALONE else {}
    w = ZoneWorld(layout=(maps or {}).get(layout, layout), seed=seed, robots=robots, overrides=dict(extra),
                  cleaners=() if kind == NO_CLEANER else (CLEANER,), **kwargs)
    cleaning = kind != NO_CLEANER
    transports = [n for n in robots if n != CLEANER]
    job = w.traffic._cleaners.get(CLEANER) if cleaning else None
    orders_end = None if transports else 0.0
    lap_done = not cleaning
    min_gap, trails, trail = math.inf, [], []
    seen = 0
    was_blocked = dict.fromkeys(transports, False)
    blocked_onsets = Counter()

    last_event, stalled = 0.0, None
    while w.t < LIMIT_S:
        n_events = len(w.events)
        w.step()
        if len(w.events) > n_events:
            last_event = w.t
        elif w.t - last_event > STALL_S:
            stalled = last_event
            break
        if orders_end is None and w.orders_done():
            orders_end = w.t
        if cleaning and not lap_done:
            lap_done = any(e.code == "CLEAN_LAP_DONE" for _, e in w.events[seen:])
            seen = len(w.events)
        # 청소 로봇: 첫 바퀴 줄·우회 중 관제가 받은 위치, 운반 로봇과의 실제 최소 거리
        if cleaning:
            c, got = w.robots[CLEANER], w.traffic._robots[CLEANER]
            for n in transports:
                r = w.robots[n]
                if not r.removed:
                    min_gap = min(min_gap, math.dist((c.x, c.y), (r.x, r.y)))
            on_line = (job.lap == 1 and job.goal_id is not None and job.leg is not None
                       and job.leg.kind in ("line", "detour"))
            if on_line:
                if not trail or math.dist(trail[-1], (got.x, got.y)) >= TRAIL_STEP_M:
                    trail.append((got.x, got.y))
            elif trail:
                trails.append(trail)
                trail = []
        for n in transports:
            run = w.robots[n].run
            now_blocked = run is not None and getattr(run, "blocked", False)
            if now_blocked and not was_blocked[n]:
                blocked_onsets[n] += 1
            was_blocked[n] = now_blocked
        if orders_end is not None and lap_done:
            break
    if trail:
        trails.append(trail)

    out = {"condition": name, "layout": layout, "kind": kind, "seed": seed, "overrides": extra,
           "t_end": w.t, "orders_end": orders_end, "guard_stops": w.guard_stops,
           "stalled_at": stalled,
           # 막혔을 때 누가 무엇을 쥐고 무엇을 기다리나
           "stall_state": None if stalled is None else {
               "cleaner_stage": job.stage if job else None,
               "held": {n: list(w.grants().nodes_of(n)) for n in robots},
               "waiting": {n: list(v) for n, v in w.grants().waiting().items()}},
           "min_gap": round(min_gap, 4) if math.isfinite(min_gap) else None}
    viol = list(w.violations)
    out["violations"] = [f"{v.code} {v.robot} {v.t:.1f}" for v in viol]
    out["violations_cleaner"] = sum(v.robot == CLEANER or CLEANER in v.detail for v in viol)
    out["i5_standing"] = sum(v.code == "I5_STANDING" for v in viol)
    tev = Counter(e.code for _, e in w.events if e.robot in transports)
    out["transport_blocked_events"] = sum(tev[c] for c in TRANSPORT_BLOCK)
    out["transport_yield_events"] = sum(tev[c] for c in TRANSPORT_YIELD)
    out["transport_blocked_onsets"] = sum(blocked_onsets.values())
    if not cleaning:
        return out
    cev = Counter(e.code for _, e in w.events if e.robot == CLEANER)
    cev["BACKUP"] = sum(e.code == "CLEAN_YIELD" and e.detail.startswith("backs up")
                        for _, e in w.events if e.robot == CLEANER)
    out["clean_yield_events"] = {c: cev[c] for c in CLEAN_YIELD_CODES}

    # ---- 첫 바퀴: 끝, 미룸 원인, 덮은 비율 ----
    lap, done = first_lap([(t, e) for t, e in w.events if e.robot == CLEANER])
    causes, lines = Counter(), Counter()
    for _, e in lap:
        if e.code in ("CLEAN_LINE_DEFERRED", "CLEAN_LINE_SKIPPED"):
            reason = cause(e.detail)
            causes[reason] += 1
            lines[(int(e.detail.split()[1]), reason)] += 1
    out["defer_causes"] = {c: causes[c] for c in CAUSE_NAMES}
    out["deferrals"] = sum(causes.values())
    out["defer_lines"] = {f"{k}:{r}": v for (k, r), v in sorted(lines.items())}
    out["cleaner_blocked_wait"] = sum(e.code == "BLOCKED_WAIT" for _, e in lap)
    plan = w.traffic._clean_plan
    hit = covered_by(plan.grid, plan.params, trails) if trails else None
    out["covered_received"] = (round(float((hit & plan.free).sum() / plan.free.sum()), 4)
                               if hit is not None else 0.0)
    if done:
        m = LAP.search(done[1].detail)
        out.update(lap_s=float(m[1]), lines_done=int(m[2]), lines_total=int(m[3]),
                   lap_deferrals=int(m[4]),
                   skipped=[int(x) for x in m[5].split(",") if x.strip()],
                   covered_planned=float(m[6]))
    else:
        out.update(lap_s=None, lines_done=len(job.done), lines_total=len(job.segs),
                   lap_deferrals=sum(job.deferrals.values()), skipped=sorted(job.skipped),
                   covered_planned=None)
    return out


def _run(args):
    return run_one(*args)


# ---- 조건별 요약 ----
def mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def summarize(runs: list[dict]) -> list[dict]:
    base = {(r["layout"], r["seed"]): r["orders_end"] for r in runs if r["kind"] == NO_CLEANER}
    rows = []
    for name in dict.fromkeys(r["condition"] for r in runs):
        rs = sorted((r for r in runs if r["condition"] == name), key=lambda r: r["seed"])
        row = {"condition": name, "layout": rs[0]["layout"], "kind": rs[0]["kind"],
               "overrides": rs[0]["overrides"], "seeds": [r["seed"] for r in rs]}
        delays = [r["orders_end"] - base[(r["layout"], r["seed"])]
                  for r in rs if r["kind"] == FLEET and r["orders_end"] is not None
                  and base.get((r["layout"], r["seed"])) is not None]
        row["orders_end"] = mean(r["orders_end"] for r in rs)
        row["orders_unfinished"] = sum(r["orders_end"] is None for r in rs)
        row["stalled"] = [r["seed"] for r in rs if r["stalled_at"] is not None]
        row["delay_mean"] = mean(delays)
        row["delay_max"] = max(delays) if delays else None
        row["violations"] = sum(len(r["violations"]) for r in rs)
        row["violations_cleaner"] = sum(r["violations_cleaner"] for r in rs)
        row["i5_standing"] = sum(r["i5_standing"] for r in rs)
        row["guard_stops"] = sum(r["guard_stops"] for r in rs)
        for key in ("transport_blocked_events", "transport_yield_events",
                    "transport_blocked_onsets"):
            row[key] = sum(r[key] for r in rs)
        if rs[0]["kind"] != NO_CLEANER:
            row["lines_done"] = [r["lines_done"] for r in rs]
            row["lines_total"] = rs[0]["lines_total"]
            row["deferrals"] = [r["deferrals"] for r in rs]
            row["causes"] = {c: sum(r["defer_causes"][c] for r in rs) for c in CAUSE_NAMES}
            row["skipped"] = [len(r["skipped"]) for r in rs]
            row["covered_received"] = mean(r["covered_received"] for r in rs)
            row["covered_planned"] = mean(r["covered_planned"] for r in rs)
            row["lap_s"] = mean(r["lap_s"] for r in rs)
            row["laps_unfinished"] = sum(r["lap_s"] is None for r in rs)
            gaps = [r.get("min_gap") for r in rs]
            row["min_gap"] = min((g for g in gaps if g is not None), default=None)
            row["clean_yield_events"] = {c: sum(r["clean_yield_events"][c] for r in rs)
                                         for c in CLEAN_YIELD_CODES}
        rows.append(row)
    return rows


def _f(value, digits=1) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def _span(values) -> str:
    return f"{min(values)}~{max(values)}" if min(values) != max(values) else str(values[0])


def markdown(rows: list[dict]) -> str:
    lines = ["| 조건 | 바꾼 값 | 끝낸 줄 | 미룸 | R1 | R2 | R3 | R4 | take | path | BLOCKED | meet "
             "| wait | 기타 "
             "| 포기 줄 | 덮은 비율(받은 위치) | 첫 바퀴 s | 주문 끝 s | 지연 평균/최대 s "
             "| 최소 거리 m | 위반(청소) | I5 | 운반 BLOCKED 사건/멈춤 | 운반 양보·기다림 "
             "| 비키기/후진/집/STUCK/기다림 |",
             "|" + "---|" * 25]
    for r in rows:
        changed = ", ".join(f"{k.replace('clean_', '')}={v}" for k, v in r["overrides"].items())
        changed = changed or "-"
        tail = (f"| {_f(r['orders_end'])}"
                + (f" ({r['orders_unfinished']} 못 끝냄)" if r["orders_unfinished"] else "")
            + (f" 막힘 seed {r['stalled']}" if r["stalled"] else "")
                + f" | {_f(r['delay_mean'])}/{_f(r['delay_max'])} ")
        if r["kind"] == NO_CLEANER:
            lines.append(f"| {r['condition']} | {changed} |" + " - |" * 15 + tail[1:]
                         + f"| - | {r['violations']}(-) | {r['i5_standing']} "
                         f"| {r['transport_blocked_events']}/{r['transport_blocked_onsets']} "
                         f"| {r['transport_yield_events']} | - |")
            continue
        c = r["causes"]
        lines.append(
            f"| {r['condition']} | {changed} | {_span(r['lines_done'])}/{r['lines_total']} "
            f"| {_span(r['deferrals'])} | " + " | ".join(str(c[k]) for k in CAUSE_NAMES)
            + f" | {_span(r['skipped'])} | {_f(r['covered_received'], 3)} | {_f(r['lap_s'])}"
            + (f" ({r['laps_unfinished']} 못 끝냄)" if r["laps_unfinished"] else "")
            + f" {tail}| {_f(r['min_gap'], 3)} | {r['violations']}({r['violations_cleaner']}) "
            f"| {r['i5_standing']} | {r['transport_blocked_events']}/"
            f"{r['transport_blocked_onsets']} | {r['transport_yield_events']} "
            f"| {'/'.join(str(v) for v in r['clean_yield_events'].values())} |")
    return "\n".join(lines)


def map_line(layouts, maps) -> str:
    """표 머리: 쓴 지도 파일과 그 내용의 sha1 앞 10자리."""
    from fleet_core.fleet_map.fleet_map import DATA
    parts = []
    for layout in sorted(layouts):
        path = Path(maps.get(layout, DATA / f"fleet_map_{layout}.yaml"))
        digest = hashlib.sha1(path.read_bytes()).hexdigest()[:10]
        parts.append(f"{layout} = `{path}` (sha1 {digest})")
    return "지도: " + ", ".join(parts)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-c", "--conditions", nargs="+", choices=NAMES, default=NAMES)
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--jobs", type=int, default=0, help="프로세스 수(0: CPU 수)")
    ap.add_argument("--json", type=Path, help="판마다 결과와 요약을 쓸 json")
    ap.add_argument("--md", type=Path, help="표를 쓸 md")
    ap.add_argument("--map", action="append", default=[], metavar="LAYOUT=PATH",
                    help="배치 이름 대신 쓸 지도 yaml (예: v7=pc/fleet_core/fleet_core/data/"
                         "fleet_map_v7.yaml). 여러 번 줄 수 있다")
    args = ap.parse_args(argv)
    chosen = [c for c in CONDITIONS if c[0] in args.conditions]
    # 지연을 재려면 같은 배치의 청소 없음 판이 있어야 한다
    for layout in {c[1] for c in chosen}:
        if not any(c[1] == layout and c[2] == NO_CLEANER for c in chosen):
            chosen.insert(0, next(c for c in CONDITIONS if c[1] == layout and c[2] == NO_CLEANER))
    maps = dict(pair.split("=", 1) for pair in args.map)
    tasks = [(*c, seed, maps) for c in chosen for seed in args.seeds]
    with ProcessPoolExecutor(max_workers=args.jobs or None) as pool:
        runs = list(pool.map(_run, tasks))
    rows = summarize(runs)
    table = map_line({c[1] for c in chosen}, maps) + "\n\n" + markdown(rows)
    print(table)
    if args.md:
        args.md.write_text(table + "\n", encoding="utf-8")
    if args.json:
        args.json.write_text(json.dumps({"runs": runs, "rows": rows}, ensure_ascii=False,
                                        indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())

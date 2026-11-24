"""청소 경로 요약, 경로 최소 거리, 바닥 물체 판을 한 저장소(코드와 지도)에서 다시 잰다.

    python3 measure_extra.py REPO [--map v7=FILE] > out.yaml
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import yaml


# ---- 둥근 경로를 2 mm 표본으로 나눠 벽과 막힌 사각형까지 가장 가까운 거리 ----
def sampled_clearance(cov, plan, step=0.002):
    grid, best = plan.grid, math.inf
    for path in plan.paths():
        pts = cov.rounded(path, plan.params.turn_radius_m)
        for a, b in zip(pts, pts[1:]):
            n = max(1, math.ceil(math.dist(a, b) / step))
            for k in range(n + 1):
                p = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                d = min(cov._wall(grid.box, p), *(cov._point_rect(p, r) for r in grid.blocked))
                best = min(best, d)
    return best


def exact_clearance(cov, plan):
    best = math.inf
    for path in plan.paths():
        pts = cov.rounded(path, plan.params.turn_radius_m)
        for a, b in zip(pts, pts[1:]):
            best = min(best, cov.segment_clearance(plan.grid, a, b))
    return best


def coverage_rows(cov, load, maps):
    out = {}
    for layout in ("v7", "d2"):
        site = load(maps.get(layout, layout))
        for spacing in (0.10, 0.068):
            plan = cov.make_plan(site, cov.Params(line_spacing_m=spacing))
            row = {"home_moved": plan.home_moved, "start": [round(v, 4) for v in plan.start],
                   **cov.summary(plan), "problems": len(cov.rounded_problems(plan))}
            if spacing == 0.10:
                row["clearance_sampled_2mm"] = round(sampled_clearance(cov, plan), 4)
                row["clearance_exact"] = round(exact_clearance(cov, plan), 4)
                row["line_starts_y"] = sorted({round(a[1], 4) for a, _, _ in plan.lines})
            out[f"{layout}_spacing_{spacing}"] = row
    return out


# ---- 바닥 물체 판: 청소 로봇만, 물체 (0.35, 0.4875) (test_world_cleaner와 같음) ----
LAP = re.compile(r"lap 1 in ([\d.]+) s: (\d+)/(\d+) lines, (\d+) deferrals, skipped \[([^\]]*)\]"
                 r".*covered ([\d.]+)")


def obstacle_run(maps):
    from fake_robots.world import Obstacle
    from fake_robots.world.world import ZoneWorld
    w = ZoneWorld(layout=maps.get("v7", "v7"), seed=1, robots={"pode3": "P-1"},
                  cleaners=("pode3",), orders=[], injections=[Obstacle(0.35, 0.4875, 0.05)])
    laps = lambda n: sum(e.code == "CLEAN_LAP_DONE" for _, e in w.events) >= n  # noqa: E731
    w.run_until(lambda: laps(1), 600)
    w.run_until(lambda: any(e.code == "CLEAN_LAP_START" and "lap 2" in e.detail
                            for _, e in w.events), 60)
    done = next((e.detail for _, e in w.events if e.code == "CLEAN_LAP_DONE"), "")
    lap2 = next((e.detail for _, e in w.events
                 if e.code == "CLEAN_LAP_START" and "lap 2" in e.detail), "")
    codes = [e.code for _, e in w.events if e.robot == "pode3"]
    gaps = [e.detail for _, e in w.events if e.code == "CLEAN_GAP"]
    return {"lap_done": done, "lap2_start": lap2, "gaps": gaps,
            "blocked_wait": codes.count("BLOCKED_WAIT"), "detours": codes.count("CLEAN_DETOUR"),
            "detour_failed": codes.count("CLEAN_DETOUR_FAILED"),
            "violations": [f"{v.code} {v.robot} {v.t:.1f}" for v in w.violations]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("repo", type=Path)
    ap.add_argument("--map", action="append", default=[])
    ap.add_argument("--skip-obstacle", action="store_true")
    a = ap.parse_args()
    for sub in ("pc/fleet_core", "sim/fake_robots"):
        sys.path.insert(0, str(a.repo / sub))
    maps = dict(p.split("=", 1) for p in a.map)
    from fleet_core import coverage as cov
    from fleet_core.fleet_map import load_fleet_map
    out = {"repo": str(a.repo), "maps": maps, "coverage": coverage_rows(cov, load_fleet_map, maps)}
    if not a.skip_obstacle:
        out["obstacle_run"] = obstacle_run(maps)
    print(yaml.safe_dump(out, sort_keys=False, allow_unicode=True, width=200))


if __name__ == "__main__":
    main()

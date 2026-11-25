"""줄 끝에서 멈춘 자리와 장애물 거리. 쓰는 법: PYTHONPATH=pc/fleet_core:sim/fake_robots python3 docs/cleaner_yield_main_2026-11-25/why_path_defer.py 5"""
import math, sys
from fake_robots.world.world import ZoneWorld
from fleet_core import coverage as cov
seed = int(sys.argv[1])
w = ZoneWorld(layout="v7", seed=seed, robots={"pode3": "P-1"}, orders=[], cleaners=("pode3",))
job = w.traffic._cleaners["pode3"]; plan = w.traffic._clean_plan; g = plan.grid
seen = 0
while w.t < 200:
    w.step()
    for _, e in w.events[seen:]:
        if e.code in ("CLEAN_LINE_DEFERRED", "CLEAN_LINE_DONE", "CLEAN_LAP_DONE"):
            got = w.traffic._robots["pode3"]; true = w.robots["pode3"]
            here = (got.x, got.y)
            cell = g.cell(here); inside = cell is not None and plan.free[cell[1], cell[0]]
            clr = cov.segment_clearance(g, here, here)
            print(f"{w.t:6.1f} {e.code:20s} {e.detail[:60]:60s} got=({got.x:.3f},{got.y:.3f}) true=({true.x:.3f},{true.y:.3f}) free_cell={inside} clearance={clr:.3f} reach={g.reach:.3f}")
    seen = len(w.events)
    if any(e.code == "CLEAN_LAP_DONE" for _, e in w.events): break

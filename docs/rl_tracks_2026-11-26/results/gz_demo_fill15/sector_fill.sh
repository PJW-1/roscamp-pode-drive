#!/usr/bin/env bash
# 열다섯째 Gazebo 묶음의 g_demo: 기록된 로봇 자세에서 벽·진열대·카트·사람에 광선을 다시 쏘아, 넓은 가림 + 채우기가
# 정책의 섹터 값을 얼마나 바꿨는지 어림한다(가장 가까워지기 전 3 s). 정책이 실제로 본 값이 아니다. 읽기만 한다.
D=$HOME/rl_gz_check/demo_fill15_n26; CFG=$D/gz_demo.yaml
source /opt/ros/jazzy/setup.bash > /dev/null 2>&1
cd "$HOME/nf_src/pd-rl-int/tools/nav_bench" && nice -n 19 python3 - "$D/main/legs" "$CFG" <<'PY'
import sys, json, math, glob
from bisect import bisect_left
import metrics as mt
import movers as mv
legs, cfgp = sys.argv[1], sys.argv[2]
cfg = mv.load(cfgp)
statics = mt.arena_rects()
body = mt.footprint()
LIDAR_X, OBS_MAX, REACH = -0.017, 2.0, 4
MASK = [(138.0, 157.0), (-152.5, -137.0)]
RAYS = [-180 + k for k in range(360)]
MASKED = [any(lo - 1e-6 <= a <= hi + 1e-6 for lo, hi in MASK) for a in RAYS]

def cast(ox, oy, ang, polys):
    dx, dy = math.cos(ang), math.sin(ang)
    best = OBS_MAX
    for poly in polys:
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > 0.0 and 0.0 <= u <= 1.0 and t < best:
                best = t
    return best

def sectors(rays, use):
    out, usable = [], []
    for s in range(36):
        vals = [rays[k] for k in range(10 * s, 10 * s + 10) if use[k]]
        usable.append(bool(vals))
        out.append(min(vals) if vals else OBS_MAX)
    return out, usable

def fill(vals, usable):
    out = list(vals)
    for s in range(36):
        if usable[s]:
            continue
        best = OBS_MAX
        for step in (-1, 1):
            for d in range(1, REACH + 1):
                j = (s + step * d) % 36
                if usable[j]:
                    best = min(best, vals[j])
                    break
        out[s] = best
    return out

WATCH = [2, 3, 4, 31, 32, 33]
print("sector s covers rays -180+10s .. -171+10s deg (base frame); watched:", WATCH, "(3 and 32 are emptied by the mask)")
for f in sorted(glob.glob(legs + "/*.json")):
    rec = json.load(open(f))
    name = rec["controller"].replace("FollowPathRLShim", "")
    if rec["scenario"] != "g_demo" or name == "DemoHistMask":
        continue
    names = rec.get("movers_names", [])
    track = rec["movers"]; times = [r[0] for r in track]
    truth = mt.within(rec["truth"], rec["t_start"], rec["t_end"])
    ci = names.index("demo_cart")
    dist = []
    for t, x, y, yaw in truth:
        k = min(bisect_left(times, t), len(times) - 1)
        cart = mv.mover_shape(cfg["movers"]["demo_cart"], tuple(track[k][1:][ci]))
        dist.append(mv.poly_dist(mv.to_world(body, x, y, yaw), cart))
    first0 = next((i for i, d in enumerate(dist) if d <= 0.001), None)
    iref = first0 if first0 is not None else min(range(len(dist)), key=lambda i: dist[i])
    tref = truth[iref][0]
    print(f'## {name} {rec["repeat"]} {rec["status"][:4]} cart min {min(dist):.3f} ref t {tref - rec["t_start"]:.1f}s ({"first contact" if first0 is not None else "closest"})')
    last, worst, worst_row = -9.0, 0.0, None
    for (t, x, y, yaw) in truth:
        if not (tref - 3.0 <= t <= tref + 0.3) or t - last < 0.25:
            continue
        last = t
        k = min(bisect_left(times, t), len(times) - 1)
        polys = list(statics) + [mv.mover_shape(cfg["movers"][n], tuple(p)) for n, p in zip(names, track[k][1:])]
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        rays = [cast(ox, oy, yaw + math.radians(a), polys) for a in RAYS]
        plain, _ = sectors(rays, [True] * 360)
        part, usable = sectors(rays, [not m for m in MASKED])
        filled = fill(part, usable)
        diff = [filled[s] - plain[s] for s in range(36)]
        big = max(range(36), key=lambda s: abs(diff[s]))
        if abs(diff[big]) > worst:
            worst, worst_row = abs(diff[big]), (t - rec["t_start"], big, plain[big], filled[big])
        cells = " ".join(f"{s}:{plain[s]:.2f}>{filled[s]:.2f}" for s in WATCH)
        print(f"   t {t - rec['t_start']:5.1f} yaw {math.degrees(yaw):6.1f} | {cells} | max diff sector {big} {diff[big]:+.2f} m")
    if worst_row:
        print(f"   largest change in the window: t {worst_row[0]:.1f}s sector {worst_row[1]} plain {worst_row[2]:.2f} m -> filled {worst_row[3]:.2f} m")
PY

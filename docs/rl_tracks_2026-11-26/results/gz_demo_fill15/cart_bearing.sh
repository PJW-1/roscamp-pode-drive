#!/usr/bin/env bash
# 열다섯째 Gazebo 묶음의 g_demo 판마다, 가장 가까워지기 전에 통로 카트의 가장 가까운 점이 로봇 기준 어느 방위에 있었는지 본다. 읽기만 한다.
D=$HOME/rl_gz_check/demo_fill15_n26; CFG=$D/gz_demo.yaml
source /opt/ros/jazzy/setup.bash > /dev/null 2>&1
cd "$HOME/nf_src/pd-rl-int/tools/nav_bench" && nice -n 19 python3 - "$D/main/legs" "$CFG" <<'PY'
import sys, json, math, glob
from bisect import bisect_left
import metrics as mt
import movers as mv
legs, cfgp = sys.argv[1], sys.argv[2]
cfg = mv.load(cfgp)
body = mt.footprint()
MASK = [(138.0, 157.0), (-152.5, -137.0)]

def seg_pts(poly, n=40):
    out = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        for k in range(n):
            u = k / n
            out.append((a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u))
    return out

def masked(deg):
    return any(lo <= deg <= hi for lo, hi in MASK)

print("body footprint:", [(round(x, 3), round(y, 3)) for x, y in body])
for f in sorted(glob.glob(legs + "/*.json")):
    rec = json.load(open(f))
    if rec["scenario"] != "g_demo":
        continue
    names = rec.get("movers_names", [])
    if "demo_cart" not in names:
        continue
    ci = names.index("demo_cart")
    m = cfg["movers"]["demo_cart"]
    track = rec["movers"]; times = [r[0] for r in track]
    truth = mt.within(rec["truth"], rec["t_start"], rec["t_end"])
    rows = []
    for t, x, y, yaw in truth:
        k = min(bisect_left(times, t), len(times) - 1)
        cart = mv.mover_shape(m, tuple(track[k][1:][ci]))
        shape = mv.to_world(body, x, y, yaw)
        d = mv.poly_dist(shape, cart)
        # nearest cart boundary point to the robot centre line: by distance to the body polygon
        best = None
        for p in seg_pts(cart):
            dp = mv.poly_dist(shape, [p, p, p])
            if best is None or dp < best[0]:
                best = (dp, p)
        px, py = best[1]
        dx, dy = px - x, py - y
        bx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        by = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        rows.append((t - rec["t_start"], d, math.degrees(math.atan2(by, bx)), math.hypot(bx, by), x, y, math.degrees(yaw)))
    imin = min(range(len(rows)), key=lambda i: rows[i][1])
    tmin = rows[imin][0]
    first0 = next((r[0] for r in rows if r[1] <= 0.001), None)
    tref = first0 if first0 is not None else tmin
    name = rec["controller"].replace("FollowPathRLShim", "")
    print(f'## {name} {rec["repeat"]} {rec["status"][:4]} min {rows[imin][1]:.3f} at {tmin:.1f}s first contact {first0 if first0 is None else round(first0, 1)}')
    last = -9
    for r in rows:
        if tref - 4.0 <= r[0] <= tref + 0.6 and r[0] - last >= 0.5:
            last = r[0]
            print(f'   t {r[0]:5.1f} d {r[1]:.3f} bearing {r[2]:7.1f} deg{" MASKED" if masked(r[2]) else "       "} range {r[3]:.3f} robot ({r[4]:.2f},{r[5]:.2f}) yaw {r[6]:6.1f}')
    win = [r for r in rows if tref - 3.0 <= r[0] <= tref]
    near = [r for r in win if r[1] < 0.06]
    if near:
        print(f'   within 3 s before: {len(near)} samples under 0.06 m, of which nearest point in the masked angles: {sum(masked(r[2]) for r in near)}')
PY

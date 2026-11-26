"""pode_rl_controller 의 ClosingGuard (C++) 가 task_spec.ClosingGuard 와 같은 판정을 내는지 견줄 파일을
쓴다: 장면마다 스텝별 레이 (라이다 중심에서의 거리, -180 deg 부터 1 deg 간격), 라이다 원점, base yaw,
시각, 목표 v 와 Python 이 낸 "걸렸는지" (hit), "막았는지" (blocked).

    python make_closing_cases.py <out.json>

Python 은 리셋 뒤 첫 스캔을 기준으로 쓰지 않는다. C++ 시험은 장면의 스텝 1 부터 넣고 (first_step)
스텝 1 부터 견준다: 그러면 두 쪽의 기준 스캔이 모든 스텝에서 같다.
거리와 다가오는 속도가 문턱에 너무 가까운 스텝, 그리고 float32 (torch) 와 float64 (C++) 의 방위 칸
반올림이 갈리는 스텝은 C++ 와 판정이 갈릴 수 있어 만들지 않는다: 그런 스텝이 있으면 멈춘다.
JSON 에 inf/nan 이 없어 돌아오지 않은 레이는 YAML 문자열 (".inf", "-.inf", ".nan") 로 쓴다.
"""

import argparse
import json
import math
import os
import sys

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import task_spec as ts  # noqa: E402

STEP_DT = 0.05
LIDAR_X = -0.017  # base 에서 앞쪽 m
ROOM = (-1.0, -1.0, 1.5, 1.2)
BOX = (0.30, -0.06, 0.42, 0.10)
CORRIDOR = (-1.0, -0.4, 2.0, 0.4)
RANGE_MARGIN = 5e-5  # m; 문턱 거리와 레이의 최소 차이 (레이는 0.1 mm 단위라 문턱과 같은 값만 막는다)
CLOSING_MARGIN = 2e-4  # m/s; 문턱 속도와 다가오는 속도의 최소 차이 (0.1 mm 단위 레이로는 0.0005 m/s 간격)
STATIC_ATOL = 1e-4  # m; float32 와 float64 로 낸 static 의 최대 차이


def enc(x: float):
    """export_policy.enc 와 같다 (그 모듈은 onnx 를 불러 여기서는 쓰지 않는다)."""
    if math.isnan(x):
        return ".nan"
    if math.isinf(x):
        return ".inf" if x > 0 else "-.inf"
    return x


# ---- 시험용 스캔 (test_task_spec._scan 과 같은 식) ----
def scan(origin, yaw, rects=(), circles=(), n_rays=360, far=9.0):
    ang = yaw + torch.deg2rad(-180.0 + torch.arange(n_rays, dtype=torch.float64) * (360.0 / n_rays))
    dx, dy = torch.cos(ang), torch.sin(ang)
    ox, oy = float(origin[0]), float(origin[1])
    best = torch.full((n_rays,), far, dtype=torch.float64)
    miss = torch.full((n_rays,), float("inf"), dtype=torch.float64)
    for x0, y0, x1, y1 in rects:
        tx = torch.stack(((x0 - ox) / dx, (x1 - ox) / dx))
        ty = torch.stack(((y0 - oy) / dy, (y1 - oy) / dy))
        enter = torch.maximum(tx.amin(0), ty.amin(0))
        leave = torch.minimum(tx.amax(0), ty.amax(0))
        hit = leave >= torch.clamp(enter, min=0.0)
        best = torch.minimum(best, torch.where(hit, torch.where(enter > 0.0, enter, leave), miss))
    for cx, cy, r in circles:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - r * r)
        t = -b - torch.sqrt(torch.clamp(disc, min=0.0))
        best = torch.minimum(best, torch.where((disc >= 0.0) & (t > 0.0), t, miss))
    return best.tolist()


def lidar_at(x, y, yaw):
    return (x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw))


# ---- 장면: 스텝별 (라이다 원점, base yaw, 레이, 목표 v) ----
def approach():
    """서 있는 로봇에게 앞 왼쪽에서 사람이 0.11 m/s 로 다가온다."""
    origin, out = lidar_at(0.0, 0.0, 0.0), []
    for i in range(40):
        s = 0.11 * STEP_DT * i
        person = (0.40 - 0.958 * s, 0.12 - 0.287 * s, 0.04)
        out.append((origin, 0.0, scan(origin, 0.0, (ROOM,), (person,)), 0.12))
    return {}, out


def drive_static():
    """서 있는 방과 상자 사이를 가며 돈다: 가까워져도 걸리지 않는다."""
    x, y, yaw, out = 0.0, 0.0, -0.5, []
    for _ in range(32):
        origin = lidar_at(x, y, yaw)
        out.append((origin, yaw, scan(origin, yaw, (ROOM, BOX)), 0.12))
        x, y, yaw = x + 0.006 * math.cos(yaw), y + 0.006 * math.sin(yaw), yaw + 0.04
    return {"stop_range": 0.50, "half_width_deg": 90.0}, out


def crossing_long():
    """긴 창 (1.0 s, +-4 deg, 0.08 m/s): 앞 0.22 m 를 사람이 0.08 m/s 로 가로지른다."""
    origin, out = lidar_at(0.0, 0.0, 0.0), []
    for i in range(50):
        person = (0.22, 0.16 - 0.08 * STEP_DT * i, 0.04)
        out.append((origin, 0.0, scan(origin, 0.0, (CORRIDOR,), (person,)), 0.12))
    params = {"lookback_s": 1.0, "spread_deg": 4.0, "closing_mps": 0.08}
    return params, out


def ramp_holes():
    """정면 7 레이가 0.2 m/s 로 다가오다 선다. 외톨이 레이 하나도 다가오지만 min_rays 3 에는 모자란다.
    돌아오지 않은 레이가 섞이고, 목표 v 는 가끔 후진이나 0 이다."""
    out = []
    for i in range(36):
        rays = [1.0] * 360
        front = 0.405 - 0.01 * min(i, 20)
        for k in range(177, 184):
            rays[k] = front
        rays[150] = 0.405 - 0.01 * min(i, 25)
        for k in range(100, 110):
            rays[k] = math.nan
        for k in range(250, 260):
            rays[k] = math.inf
        for k in range(60, 70):
            rays[k] = -math.inf
        v = -0.05 if i % 5 == 3 else (0.0 if i % 7 == 6 else 0.12)
        out.append(((0.0, 0.0), 0.0, rays, v))
    return {"min_rays": 3}, out


SCENES = {
    "approach": approach,
    "drive_static": drive_static,
    "crossing_long": crossing_long,
    "ramp_holes": ramp_holes,
}


# ---- 기대값과 문턱 여유 확인 ----
def margin_check(name, params, steps, back):
    """C++ (float64) 와 판정이 갈릴 수 있는 스텝이 있으면 멈춘다. steps 는 반올림한 입력."""
    deg = -180.0 + torch.arange(360, dtype=torch.float64)
    front = deg.abs() <= params["half_width_deg"] + 1e-4
    spread = round(params["spread_deg"] * 360 / 360.0)
    stop, thr = params["stop_range"], params["closing_mps"]
    for s in range(1 + back, len(steps)):
        ref, cur = steps[s - back], steps[s]
        static = {}
        for dtype in (torch.float32, torch.float64):
            static[dtype] = ts.static_ranges(
                torch.tensor([ref["rays_in"]], dtype=dtype),
                torch.tensor([ref["origin"]], dtype=dtype),
                torch.tensor([ref["yaw"]], dtype=dtype),
                torch.tensor([cur["origin"]], dtype=dtype),
                torch.tensor([cur["yaw"]], dtype=dtype),
                spread,
            )[0].double()
        rays = torch.tensor(cur["rays_in"], dtype=torch.float32).double()
        closing = (static[torch.float64] - rays) / (cur["stamp"] - ref["stamp"])
        for i in torch.nonzero(front).flatten().tolist():
            r, c = float(rays[i]), float(closing[i])
            a, b = float(static[torch.float32][i]), float(static[torch.float64][i])
            fast = math.isfinite(c) and c > thr - CLOSING_MARGIN
            near = r < stop + RANGE_MARGIN
            if fast and abs(r - stop) < RANGE_MARGIN:
                raise SystemExit(f"{name} step {s} ray {i}: range {r} at the {stop} m bar")
            if near and math.isfinite(c) and abs(c - thr) < CLOSING_MARGIN:
                raise SystemExit(f"{name} step {s} ray {i}: closing {c} at the {thr} m/s bar")
            if near and not (a == b or abs(a - b) < STATIC_ATOL):
                raise SystemExit(f"{name} step {s} ray {i}: static float32 {a} vs float64 {b}")


def run_scene(name, make):
    overrides, raw = make()
    params = {
        "stop_range": 0.26,
        "half_width_deg": 60.0,
        "lookback_s": ts.CLOSING_LOOKBACK_S,
        "spread_deg": ts.CLOSING_SPREAD_DEG,
        "closing_mps": ts.CLOSING_MIN_MPS,
        "hold_s": ts.CLOSING_HOLD_S,
        "min_rays": 1,
        **overrides,
    }
    guard = ts.ClosingGuard(
        STEP_DT,
        params["stop_range"],
        params["half_width_deg"],
        lookback_s=params["lookback_s"],
        spread_deg=params["spread_deg"],
        closing_mps=params["closing_mps"],
        hold_s=params["hold_s"],
        min_rays=params["min_rays"],
    )
    steps = []
    for i, (origin, yaw, rays, v) in enumerate(raw):
        rays_in = [r if not math.isfinite(r) else round(r, 4) for r in rays]
        st = {
            "stamp": round(i * STEP_DT, 6),
            "origin": [round(origin[0], 6), round(origin[1], 6)],
            "yaw": round(yaw, 6),
            "v": v,
            "rays_in": rays_in,
        }
        out, blocked = guard.step(
            torch.tensor([[v, 0.3]]),
            torch.tensor([rays_in], dtype=torch.float32),
            torch.tensor([st["origin"]]),
            torch.tensor([st["yaw"]]),
        )
        st["hit"] = bool(guard.hit[0])
        st["blocked"] = bool(blocked[0])
        st["v_out"] = float(out[0, 0])
        steps.append(st)
    # 스텝 간격이 고르므로 시각으로 고른 C++ 의 기준도 정확히 back 스텝 전이다
    margin_check(name, params, steps, guard.back)
    for st in steps:
        st["rays"] = [enc(r) for r in st.pop("rays_in")]
    hits = sum(st["hit"] for st in steps)
    blocks = sum(st["blocked"] for st in steps)
    print(f"[closing] {name}: {len(steps)} steps, {hits} hits, {blocks} blocked")
    return {"name": name, "params": params, "first_step": 1, "steps": steps}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    args = ap.parse_args()
    scenes = [run_scene(name, make) for name, make in SCENES.items()]
    data = {"step_dt": STEP_DT, "scenes": scenes}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(json.dumps(data, separators=(",", ":")) + "\n")
    print(f"[closing] wrote {args.out} ({os.path.getsize(args.out)} bytes)")


if __name__ == "__main__":
    main()

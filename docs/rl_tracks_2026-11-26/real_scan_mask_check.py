"""실물 bag의 scan에 플러그인과 같은 가림과 레이 변환을 적용해 scan 가림 값을 확인한다.

플러그인(`pode_rl_controller`의 `mask_scan`, `scan_to_rays`, `build_observation`)의 계산을
그대로 옮겼다. dump는 ROS가 있는 곳에서 bag의 scan과 lidar 방향을 npz로 뽑고, check는
numpy만으로 지표를 낸다.

사용:
  python3 real_scan_mask_check.py dump <bag.mcap> <scans.npz> [--topic /pode1/scan]
      [--base-frame base_footprint]
  python real_scan_mask_check.py check <scans.npz> <policy_spec.json> <출력 폴더>
      [--mask "lo,hi,..."]
"""
import argparse
import json
import math
import pathlib

import numpy as np

NARROW_MASK = "140,147,149,155,-150.5,-147.5,-144,-139"
POST_SECTORS = (3, 4, 32, 33)
RAY_SPANS = ((-160.0, -129.0), (138.0, 161.0))
MOVING_V, MOVING_W = 0.02, 0.10
NEAR_M = 0.10
TWO_PI = 2.0 * math.pi
LEAK_WORDS = ("새지 않는다", "가끔 샌다", "자주 샌다")
EMPTY_WORDS = ("거의 없다", "가끔 있다", "자주 있다")


# ---- 플러그인과 같은 계산 ----
def parse_mask(text):
    """'lo,hi,lo,hi' 글을 (lo, hi) 쌍의 목록으로 바꾼다. 빈 글은 가림 없음이다."""
    if not text.strip():
        return []
    v = [float(x) for x in text.split(",")]
    pairs = list(zip(v[0::2], v[1::2], strict=False))
    if len(v) % 2 or not all(-180.0 <= lo < hi <= 180.0 for lo, hi in pairs):
        raise ValueError(f"bad mask: {text}")
    return pairs


def beam_deg(n, angle_min, inc, yaw):
    """빔마다 base 기준 각을 도로 낸다(-180~180)."""
    deg = np.degrees(yaw + angle_min + np.arange(n) * inc)
    return deg - 360.0 * np.round(deg / 360.0)


def masked_beams(deg, mask):
    """가림 범위에 든 빔을 True로 낸다. 바로 뒤는 -180과 180 둘 다로 본다."""
    alt = np.where(deg <= -180.0, 180.0, np.where(deg >= 180.0, -180.0, deg))
    hit = np.zeros(len(deg), dtype=bool)
    for lo, hi in mask:
        hit |= ((deg >= lo) & (deg <= hi)) | ((alt >= lo) & (alt <= hi))
    return hit


def ray_index(n, angle_min, inc, yaw, lid):
    """학습 레이마다 가장 가까운 빔의 번호를 낸다. scan이 덮지 않는 레이는 -1이다."""
    theta = np.radians(lid["first_ray_deg"] + np.arange(lid["rays"]) * lid["ray_step_deg"])
    rel = np.fmod(theta - yaw - angle_min, TWO_PI)
    rel = np.where(rel < 0.0, rel + TWO_PI, rel)
    idx = np.floor(rel / inc + 0.5).astype(np.int64)
    wrap = np.where(TWO_PI - rel <= 0.5 * inc, 0, -1)
    return np.where(idx >= n, wrap, idx)


def ray_values(ranges, idx, range_min, range_max, obs_max):
    """레이 값과 '쓸 수 없는 레이' 표시를 낸다. 쓸 수 없는 레이는 관측 상한으로 읽힌다."""
    r = np.where(idx >= 0, ranges[np.clip(idx, 0, None)], np.inf).astype(np.float64)
    bad = ~np.isfinite(r) | (r < range_min) | (r > range_max)
    return np.where(bad, obs_max, np.minimum(r, obs_max)), bad


def sector_values(ranges, n, angle_min, inc, range_min, range_max, yaw, lid, mask):
    """scan 한 장의 섹터 값(가림 전과 후), 섹터마다 남은 레이 수, 가려진 레이 수를 낸다."""
    shape = (lid["sectors"], lid["rays"] // lid["sectors"])
    ranges = np.asarray(ranges[:n], dtype=np.float64)
    deg = beam_deg(n, angle_min, inc, yaw)
    hit = masked_beams(deg, mask)
    idx = ray_index(n, angle_min, inc, yaw, lid)
    covered = idx >= 0
    ray_hit = covered & hit[np.clip(idx, 0, None)]
    raw, bad = ray_values(ranges, idx, range_min, range_max, lid["obs_max"])
    masked, _ = ray_values(np.where(hit, np.inf, ranges), idx, range_min, range_max, lid["obs_max"])
    return {
        "raw": raw.reshape(shape).min(axis=1),
        "masked": masked.reshape(shape).min(axis=1),
        "left": (covered & ~ray_hit).reshape(shape).sum(axis=1),
        "all_bad": bad.reshape(shape).all(axis=1),
        "usable": (covered & ~ray_hit & ~bad).reshape(shape).any(axis=1),
        "rays_masked": int(ray_hit.sum()),
        "beam_hit": hit,
        "ray": raw,
        "ray_bad": bad,
        "ray_hit": ray_hit,
        "ray_beam_deg": np.where(covered, deg[np.clip(idx, 0, None)], np.nan),
    }


# ---- 지표와 판정 ----
def share(flags):
    """참인 장의 비율을 낸다. 장이 하나도 없으면 None이다."""
    return float(flags.mean()) if len(flags) else None


def grade(frac, words):
    """비율을 미리 정한 문턱(1 %, 10 %)으로 세 낱말 가운데 하나로 바꾼다."""
    return words[0] if frac <= 0.01 else (words[1] if frac <= 0.10 else words[2])


def moving_scans(data):
    """scan마다 가장 가까운 때의 odom 속도로 움직이는 중인지 낸다. odom이 없으면 None이다."""
    if "odom_t" not in data or len(data["odom_t"]) == 0:
        return None
    t, v, w = data["odom_t"], data["odom_v"], data["odom_w"]
    at = np.clip(np.searchsorted(t, data["stamp"]), 1, len(t) - 1)
    at = np.where(np.abs(t[at - 1] - data["stamp"]) <= np.abs(t[at] - data["stamp"]), at - 1, at)
    return (np.abs(v[at]) >= MOVING_V) | (np.abs(w[at]) >= MOVING_W)


def check(npz_path, spec_path, out_dir, mask_text):
    """npz의 scan 모두에 가림을 걸어 M1~M4를 내고 summary.json과 summary.md를 쓴다."""
    data = np.load(npz_path, allow_pickle=False)
    lid = json.loads(pathlib.Path(spec_path).read_text(encoding="utf-8"))["lidar"]
    mask, yaw, obs_max = parse_mask(mask_text), float(data["yaw"]), lid["obs_max"]
    all_ranges, beams, grid_meta = data["ranges"], data["n"], data["meta"]
    count, posts = len(beams), list(POST_SECTORS)
    raw = np.empty((count, lid["sectors"]))
    masked, left = np.empty_like(raw), np.empty(raw.shape, dtype=np.int64)
    all_bad, rays_masked = np.empty(raw.shape, dtype=bool), np.empty(count, dtype=np.int64)
    usable = np.empty(raw.shape, dtype=bool)
    near_in, near_out, short = {}, {}, 0
    grids, first = {}, None
    ray_val = np.empty((count, lid["rays"]))
    ray_bad = np.empty(ray_val.shape, dtype=bool)
    for i in range(count):
        n = int(beams[i])
        amin, amax, inc, rmin, rmax = (float(x) for x in grid_meta[i])
        ranges = all_ranges[i, :n]
        out = sector_values(ranges, n, amin, inc, rmin, rmax, yaw, lid, mask)
        raw[i], masked[i] = out["raw"], out["masked"]
        left[i], all_bad[i] = out["left"], out["all_bad"]
        rays_masked[i], usable[i] = out["rays_masked"], out["usable"]
        ray_val[i], ray_bad[i] = out["ray"], out["ray_bad"]
        first = first or out
        key = (n, round(math.degrees(amin), 3), round(math.degrees(amax), 3),
               round(math.degrees(inc), 4), round(rmin, 4), round(rmax, 3))
        grids[key] = grids.get(key, 0) + 1
        # 0.10 m 아래 반환을 base 기준 1도 칸으로 센다. range_min 아래는 따로 센다
        close = np.isfinite(ranges) & (ranges < NEAR_M)
        short += int((close & (ranges < rmin)).sum())
        deg = np.floor(beam_deg(n, amin, inc, yaw)).astype(int)
        keep = close & (ranges >= rmin)
        for d, inside in zip(deg[keep], out["beam_hit"][keep], strict=True):
            book = near_in if inside else near_out
            book[int(d)] = book.get(int(d), 0) + 1

    top = raw >= obs_max - 1e-9
    top_masked = masked >= obs_max - 1e-9
    others = [s for s in range(lid["sectors"]) if s not in posts]
    min_left = {str(s): int(left[:, s].min()) for s in posts}
    m2 = float((masked[:, posts] < NEAR_M).any(axis=1).mean())
    m3 = float(top_masked[:, posts].any(axis=1).mean())
    res = {
        "scans": count,
        "mask": mask_text,
        "yaw_deg": round(math.degrees(yaw), 3),
        "grids": [{"beams": k[0], "angle_min_deg": k[1], "angle_max_deg": k[2], "step_deg": k[3],
                   "range_min": k[4], "range_max": k[5], "scans": v}
                  for k, v in sorted(grids.items(), key=lambda kv: -kv[1])],
        "rays_masked": {"min": int(rays_masked.min()), "median": float(np.median(rays_masked)),
                        "max": int(rays_masked.max())},
        "m1": {
            "min_left": min_left,
            "median_left": {str(s): float(np.median(left[:, s])) for s in posts},
            "scans_with_empty_sector": int((left[:, posts] == 0).any(axis=1).sum()),
            "min_left_any_sector": int(left.min()),
            "verdict": ("실물 scan 격자에서 섹터마다 레이가 남는다" if min(min_left.values()) >= 1
                        else "남지 않는 섹터가 있다: 이 값을 실물에 쓰면 안 된다"),
        },
        "m2": {"masked": m2, "raw": float((raw[:, posts] < NEAR_M).any(axis=1).mean()),
               "per_sector_masked": {str(s): float((masked[:, s] < NEAR_M).mean()) for s in posts},
               "per_sector_raw": {str(s): float((raw[:, s] < NEAR_M).mean()) for s in posts},
               "verdict": grade(m2, LEAK_WORDS)},
        "m3": {"masked": m3, "raw": float(top[:, posts].any(axis=1).mean()),
               "per_sector_masked": {str(s): float(top_masked[:, s].mean()) for s in posts},
               "per_sector_raw": {str(s): float(top[:, s].mean()) for s in posts},
               "no_usable": float((top_masked[:, posts] & ~usable[:, posts]).any(axis=1).mean()),
               "per_sector_no_usable": {
                   str(s): float((top_masked[:, s] & ~usable[:, s]).mean()) for s in posts},
               "verdict": grade(m3, EMPTY_WORDS)},
        "m4": {
            "other_sector_top": {str(s): float(top[:, s].mean()) for s in others},
            "other_sector_top_all_bad": {
                str(s): (float((top[:, s] & all_bad[:, s]).sum() / top[:, s].sum())
                         if top[:, s].any() else None) for s in others},
            "other_any_top": float(top[:, others].any(axis=1).mean()),
            "other_top_mean": float(top[:, others].mean()),
        },
        "post_value_masked_m": {
            str(s): [round(float(x), 3) for x in np.percentile(masked[:, s], [5, 50, 95])]
            for s in posts},
        "rays": [],
        "motion": {},
        "near": {"below_range_min": short, "inside_mask": int(sum(near_in.values())),
                 "outside_mask": int(sum(near_out.values())),
                 "inside_by_deg": {str(k): near_in[k] for k in sorted(near_in)},
                 "outside_by_deg": {str(k): near_out[k] for k in sorted(near_out)}},
    }
    # odom이 있으면 움직이는 동안과 서 있는 동안으로 갈라 같은 비율을 적는다
    moving = moving_scans(data)
    if moving is not None:
        for name, sel in (("moving", moving), ("stopped", ~moving)):
            res["motion"][name] = {
                "scans": int(sel.sum()),
                "m2_masked": share((masked[sel][:, posts] < NEAR_M).any(axis=1)),
                "m3_masked": share(top_masked[sel][:, posts].any(axis=1)),
                "m3_per_sector_masked": {str(s): share(top_masked[sel][:, s]) for s in posts},
                "m2_raw": share((raw[sel][:, posts] < NEAR_M).any(axis=1)),
            }
    # 기둥 둘레의 학습 레이마다 가림 전의 값을 적는다. 고른 빔과 가림 여부는 첫 장의 격자로 본다
    for k in range(lid["rays"]):
        ray_deg = lid["first_ray_deg"] + k * lid["ray_step_deg"]
        if not any(lo <= ray_deg <= hi for lo, hi in RAY_SPANS):
            continue
        good = ray_val[~ray_bad[:, k], k]
        res["rays"].append({
            "ray_deg": ray_deg,
            "beam_deg": round(float(first["ray_beam_deg"][k]), 2),
            "masked": bool(first["ray_hit"][k]),
            "valid": float((~ray_bad[:, k]).mean()),
            "range_m": ([round(float(x), 3) for x in np.percentile(good, [5, 50, 95])]
                        if len(good) else None),
            "near": float((good < NEAR_M).mean()) if len(good) else None,
        })
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(
        json.dumps(res, ensure_ascii=False, indent=1) + "\n", encoding="utf-8", newline="\n")
    (out_dir / "summary.md").write_text(markdown(res), encoding="utf-8", newline="\n")
    return res


def pct(x):
    """비율을 퍼센트 글로 바꾼다."""
    return "없음" if x is None else f"{100.0 * x:.2f} %"


def row(label, cells, last=""):
    """표 한 줄을 만든다."""
    return "| " + " | ".join([label, *cells, last]) + " |"


def by_deg(book):
    """각도 칸마다의 개수를 한 줄 글로 바꾼다."""
    return ", ".join(f"{k}°: {v}" for k, v in book.items()) or "없음"


def markdown(res):
    """결과를 사람이 읽는 표로 바꾼다."""
    posts = [str(s) for s in POST_SECTORS]
    lines = [
        "# 실물 scan에 건 scan 가림 확인", "",
        f"scan {res['scans']}장, 가림 `{res['mask']}`, lidar yaw {res['yaw_deg']}°.",
        f"가려진 학습 레이 수(장마다): 최소 {res['rays_masked']['min']}, 중앙값 "
        f"{res['rays_masked']['median']:.0f}, 최대 {res['rays_masked']['max']}.", "",
        "## scan 격자", "",
        "| 빔 수 | 각 범위 ° | 간격 ° | range_min m | range_max m | 장 수 |",
        "|---|---|---|---|---|---|"]
    for g in res["grids"][:8]:
        span = f"{g['angle_min_deg']} ~ {g['angle_max_deg']}"
        lines.append(row(str(g["beams"]), [span, str(g["step_deg"]), str(g["range_min"]),
                                           str(g["range_max"])], str(g["scans"])))
    if len(res["grids"]) > 8:
        lines.append(f"| (그 밖의 격자 {len(res['grids']) - 8}가지) | | | | | "
                     f"{sum(g['scans'] for g in res['grids'][8:])} |")
    m1, m2, m3, m4 = res["m1"], res["m2"], res["m3"], res["m4"]

    def cells(book):
        return [pct(book[s]) for s in posts]

    lines += [
        "", "## 기둥 섹터 (3, 4, 32, 33번)", "",
        "| 지표 | 3 | 4 | 32 | 33 | 넷 가운데 하나라도 |", "|---|---|---|---|---|---|",
        row("M1 남은 레이 수의 최솟값", [str(m1["min_left"][s]) for s in posts],
            f"빈 섹터가 있는 장 {m1['scans_with_empty_sector']}"),
        row("M1 남은 레이 수의 중앙값", [f"{m1['median_left'][s]:.0f}" for s in posts]),
        row("M2 가림 뒤 0.10 m 아래로 읽힌 장", cells(m2["per_sector_masked"]), pct(m2["masked"])),
        row("(가림 없이) 0.10 m 아래로 읽힌 장", cells(m2["per_sector_raw"]), pct(m2["raw"])),
        row("M3 가림 뒤 2.0 m로 읽힌 장", cells(m3["per_sector_masked"]), pct(m3["masked"])),
        row("M3 가운데 가리지 않은 레이의 반환이 하나도 없던 장", cells(m3["per_sector_no_usable"]),
            pct(m3["no_usable"])),
        row("(가림 없이) 2.0 m로 읽힌 장", cells(m3["per_sector_raw"]), pct(m3["raw"])),
        row("가림 뒤 섹터 값 m (5 %, 중앙값, 95 %)",
            [", ".join(f"{x:.2f}" for x in res["post_value_masked_m"][s]) for s in posts]),
        "",
        f"- M1: {m1['verdict']} (36개 섹터 전체에서 남은 레이 수의 최솟값 "
        f"{m1['min_left_any_sector']})",
        f"- M2: {m2['verdict']}", f"- M3: {m3['verdict']}", "",
        "## 그 밖의 섹터 32개 (가림 없이)", "",
        f"2.0 m로 읽힌 비율: 섹터·장 평균 {pct(m4['other_top_mean'])}, 하나라도 그런 장 "
        f"{pct(m4['other_any_top'])}.", "",
        "| 섹터 | 2.0 m로 읽힌 장 | 그 가운데 레이 열 개가 모두 쓸 수 없던 장 |",
        "|---|---|---|"]
    for s, frac in m4["other_sector_top"].items():
        lines.append(f"| {s} | {pct(frac)} | {pct(m4['other_sector_top_all_bad'][s])} |")
    if res["motion"]:
        lines += [
            "", "## 움직이는 동안과 서 있는 동안", "",
            f"odom의 |v|가 {MOVING_V} m/s 이상이거나 |w|가 {MOVING_W} rad/s 이상이면 "
            "움직이는 장이다.", "",
            "| | 장 수 | M2 가림 뒤 0.10 m 아래 | M3 가림 뒤 2.0 m | M3 섹터 3, 4, 32, 33 "
            "| 가림 없이 0.10 m 아래 |",
            "|---|---|---|---|---|---|"]
        for name, label in (("moving", "움직이는 동안"), ("stopped", "서 있는 동안")):
            g = res["motion"][name]
            per = ", ".join(pct(g["m3_per_sector_masked"][s]) for s in posts)
            cells = [str(g["scans"]), pct(g["m2_masked"]), pct(g["m3_masked"]), per]
            lines.append(row(label, cells, pct(g["m2_raw"])))
    lines += [
        "", "## 기둥 둘레의 학습 레이 (가림 전의 값, 고른 빔과 가림은 첫 장의 격자)", "",
        "| 레이 ° | 고른 빔 ° | 가림 | 쓸 수 있던 장 | 거리 m (5 %, 중앙값, 95 %) | 0.10 m 아래 |",
        "|---|---|---|---|---|---|"]
    for r in res["rays"]:
        dist = "없음" if r["range_m"] is None else ", ".join(f"{x:.3f}" for x in r["range_m"])
        mark = "가림" if r["masked"] else ""
        cells = [f"{r['beam_deg']:.2f}", mark, pct(r["valid"]), dist]
        lines.append(row(f"{r['ray_deg']:.0f}", cells, pct(r["near"])))
    near = res["near"]
    lines += [
        "", "## 0.10 m 아래 반환 (모든 빔, base 기준 1도 칸)", "",
        f"가림 안 {near['inside_mask']}개, 가림 밖 {near['outside_mask']}개, range_min 아래라 "
        f"플러그인이 버리는 것 {near['below_range_min']}개.", "",
        "가림 안: " + by_deg(near["inside_by_deg"]), "",
        "가림 밖: " + by_deg(near["outside_by_deg"]), ""]
    return "\n".join(lines)


# ---- bag에서 scan 뽑기 ----
def chain_yaw(static, frame, base):
    """정적 TF를 따라 올라가 base 기준 lidar의 yaw와 지나간 틀 이름을 낸다."""
    def mul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)

    q, path = (0.0, 0.0, 0.0, 1.0), [frame]
    while not (frame == base or frame.endswith("/" + base)):
        if frame not in static:
            raise SystemExit(f"no static transform from {path[0]} up to {base}: {path}")
        frame, rot = static[frame]
        q = mul(rot, q)
        path.append(frame)
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)), path


def dump(bag, out, topic, base):
    """bag의 scan(거리, 격자, 시각)과 정적 TF에서 읽은 lidar yaw를 npz로 쓴다."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    tf_topic = topic.rsplit("/", 1)[0] + "/tf_static"
    odom_topic = topic.rsplit("/", 1)[0] + "/odom"
    rows, meta, stamps, frames, static, odom = [], [], [], set(), {}, []
    while reader.has_next():
        name, raw, stamp = reader.read_next()
        if name == topic:
            msg = deserialize_message(raw, LaserScan)
            rows.append(np.asarray(msg.ranges, dtype=np.float32))
            meta.append([msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min,
                         msg.range_max])
            stamps.append(stamp)
            frames.add(msg.header.frame_id)
        elif name == odom_topic:
            twist = deserialize_message(raw, Odometry).twist.twist
            odom.append([stamp, twist.linear.x, twist.angular.z])
        elif name == tf_topic:
            for tr in deserialize_message(raw, TFMessage).transforms:
                q = tr.transform.rotation
                static[tr.child_frame_id] = (tr.header.frame_id, (q.x, q.y, q.z, q.w))
    if len(frames) != 1:
        raise SystemExit(f"expected one scan frame, got {sorted(frames)}")
    yaw, path = chain_yaw(static, next(iter(frames)), base)
    width = max(len(r) for r in rows)
    ranges = np.full((len(rows), width), np.nan, dtype=np.float32)
    for i, r in enumerate(rows):
        ranges[i, :len(r)] = r
    np.savez_compressed(
        out, ranges=ranges, n=np.array([len(r) for r in rows]),
        meta=np.array(meta, dtype=np.float64), stamp=np.array(stamps), yaw=np.array(yaw),
        frames=np.array(json.dumps(path)),
        odom_t=np.array([o[0] for o in odom], dtype=np.int64),
        odom_v=np.array([o[1] for o in odom]), odom_w=np.array([o[2] for o in odom]))
    print(f"scans {len(rows)}, beams {min(map(len, rows))}..{width}, "
          f"yaw {math.degrees(yaw):.3f} deg, tf {path}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dump")
    d.add_argument("bag")
    d.add_argument("out")
    d.add_argument("--topic", default="/pode1/scan")
    d.add_argument("--base-frame", default="base_footprint")
    c = sub.add_parser("check")
    c.add_argument("npz")
    c.add_argument("spec")
    c.add_argument("out_dir")
    c.add_argument("--mask", default=NARROW_MASK)
    a = ap.parse_args()
    if a.cmd == "dump":
        dump(a.bag, a.out, a.topic, a.base_frame)
    else:
        print(markdown(check(a.npz, a.spec, a.out_dir, a.mask)))


if __name__ == "__main__":
    main()

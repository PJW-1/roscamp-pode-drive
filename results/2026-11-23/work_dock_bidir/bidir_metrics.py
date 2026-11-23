"""WORK 양방향 진입 비교 지표(2026-11-23). Gazebo 묶음 폴더(판마다 result/, truth.csv)를 읽는다.

사용: python3 bidir_metrics.py 이름=묶음폴더 ...  (표를 마크다운으로, 판별 값을 JSON으로 낸다)
지표:
- 처리량: 실은 주문 수 / duration_s x 3600
- W 체류: 실제 위치가 W 0.05 m 안에 든 때부터 벗어난 때까지(나가기 전 제자리 돌기 포함)
- W 앞 대기: WORK DOCK_QUEUED부터 그 로봇의 "WORK_DOCK ... to W" 허가까지
- 제자리 회전: W 0.35 m, M_E 0.12 m, Q1 0.12 m 안에서 속도 < 0.03 m/s, 각속도 > 0.3 rad/s인 시간
- 마주 봄: 두 로봇 중심 0.30 m 안, 방향 차 150도 넘게, 서로 다가감(따로 1 s 넘게 떨어진 것을 한 번으로)
- 조기 해제: WORK_DOCK 해제 때 보고 위치 거리 < 0.135(노드 좌표 지름길이 쓰였다), 실제 거리 < 0.08(감시 한계)
- 들어간 쪽: W에 들어선 때의 실제 방향(북쪽을 보면 남쪽 진입, 남쪽을 보면 북쪽 진입)
"""
import csv
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

W = (1.86, 0.495)
M_E = (1.645, 0.55)
Q1 = (1.8491, 0.26)
AT_W_M = 0.05
NEAR = ((W, 0.35), (M_E, 0.12), (Q1, 0.12))
FACE_M, FACE_DEG, FACE_GAP_S = 0.30, 150.0, 1.0


def truth(path: Path) -> dict:
    rows = defaultdict(list)
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows[r["robot"]].append((float(r["sim_t"]), float(r["x"]), float(r["y"]),
                                     float(r["yaw"])))
    for robot in rows:
        rows[robot].sort()
    return rows


def events(path: Path) -> list:
    with open(path, encoding="utf-8") as f:
        return [(float(r["time_s"]), r["source"], r["code"], r["detail"]) for r in csv.DictReader(f)]


def w_visits(track) -> list:
    """(들어선 시각, 떠난 시각, 들어설 때 방향)."""
    out, start, yaw0 = [], None, 0.0
    for t, x, y, yaw in track:
        inside = math.dist((x, y), W) <= AT_W_M
        if inside and start is None:
            start, yaw0 = t, yaw
        elif not inside and start is not None:
            out.append((start, t, yaw0))
            start = None
    return out


def spin_s(track) -> float:
    total = 0.0
    for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(track, track[1:], strict=False):
        dt = t1 - t0
        if dt <= 0.0 or dt > 0.5:
            continue
        if not any(math.dist((x0, y0), p) <= r for p, r in NEAR):
            continue
        v = math.dist((x0, y0), (x1, y1)) / dt
        w = abs(math.remainder(a1 - a0, math.tau)) / dt
        if v < 0.03 and w > 0.3:
            total += dt
    return total


def face_to_face(tracks) -> tuple[int, int]:
    """(전체, W 동쪽 끝 x > 1.5) 마주 봄 횟수."""
    names = sorted(tracks)
    grid = {name: {round(t, 1): (x, y, a) for t, x, y, a in tracks[name]} for name in names}
    total = near_w = 0
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            last, prev = -1e9, None
            for t in sorted(set(grid[a]) & set(grid[b])):
                (ax, ay, aa), (bx, by, ba) = grid[a][t], grid[b][t]
                gap = math.dist((ax, ay), (bx, by))
                opposed = abs(math.degrees(math.remainder(aa - ba, math.tau))) > FACE_DEG
                closing = prev is not None and gap < prev - 1e-4
                prev = gap
                if gap < FACE_M and opposed and closing:
                    if t - last > FACE_GAP_S:
                        total += 1
                        near_w += (ax + bx) / 2 > 1.5
                    last = t
    return total, near_w


def one_run(run: Path) -> dict:
    res = run / "result"
    summary = json.loads((res / "summary.json").read_text(encoding="utf-8"))
    evs = events(res / "events.csv")
    tracks = truth(run / "truth.csv")
    out = {"run": run.name, "pass": summary["pass"], "shipped": summary["orders_shipped"],
           "total": summary["orders_total"], "duration_s": summary["duration_s"]}
    out["orders_per_h"] = summary["orders_shipped"] / summary["duration_s"] * 3600.0
    out["routes"] = sum(1 for e in evs if e[2] == "ORDER_SENT")
    visits = [v for track in tracks.values() for v in w_visits(track)]
    out["w_visits"] = len(visits)
    out["w_dwell_s"] = [round(b - a, 2) for a, b, _ in visits]
    out["north_entries"] = sum(1 for _, _, yaw in visits if math.sin(yaw) < -0.5)
    out["south_entries"] = sum(1 for _, _, yaw in visits if math.sin(yaw) > 0.5)
    waits, queued = [], {}
    for t, robot, code, detail in evs:
        if code == "DOCK_QUEUED" and detail.startswith("WORK is busy") and robot not in queued:
            queued[robot] = t
        elif code == "ZONE_GRANTED" and detail.endswith("to W") and "WORK_DOCK" in detail:
            if robot in queued:
                waits.append(round(t - queued.pop(robot), 2))
    out["w_queue_s"] = waits
    out["queued_at"] = {}
    for t, robot, code, detail in evs:
        if code == "DOCK_QUEUED" and detail.startswith("WORK is busy"):
            spot = detail.rsplit(" ", 1)[-1]
            out["queued_at"][spot] = out["queued_at"].get(spot, 0) + 1
    out["spin_near_w_s"] = round(sum(spin_s(track) for track in tracks.values()), 2)
    codes = defaultdict(int)
    for _, _, code, _ in evs:
        codes[code] += 1
    out["blocked"] = sum(n for code, n in codes.items() if "BLOCK" in code)
    out["deadlock_seen"] = codes.get("DEADLOCK_SEEN", 0)
    out["isolated"] = codes.get("ROBOT_ISOLATED", 0)
    out["zone_wait_warn"] = sum(1 for e in evs if e[2] == "ZONE_WAIT" and "has waited" in e[3])
    out["face_to_face"], out["face_to_face_east"] = face_to_face(tracks)
    early_report = early_true = 0
    releases = []
    with open(res / "releases.csv", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["zone"] != "WORK_DOCK":
                continue
            true_m, rep_m = float(r["true_m"]), float(r["reported_m"])
            releases.append((round(true_m, 3), round(rep_m, 3)))
            early_report += rep_m < 0.135
            early_true += true_m < 0.08
    out["work_dock_releases"] = len(releases)
    out["early_release_reported_lt_0135"] = early_report
    out["early_release_true_lt_008"] = early_true
    out["min_work_dock_release_true_m"] = min((a for a, _ in releases), default=None)
    out["violations"] = summary.get("violations", [])
    # 두 로봇 중심 사이 최소 거리: 판이 끝날 때 쓴 fleet_check.json(로그의 주기 출력은 판 끝 전에 멈출 수 있다)
    check = res / "fleet_check.json"
    if check.is_file():
        out["min_robot_gap_m"] = json.loads(check.read_text(encoding="utf-8")).get("min_distance_m")
    else:
        gaps = re.findall(r"min distance ([0-9.]+) m", (run / "launch.out").read_text(errors="replace"))
        out["min_robot_gap_m"] = float(gaps[-1]) if gaps else None
    return out


def mean(values) -> float:
    values = list(values)
    return sum(values) / len(values) if values else float("nan")


def table(groups: dict) -> str:
    rows = [("판 수(통과)", lambda rs: f"{len(rs)} ({sum(r['pass'] for r in rs)})"),
            ("실은 주문 / 전체", lambda rs: f"{sum(r['shipped'] for r in rs)} / "
                                       f"{sum(r['total'] for r in rs)}"),
            ("걸린 시간 합(s)", lambda rs: f"{sum(r['duration_s'] for r in rs):.1f}"),
            ("시간당 처리 주문(합)", lambda rs: f"{sum(r['shipped'] for r in rs) / sum(r['duration_s'] for r in rs) * 3600:.1f}"),
            ("Route 수(ORDER_SENT)", lambda rs: str(sum(r["routes"] for r in rs))),
            ("W 방문", lambda rs: str(sum(r["w_visits"] for r in rs))),
            ("W 체류 평균 / 최대(s)", lambda rs: f"{mean(d for r in rs for d in r['w_dwell_s']):.1f} / "
                                          f"{max((d for r in rs for d in r['w_dwell_s']), default=0):.1f}"),
            ("W 앞 대기 평균 / 합(s)", lambda rs: f"{mean(d for r in rs for d in r['w_queue_s']):.1f} / "
                                           f"{sum(d for r in rs for d in r['w_queue_s']):.1f}"),
            ("제자리 회전 W 앞뒤 합(s)", lambda rs: f"{sum(r['spin_near_w_s'] for r in rs):.1f}"),
            ("진입 남 / 북", lambda rs: f"{sum(r['south_entries'] for r in rs)} / "
                                     f"{sum(r['north_entries'] for r in rs)}"),
            ("BLOCKED 계열 이벤트", lambda rs: str(sum(r["blocked"] for r in rs))),
            ("DEADLOCK_SEEN / 격리 / 대기 경고", lambda rs: f"{sum(r['deadlock_seen'] for r in rs)} / "
                                                   f"{sum(r['isolated'] for r in rs)} / "
                                                   f"{sum(r['zone_wait_warn'] for r in rs)}"),
            ("마주 봄(전체 / x>1.5)", lambda rs: f"{sum(r['face_to_face'] for r in rs)} / "
                                            f"{sum(r['face_to_face_east'] for r in rs)}"),
            ("WORK_DOCK 해제", lambda rs: str(sum(r["work_dock_releases"] for r in rs))),
            ("조기 해제: 보고 < 0.135 / 실제 < 0.08", lambda rs: f"{sum(r['early_release_reported_lt_0135'] for r in rs)} / "
                                                    f"{sum(r['early_release_true_lt_008'] for r in rs)}"),
            ("WORK_DOCK 해제 실제 거리 최소(m)", lambda rs: f"{min(r['min_work_dock_release_true_m'] for r in rs if r['min_work_dock_release_true_m'] is not None):.3f}"),
            ("로봇 중심 최소 거리(m, fleet_check)", lambda rs: f"{min(r['min_robot_gap_m'] for r in rs if r['min_robot_gap_m'] is not None):.3f}"),
            ("실패 판", lambda rs: ", ".join(r["run"] for r in rs if not r["pass"]) or "없음")]
    names = list(groups)
    lines = ["| 지표 | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for label, f in rows:
        lines.append(f"| {label} | " + " | ".join(f(groups[n]) for n in names) + " |")
    return "\n".join(lines)


def main(argv) -> None:
    groups = {}
    for arg in argv:
        name, folder = arg.split("=", 1)
        runs = sorted(p for p in Path(folder).iterdir() if (p / "result" / "summary.json").is_file())
        groups[name] = [one_run(run) for run in runs]
    print(table(groups))
    print()
    print(json.dumps(groups, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])

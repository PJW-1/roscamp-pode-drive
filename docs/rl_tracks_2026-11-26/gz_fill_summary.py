"""Gazebo 채우기 묶음(35절, 37절)의 결과 폴더에서 갈래별 셈을 낸다.

사용: python gz_fill_summary.py <결과 폴더> [<결과 폴더> ...]
폴더마다 legs.csv, closest.txt, cart_bearing.txt를 읽는다. 폴더를 여럿 주면 끝에 합친 셈과
대조·채우기 성공 수의 Fisher 정확 검정 양쪽 p값을 같이 낸다.
"""
import csv
import pathlib
import re
import sys
from collections import defaultdict
from math import comb

NARROW_M = 0.03  # 통로 카트와의 최소 거리가 이보다 작으면 "좁은 길"
CONTACT_M = 0.001
HEAD_RE = r"## (\w+) (\d+) \w+ min [\d.]+ at [\d.]+s first contact ([\d.]+|None)"
ARMS = {"FollowPathRLShimDemoHist": "대조", "FollowPathRLShimDemoHistMask": "가림",
        "FollowPathRLShimDemoHistFill": "채우기"}


# ---- 읽기 ----
def read_folder(folder):
    """g_demo 판마다 (갈래, 반복) -> 값 사전."""
    folder = pathlib.Path(folder)
    runs = {}
    with open(folder / "legs.csv", encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            if r["scenario"] != "g_demo":
                continue
            runs[(r["controller"], int(r["repeat"]))] = {
                "ok": r["ok"] == "True", "status": r["status"], "time_s": float(r["time_s"])}
    for line in (folder / "closest.txt").read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 5 or parts[0] != "g_demo":
            continue
        key = ("FollowPath" + parts[1], int(parts[2]))
        cart = re.search(r"\bdemo_cart (\d+\.\d+)@", line)
        person = re.search(r"\bperson (\d+\.\d+)@", line)
        runs[key]["cart_m"] = float(cart.group(1)) if cart else float("inf")
        runs[key]["person_m"] = float(person.group(1)) if person else float("inf")
    key = None
    for line in (folder / "cart_bearing.txt").read_text(encoding="utf-8").splitlines():
        head = re.match(HEAD_RE, line)
        if head:
            key = ("FollowPathRLShim" + head.group(1), int(head.group(2)))
            runs[key]["contact_t"] = None if head.group(3) == "None" else float(head.group(3))
            continue
        row = re.match(r"\s+t\s+([\d.]+) d ([\d.]+) bearing\s+(-?[\d.]+) deg", line)
        if not (row and key) or runs[key].get("contact_t") is None or "contact_deg" in runs[key]:
            continue
        if float(row.group(2)) <= CONTACT_M:
            runs[key]["contact_deg"] = float(row.group(3))
    return runs


# ---- 셈 ----
def fisher_two_sided(a, b, c, d):
    """2x2 표 [[a, b], [c, d]]의 Fisher 정확 검정 양쪽 p값."""
    n1, n2, k = a + b, c + d, a + c
    total = comb(n1 + n2, k)
    p0 = comb(n1, a) * comb(n2, c) / total
    p = 0.0
    for x in range(max(0, k - n2), min(n1, k) + 1):
        px = comb(n1, x) * comb(n2, k - x) / total
        if px <= p0 + 1e-12:
            p += px
    return p


def summarize(runs):
    by_arm = defaultdict(list)
    for (arm, _rep), v in sorted(runs.items()):
        by_arm[arm].append(v)
    out = {}
    for arm, rows in by_arm.items():
        narrow = [v for v in rows if v["cart_m"] < NARROW_M]
        touch = [v for v in rows if v["cart_m"] <= CONTACT_M]
        out[arm] = {
            "n": len(rows), "ok": sum(v["ok"] for v in rows),
            "aborted": sum(v["status"] == "ABORTED" for v in rows),
            "cart_touch": len(touch), "person_touch": sum(v["person_m"] <= CONTACT_M for v in rows),
            "narrow": len(narrow), "narrow_touch": sum(v["cart_m"] <= CONTACT_M for v in narrow),
            "contact_deg": sorted(round(v["contact_deg"]) for v in touch if "contact_deg" in v),
            "ok_time": sorted(round(v["time_s"], 1) for v in rows if v["ok"]),
            "cart_m": sorted(round(v["cart_m"], 3) for v in rows),
        }
    return out


def show(title, summary):
    print(f"== {title}")
    for arm, name in ARMS.items():
        s = summary.get(arm)
        if not s:
            continue
        print(f"{name}: 성공 {s['ok']}/{s['n']}, ABORTED {s['aborted']}, "
              f"통로 카트에 닿은 판 {s['cart_touch']}, 사람과 닿은 판 {s['person_touch']}, "
              f"좁은 길 {s['narrow']}판(그 가운데 닿은 판 {s['narrow_touch']})")
        print(f"    닿은 점의 방위(도) {s['contact_deg']}, 성공한 판의 시간 {s['ok_time']}")
        print(f"    통로 카트와의 최소 거리 {s['cart_m']}")
    c, f = summary.get("FollowPathRLShimDemoHist"), summary.get("FollowPathRLShimDemoHistFill")
    if c and f:
        p = fisher_two_sided(c["ok"], c["n"] - c["ok"], f["ok"], f["n"] - f["ok"])
        print(f"대조 {c['ok']}/{c['n']} 대 채우기 {f['ok']}/{f['n']}: Fisher 양쪽 p {p:.3f}")


def main():
    merged = {}
    for i, folder in enumerate(sys.argv[1:]):
        runs = read_folder(folder)
        show(folder, summarize(runs))
        merged.update({(arm, (i, rep)): v for (arm, rep), v in runs.items()})
    if len(sys.argv) > 2:
        show("합친 값", summarize(merged))


if __name__ == "__main__":
    main()

"""Gazebo 열일곱째 묶음(작업 기록 39절)의 관문 숫자를 두 세션 폴더에서 센다. 읽기만 한다.

사용: python gz_gate17.py <세션 a 폴더> <세션 b 폴더>
세션 폴더에는 벤치 표 `legs.csv`와 만남 기록 `meet.txt`(wsl_meet.sh의 출력)가 있어야 한다.
충돌 판은 벤치의 `collisions`가 1 이상인 판이고, 도착은 벤치의 `ok`다(13절, 17절과 같은 뜻).
"""
import collections
import csv
import pathlib
import re
import sys

from gz_fill_summary import fisher_two_sided

# ---- 이름 ----
SYM, CTRL = "FollowPathRLShimDemoHistSym", "FollowPathRLShimDemoHist"
SHORT = {CTRL: "t_demo_hist", SYM: "t_demo_hist_sym"}
SCENARIOS = ("f_cart", "g_demo", "k_cross", "j_cross")
GAP = 4  # 합계 문턱(판). 39절에 미리 적은 값
F_CART_RUNS, F_CART_CORRIDOR = 5, 4  # f_cart 조건: 계획한 5판이 모두 있고 통로 판 4판 이상


# ---- 읽기 ----
def read(folders):
    """두 세션의 판 줄과 만남 줄을 합쳐 돌려준다."""
    rows, meets = [], []
    for folder in folders:
        folder = pathlib.Path(folder)
        with open(folder / "legs.csv", encoding="utf-8", newline="") as f:
            rows += list(csv.DictReader(f))
        meets += (folder / "meet.txt").read_text(encoding="utf-8").splitlines()
    return rows, meets


# ---- 셈 ----
def per_cell(rows):
    """(시나리오, 컨트롤러)마다 판 수, 도착, 통로 판, 충돌 판과 판별 값."""
    g = collections.defaultdict(list)
    for r in rows:
        g[(r["scenario"], r["controller"])].append(r)
    out = {}
    for key, rs in g.items():
        rs = sorted(rs, key=lambda r: int(r["repeat"]))
        out[key] = {
            "n": len(rs),
            "ok": sum(r["ok"] == "True" for r in rs),
            "plan": sum(r["plan_in_band"] == "True" for r in rs),
            "truth": sum(r["truth_in_band"] == "True" for r in rs),
            "corridor": sum(r["in_corridor"] == "True" for r in rs),
            "col": sum(int(float(r["collisions"] or 0)) > 0 for r in rs),
            "time": [float(r["time_s"]) for r in rs],
            "body": [float(r["body_min_m"]) for r in rs],
        }
    return out


def meet_counts(meets):
    """(시나리오, 컨트롤러 짧은 이름)마다 판 수, 만남 판, 만나고 닿지 않고 도착한 판,
    만나고 닿은 판을 센다."""
    m = collections.defaultdict(lambda: [0, 0, 0, 0])
    for line in meets:
        p = line.split()
        if len(p) < 10 or p[0] not in SCENARIOS:
            continue
        ok = re.search(r"\bok (True|False)", line).group(1) == "True"
        met = re.search(r"\bmeet (True|False)", line).group(1) == "True"
        touch = "first contact" in line
        v = m[(p[0], p[1])]
        v[0] += 1
        v[1] += met
        v[2] += met and ok and not touch
        v[3] += met and touch
    return m


def gate(cells):
    """39절의 판정: f_cart 조건, 합계 판정, 저장소에 넣는가."""
    f = cells[("f_cart", SYM)]
    f_ok = f["n"] == F_CART_RUNS and f["corridor"] >= F_CART_CORRIDOR and f["col"] == 0
    tot = {c: [sum(cells[(s, c)][k] for s in SCENARIOS) for k in ("n", "ok", "col")]
           for c in (CTRL, SYM)}
    d_arr, d_col = tot[SYM][1] - tot[CTRL][1], tot[SYM][2] - tot[CTRL][2]
    if d_arr > GAP and not d_col > GAP:
        verdict = "Gazebo에서도 더 낫다"
    elif d_arr < -GAP or d_col > GAP:
        verdict = "Gazebo에서는 나빠졌다"
    else:
        verdict = "이 판 수로는 차이가 보이지 않는다"
    return f_ok, tot, d_arr, d_col, verdict


# ---- 출력 ----
def main():
    rows, meets = read(sys.argv[1:3])
    cells = per_cell(rows)
    print("| 시나리오 | 컨트롤러 | 도착 | 계획 경로 통로 안 | 궤적 통로 안 | 통로 판 | 충돌 판"
          " | 시간 s | 몸 여유 m |")
    print("|---|---|---|---|---|---|---|---|---|")
    for s in SCENARIOS:
        for c in (CTRL, SYM):
            v = cells[(s, c)]
            n = v["n"]
            print(f"| {s} | {SHORT[c]} | {v['ok']}/{n} | {v['plan']}/{n} | {v['truth']}/{n} | "
                  f"{v['corridor']}/{n} | {v['col']} | " + ", ".join(f"{t:.1f}" for t in v["time"])
                  + " | " + ", ".join(f"{b:.3f}" for b in v["body"]) + " |")
    print()
    f_ok, tot, d_arr, d_col, verdict = gate(cells)
    for c in (CTRL, SYM):
        print(f"합계 {SHORT[c]}: 판 {tot[c][0]}, 도착 {tot[c][1]}, 충돌 판 {tot[c][2]}")
    a, b = cells[("g_demo", SYM)], cells[("g_demo", CTRL)]
    p_ok = fisher_two_sided(a["ok"], a["n"] - a["ok"], b["ok"], b["n"] - b["ok"])
    p_col = fisher_two_sided(a["col"], a["n"] - a["col"], b["col"], b["n"] - b["col"])
    print(f"g_demo 도착 {a['ok']}/{a['n']} 대 {b['ok']}/{b['n']}, Fisher 양쪽 p {p_ok:.4f}")
    print(f"g_demo 충돌 판 {a['col']}/{a['n']} 대 {b['col']}/{b['n']}, Fisher 양쪽 p {p_col:.4f}")
    for key, v in sorted(meet_counts(meets).items()):
        print(f"만남 {key[0]} {key[1]}: 판 {v[0]}, 만남 {v[1]}, "
              f"만나고 닿지 않고 도착 {v[2]}, 만나고 닿음 {v[3]}")
    f = cells[("f_cart", SYM)]
    print(f"f_cart 새 정책: 통로 판 {f['corridor']}/{f['n']}, 충돌 판 {f['col']} -> "
          + ("조건을 채웠다" if f_ok else "조건을 채우지 못했다"))
    print(f"합계: 도착 차이 {d_arr:+d}, 충돌 판 차이 {d_col:+d} -> {verdict}")
    put = f_ok and verdict != "Gazebo에서는 나빠졌다"
    print("저장소에 넣는가:", "넣는다" if put else "넣지 않는다")


if __name__ == "__main__":
    main()

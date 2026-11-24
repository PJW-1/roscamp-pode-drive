"""acd3 Gazebo 표: shim으로 감싼 4b, acd2, acd3의 칸 값과 쌍별 차이(후보 - 기준).
  python3 gz_results.py <gz_run.sh OUT> [dest]   (WSL, ROS install을 source한 셸)
구간 값과 통계는 rl_acd_2026-11-24/gz_results.py를 그대로 쓰고, 컨트롤러 이름과 비교 쌍만 바꾼다.
"""
import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "gz_results_acd2", HERE.parent / "rl_acd_2026-11-24" / "gz_results.py")
g = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g)

NAMES = {"FollowPathRLShim": "RL+shim 4b", "FollowPathRLShimACD2": "RL+shim acd2",
         "FollowPathRLShimACD3": "RL+shim acd3"}
# 비교 쌍 (기준, 후보): 차이는 후보 - 기준
PAIRS = (("FollowPathRLShimACD2", "FollowPathRLShimACD3"),
         ("FollowPathRLShim", "FollowPathRLShimACD3"),
         ("FollowPathRLShim", "FollowPathRLShimACD2"))


# ---- 쌍마다 원래 summarize를 돌려 칸은 한 번씩만, 차이에는 쌍 이름을 붙인다 ----
def summarize(rows):
    cells, comps, seen = [], [], set()
    for base, cand in PAIRS:
        g.A, g.B = base, cand
        g.NAMES = {base: NAMES[base], cand: NAMES[cand]}
        part_cells, part_comps = g.summarize(rows)
        for c in part_cells:
            if (c["scenario"], c["name"]) not in seen:
                seen.add((c["scenario"], c["name"]))
                cells.append(c)
        for c in part_comps:
            d = c.pop("diff_acd2_minus_4b")
            comps.append({"pair": f"{NAMES[cand]} - {NAMES[base]}", **c, "diff": d})
    cells.sort(key=lambda c: (c["scenario"], list(NAMES.values()).index(c["name"])))
    return cells, comps


def main():
    out = Path(sys.argv[1])
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE
    rows = g.legs(out)
    cells, comps = summarize(rows)
    g.write(dest / "gz_legs.csv", rows)
    g.write(dest / "gz_cells.csv", cells)
    g.write(dest / "gz_compare.csv", comps)
    print(g.table(cells))
    print()
    print(g.table(comps))


if __name__ == "__main__":
    main()

"""acd3 표: 고친 v7 지도에서 acd2와 새 학습 셋(map, wall, lidar)을 같은 에피소드로 견준다 (Isaac 불필요).

  python docs/rl_acd3_2026-11-25/acd3_tables.py <acd3 runs dir> <out dir> [partial]

평가는 모두 play.py --episodes 1 --seed 7, 4096 env 라 같은 칸의 실행을 env 번호로 짝짓는다.
cells.csv   정책 x 칸마다 한 줄(acd_tables.cell_row와 같은 열)
paired.csv  같은 칸에서 정책 - acd2 의 성공 차이(%p)와 SE, |차이| > 2 SE 면 outside
lidar.csv   lidar 배율 0.958(pode2 실측) 평가: 같은 정책의 배율 1.0 대비 차이, 그리고 그 칸의 정책 - acd2
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning" / "store_arena"))
from acd_tables import cell_row, noise, r4, same_episodes  # noqa: E402
from report_tables import paired, write  # noqa: E402

POLICIES = ("acd2", "map", "wall", "lidar")
CELLS = [(0.12, "rand"), (0.12, "head"), (0.08, "rand"), (0.08, "head")]


def run_dir(runs, pol, cap, s, l958=False):
    ct = "08" if cap == 0.08 else ""
    return f"{runs}/e_{pol}_{ct}{s}" + ("_l958" if l958 else "")


def have(d):
    return os.path.exists(d + "/eval.json")


# ---- 칸마다 정책 줄과 acd2 대비 짝 차이 ----
def tables(runs, partial=False):
    cells, pairs = [], []
    for cap, s in CELLS:
        dirs = {}
        for pol in POLICIES:
            d = run_dir(runs, pol, cap, s)
            if partial and not have(d):
                continue
            cells.append(cell_row(d, pol, None, cap, s))
            dirs[pol] = d
        if "acd2" not in dirs:
            continue
        for pol, d in dirs.items():
            if pol == "acd2":
                continue
            if not same_episodes(dirs["acd2"], d):
                raise ValueError(f"{dirs['acd2']} and {d} do not start the same episodes")
            m, se = paired(d, dirs["acd2"])
            pairs.append({"cap_mps": cap, "set": s, "policy": pol, "vs": "acd2",
                          "diff_pp": r4(m), "se_pp": r4(se), "noise": noise(m, se)})
    return cells, pairs


# ---- lidar 배율 0.958: 같은 정책 1.0 대비, 그리고 acd2(0.958) 대비 ----
def lidar_rows(runs):
    rows = []
    base = run_dir(runs, "acd2", 0.12, "rand", l958=True)
    for pol in POLICIES:
        d, d1 = run_dir(runs, pol, 0.12, "rand", l958=True), run_dir(runs, pol, 0.12, "rand")
        if not (have(d) and have(d1)):
            continue
        row = cell_row(d, pol, None, 0.12, "rand_l958")
        m, se = paired(d, d1)
        row.update({"vs_same_policy_scale1_pp": r4(m), "vs_same_policy_se_pp": r4(se),
                    "vs_same_policy_noise": noise(m, se)})
        if pol != "acd2" and have(base):
            m, se = paired(d, base)
            row.update({"vs_acd2_l958_pp": r4(m), "vs_acd2_l958_se_pp": r4(se),
                        "vs_acd2_l958_noise": noise(m, se)})
        rows.append(row)
    return rows


if __name__ == "__main__":
    runs, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    cells, pairs = tables(runs, partial="partial" in sys.argv[3:])
    write(f"{out}/cells.csv", cells)
    write(f"{out}/paired.csv", pairs)
    lid = lidar_rows(runs)
    if lid:
        write(f"{out}/lidar.csv", lid)

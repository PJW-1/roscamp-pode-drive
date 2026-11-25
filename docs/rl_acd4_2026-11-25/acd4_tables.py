"""acd4 표: 4 cm 진열대 v7 지도에서 acd3와 이어 학습한 s4를 같은 에피소드로 견준다 (Isaac 불필요).

  python docs/rl_acd4_2026-11-25/acd4_tables.py <acd4 runs dir> <out dir> [<acd3 runs dir>]

평가는 모두 play.py --episodes 1 --seed 7, 4096 env 라 같은 칸의 실행을 env 번호로 짝짓는다.
cells.csv   정책 x 칸마다 한 줄(acd_tables.cell_row와 같은 열). acd3 runs dir을 주면 옛 지도(진열대 A 5 mm)의 acd3 줄도 더한다
paired.csv  같은 칸에서 s4 - acd3 의 성공 차이(%p)와 SE, |차이| > 2 SE 면 outside
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning" / "store_arena"))
from acd_tables import cell_row, noise, r4, same_episodes  # noqa: E402
from report_tables import paired, write  # noqa: E402

POLICIES = ("acd3", "s4")
CELLS = [(0.12, "rand"), (0.12, "head"), (0.08, "rand"), (0.08, "head")]


def run_dir(runs, pol, cap, s):
    ct = "08" if cap == 0.08 else ""
    return f"{runs}/e_{pol}_{ct}{s}"


def have(d):
    return os.path.exists(d + "/eval.json")


# ---- 칸마다 정책 줄과 acd3 대비 짝 차이 ----
def tables(runs, old_runs=None):
    cells, pairs = [], []
    for cap, s in CELLS:
        dirs = {pol: run_dir(runs, pol, cap, s) for pol in POLICIES}
        dirs = {pol: d for pol, d in dirs.items() if have(d)}
        for pol, d in dirs.items():
            cells.append(cell_row(d, pol, None, cap, s))
        if old_runs:
            ct = "08" if cap == 0.08 else ""
            old = f"{old_runs}/e_wall_{ct}{s}"
            if have(old):
                cells.append(cell_row(old, "acd3_oldmap", None, cap, s))
        if len(dirs) < 2:
            continue
        if not same_episodes(dirs["acd3"], dirs["s4"]):
            raise ValueError(f"{dirs['acd3']} and {dirs['s4']} do not start the same episodes")
        m, se = paired(dirs["s4"], dirs["acd3"])
        pairs.append({"cap_mps": cap, "set": s, "policy": "s4", "vs": "acd3",
                      "diff_pp": r4(m), "se_pp": r4(se), "noise": noise(m, se)})
    return cells, pairs


def main(argv):
    runs, out = argv[0], Path(argv[1])
    old = argv[2] if len(argv) > 2 else None
    out.mkdir(parents=True, exist_ok=True)
    cells, pairs = tables(runs, old)
    write(out / "cells.csv", cells)
    if pairs:
        write(out / "paired.csv", pairs)
    for p in pairs:
        print(p)


if __name__ == "__main__":
    main(sys.argv[1:])

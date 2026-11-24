"""청소 경로 그림: 청소 격자, 줄, 둥글게 바꾼 이동 구간, 덮은 칸을 PNG로 그린다.

    python3 tools/coverage_plot.py --layout v7 --out docs/img/coverage_v7.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc" / "fleet_core"))

from fleet_core import coverage as C  # noqa: E402


def draw(layout: str, out: Path, spacing: float) -> dict:
    site = C.load_fleet_map(layout)
    plan = C.make_plan(site, C.Params(line_spacing_m=spacing))
    grid = plan.grid
    x0, y0, x1, y1 = site.bounds()
    fig, ax = plt.subplots(figsize=(10, 10 * (y1 - y0) / (x1 - x0) + 0.8))

    # ---- 바닥과 청소 격자, 덮은 칸 ----
    shade = np.zeros(grid.free.shape)
    shade[grid.floor] = 1
    shade[grid.free] = 2
    shade[grid.floor & ~grid.free & C.covered(plan)] = 3
    colors = matplotlib.colors.ListedColormap(["#ffffff", "#e6e6e6", "#cfe8cf", "#f3e0b5"])
    ax.imshow(shade, origin="lower", extent=(x0, x1, y0, y1), cmap=colors, vmin=0, vmax=3,
              interpolation="nearest")

    # ---- 진열대와 막는 구역 ----
    for rect in dict.fromkeys((*site.shelves, *site.sim_shelves)):
        ax.add_patch(plt.Rectangle(rect[:2], rect[2] - rect[0], rect[3] - rect[1],
                                   fc="#8c6d46", ec="none", alpha=0.8))
    for rect in site.keepout_extra:
        ax.add_patch(plt.Rectangle(rect[:2], rect[2] - rect[0], rect[3] - rect[1],
                                   fc="none", ec="#b03030", hatch="//", lw=0.8))

    # ---- 줄과 이동 구간 ----
    for a, b, _ in plan.lines:
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#1f5fbf", lw=2)
    for _, _, pts in plan.transits:
        r = C.rounded(pts, plan.params.turn_radius_m)
        ax.plot([p[0] for p in r], [p[1] for p in r], color="#d9822b", lw=1, ls="--")
    ax.plot(*plan.home, "k^", ms=8)
    ax.plot(*plan.start, "ko", ms=5)

    s = C.summary(plan)
    ax.set_title(f"{layout}: {s['lines']} lines, {s['lap_time_s']} s, "
                 f"coverage hall {s['coverage_hall_floor']:.0%} / grid {s['coverage_clean_grid']:.0%}")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", default="v7")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--line-spacing", type=float, default=C.Params().line_spacing_m)
    args = ap.parse_args(argv)
    print(draw(args.layout, args.out, args.line_spacing))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

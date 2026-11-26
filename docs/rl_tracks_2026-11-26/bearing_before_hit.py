"""사람과 닿아 끝난 추적 에피소드에서, 닿기 전에 사람이 로봇의 어느 방향에 있었는지 센다.

평가의 `rollouts.npz`에는 로봇의 방향(yaw)이 없다. 그래서 로봇의 앞 방향을 자신의 이동(0.3 s 동안의 변위)에서 얻는다.
그동안 명령 속도 v의 평균이 음수였으면(후진) 변위의 반대쪽을 앞으로 본다. 5 mm보다 적게 움직인 스텝은 방위 통계에서 뺀다.
거리는 로봇 중심에서 사람 중심까지다.
닿은 사람은 끝나기 직전 스텝에 가장 가까이 있던 사람으로 본다.

사용: python bearing_before_hit.py <demo1t 실행 폴더> [<실행 폴더> ...]
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cross_warning as cw  # noqa: E402

HEADING_S = 0.3  # 진행 방향을 얻는 변위의 구간
MIN_MOVE_M = 0.005  # 이보다 적게 움직였으면 진행 방향을 모른다
BACK_S = (0.3, 0.5, 1.0, 1.5, 2.0, 3.0)  # 닿기 전 시점들
EDGES = (30.0, 60.0, 90.0)  # 방위 구간의 경계 (deg)


def abs_bearing_deg(prev_xy, xy, person_xy, reverse: bool = False, min_move: float = MIN_MOVE_M):
    """로봇이 prev_xy에서 xy로 갔을 때, 로봇의 앞에서 본 사람의 방위 크기(0 = 정면, 180 = 바로 뒤).
    reverse면 후진으로 간 것이라 앞은 변위의 반대쪽이다. 로봇이 min_move보다 적게 움직였으면 None."""
    d = np.asarray(xy, dtype=float) - np.asarray(prev_xy, dtype=float)
    if np.linalg.norm(d) < min_move:
        return None
    if reverse:
        d = -d
    rel = np.asarray(person_xy, dtype=float) - np.asarray(xy, dtype=float)
    a = np.arctan2(rel[1], rel[0]) - np.arctan2(d[1], d[0])
    return float(abs(np.degrees(np.arctan2(np.sin(a), np.cos(a)))))


def reversing(v_cmds, t: int, w: int) -> bool:
    """위치 [t - w]에서 [t]까지의 변위를 만든 명령 v(스텝 t - w + 1부터 t까지)의 평균이 음수인지.
    평가는 스텝이 끝난 뒤의 위치와 그 스텝에 쓴 명령을 같은 칸에 적는다."""
    return bool(np.mean(np.asarray(v_cmds, dtype=float)[t - w + 1:t + 1]) < 0.0)


def bins(bearings) -> list[int]:
    """방위 크기를 EDGES로 나눈 수: [30 이하, 30~60, 60~90, 90 초과]."""
    b = np.asarray(bearings, dtype=float)
    lo = (-1.0,) + EDGES
    hi = EDGES + (np.inf,)
    return [int(((b > x) & (b <= y)).sum()) for x, y in zip(lo, hi)]


def table(run) -> list[dict]:
    """시점마다 중심 거리와 방위 크기의 사분위수, 방위 구간별 수, 방위를 모르는 수."""
    run = pathlib.Path(run)
    rows = cw.rows(run)
    with np.load(run / "rollouts.npz", allow_pickle=True) as z:
        dt = float(z["dt"])
        xy, done, mover, on, cmd = z["xy"], z["done"], z["mover_xy"], z["mover_on"], z["cmd"]
    w = max(1, round(HEADING_S / dt))
    out = []
    for back_s in BACK_S:
        k = round(back_s / dt)
        bear, dist = [], []
        for r in rows:
            e = int(r["env"])
            end = int(np.argmax(done[:, e]))  # 첫 에피소드가 끝난 스텝
            t, last = end - k, max(end - 1, 0)
            who = int(np.argmin(np.linalg.norm(mover[last, e] - xy[last, e], axis=-1)))
            if t - w < 0 or not on[t, e, who]:
                continue
            dist.append(float(np.linalg.norm(mover[t, e, who] - xy[t, e])))
            b = abs_bearing_deg(xy[t - w, e], xy[t, e], mover[t, e, who], reversing(cmd[:, e, 0], t, w))
            if b is not None:
                bear.append(b)
        out.append({"back_s": back_s, "hits": len(rows), "with_bearing": len(bear),
                    "dist_q": [float(x) for x in np.percentile(dist, [25, 50, 75])] if dist else [],
                    "bearing_q": [float(x) for x in np.percentile(bear, [25, 50, 75])] if bear else [],
                    "bins": bins(bear)})
    return out


def main() -> None:
    for run in sys.argv[1:]:
        print(run)
        for r in table(run):
            dq = " / ".join(f"{x:.2f}" for x in r["dist_q"])
            bq = " / ".join(f"{x:.0f}" for x in r["bearing_q"])
            n30, n60, n90, back = r["bins"]
            print(f"  {r['back_s']:.1f} s before: centre distance {dq} m, |bearing| {bq} deg,"
                  f" <=30: {n30}, 30-60: {n60}, 60-90: {n90}, >90: {back}"
                  f" (of {r['with_bearing']} with a heading; {r['hits']} tracked person collisions)")


if __name__ == "__main__":
    main()

"""건너는 사람과의 충돌을 건넘 무리별로 센다. 두 실행을 주면 같은 에피소드끼리의 짝 차이도 낸다.

평가(play.py)가 쓴 episodes.npz의 code, hit_kind, cross, cross_path를 읽는다. 사람과의 충돌은 "충돌로 끝났고 그때 가장
가까운 것이 움직이는 물체(mover)"인 에피소드다. 무리는 건넘이 전역 경로 위, 경로 밖, 건넘 없음이다.

사용: python cross_person.py <실행 폴더> [<대조 실행 폴더>]
"""
import pathlib
import sys

import numpy as np

GROUPS = ("cross_on_path", "cross_off_path", "no_cross")


def load(run) -> dict:
    """실행 폴더의 episodes.npz에서 사람 충돌 여부와 건넘 무리를 읽는다."""
    z = np.load(pathlib.Path(run) / "episodes.npz", allow_pickle=True)
    keys = [str(k) for k in z["keys"]]
    kinds = [str(k) for k in z["hit_kinds"]]
    hit = (z["code"] == keys.index("collision")) & (z["hit_kind"] == kinds.index("mover"))
    cross, on = z["cross"].astype(bool), z["cross_path"].astype(bool)
    return {"person_hit": hit, "cross": cross, "cross_path": on, "env": z["env"]}


def masks(r: dict) -> dict:
    return {"cross_on_path": r["cross_path"], "cross_off_path": r["cross"] & ~r["cross_path"],
            "no_cross": ~r["cross"]}


def rates(r: dict) -> dict:
    """무리마다 에피소드 수, 사람 충돌 %, 이항 SE(%p)."""
    out = {}
    for g, m in masks(r).items():
        n = int(m.sum())
        p = float(r["person_hit"][m].mean()) if n else float("nan")
        out[g] = {"n": n, "person_pct": 100 * p, "se_pp": 100 * (p * (1 - p) / n) ** 0.5 if n else float("nan")}
    return out


def paired(a: dict, b: dict) -> dict:
    """같은 에피소드끼리 a - b의 사람 충돌 차이(%p)와 SE. 두 실행의 env와 건넘 무리가 같아야 한다."""
    same = (np.array_equal(a["env"], b["env"]) and np.array_equal(a["cross"], b["cross"])
            and np.array_equal(a["cross_path"], b["cross_path"]))
    if not same:
        raise ValueError("the two runs are not the same episodes (env or crossing groups differ)")
    out = {}
    for g, m in masks(a).items():
        d = a["person_hit"][m].astype(float) - b["person_hit"][m].astype(float)
        n = len(d)
        se = float(np.std(d, ddof=1) / n ** 0.5) if n > 1 else float("nan")
        out[g] = {"n": n, "diff_pp": 100 * float(d.mean()) if n else float("nan"), "se_pp": 100 * se}
    return out


def main():
    a = load(sys.argv[1])
    print(f"{sys.argv[1]}")
    for g, r in rates(a).items():
        print(f"  {g:15s} n {r['n']:5d}  person collision {r['person_pct']:5.1f} % (SE {r['se_pp']:.1f})")
    if len(sys.argv) > 2:
        b = load(sys.argv[2])
        print(f"{sys.argv[2]}")
        for g, r in rates(b).items():
            print(f"  {g:15s} n {r['n']:5d}  person collision {r['person_pct']:5.1f} % (SE {r['se_pp']:.1f})")
        print("paired (first - second)")
        for g, r in paired(a, b).items():
            two = "" if not r["n"] > 1 else ("  outside 2 SE" if abs(r["diff_pp"]) > 2 * r["se_pp"] else "  inside 2 SE")
            print(f"  {g:15s} n {r['n']:5d}  {r['diff_pp']:+6.2f} %p (SE {r['se_pp']:.2f}){two}")


if __name__ == "__main__":
    main()

"""건너는 사람과의 충돌을 건넘 무리별로 센다. 두 실행을 주면 같은 에피소드끼리의 짝 차이도 낸다.

평가(play.py)가 쓴 episodes.npz의 code, hit_kind, cross, cross_path를 읽는다. 사람과의 충돌은 "충돌로 끝났고 그때 가장
가까운 것이 움직이는 물체(mover)"인 에피소드다. 무리는 건넘이 전역 경로 위, 경로 밖, 건넘 없음이다.

두 실행이 같은 에피소드로 시작했는지는 n26_tables.check_same(추적 env의 시작 상태, 경로 표, env 묶음)으로 먼저 본다.
같지 않으면 짝 차이 대신 독립 표본 차이(두 비율의 독립 SE)를 낸다.

사용: python cross_person.py <실행 폴더> [<대조 실행 폴더>]
"""
import pathlib
import sys

import numpy as np

import n26_tables as nt

GROUPS = ("cross_on_path", "cross_off_path", "no_cross")


def load(run) -> dict:
    """실행 폴더의 episodes.npz에서 사람 충돌 여부와 건넘 무리를 읽는다. env 번호 순서로 정렬한다
    (평가는 끝난 순서대로 적으므로 정책이 다르면 기록 순서가 다르다)."""
    z = np.load(pathlib.Path(run) / "episodes.npz", allow_pickle=True)
    keys = [str(k) for k in z["keys"]]
    kinds = [str(k) for k in z["hit_kinds"]]
    hit = (z["code"] == keys.index("collision")) & (z["hit_kind"] == kinds.index("mover"))
    cross, on = z["cross"].astype(bool), z["cross_path"].astype(bool)
    order = np.argsort(z["env"], kind="stable")
    return {"person_hit": hit[order], "cross": cross[order], "cross_path": on[order], "env": z["env"][order]}


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
    """같은 에피소드끼리 a - b의 사람 충돌 차이(%p)와 SE. 두 실행의 env와 건넘 무리가 같아야 한다.
    시작 상태까지 같은지는 여기서 알 수 없다: main이 n26_tables.check_same으로 먼저 본다."""
    if len(np.unique(a["env"])) != len(a["env"]) or len(np.unique(b["env"])) != len(b["env"]):
        raise ValueError("more than one episode per env: later episodes cannot be checked to be the same")
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


def two_se(diff: float, se: float) -> str:
    """차이가 2 SE 밖인지. 판정할 수 없으면(표본 없음, SE 없음) 빈 문자열."""
    if not (np.isfinite(diff) and np.isfinite(se)):
        return ""
    return "outside 2 SE" if abs(diff) > 2 * se else "inside 2 SE"


def unpaired(a: dict, b: dict) -> dict:
    """서로 다른 에피소드일 때: 무리마다 a - b의 사람 충돌 차이(%p)와 두 비율의 독립 SE."""
    ra, rb = rates(a), rates(b)
    out = {}
    for g in GROUPS:
        out[g] = {"n_a": ra[g]["n"], "n_b": rb[g]["n"],
                  "diff_pp": ra[g]["person_pct"] - rb[g]["person_pct"],
                  "se_pp": (ra[g]["se_pp"] ** 2 + rb[g]["se_pp"] ** 2) ** 0.5}
    return out


def compare(run_a, run_b) -> tuple[str, str, dict]:
    """두 실행의 사람 충돌 차이(a - b). ("paired" 또는 "unpaired", 까닭, 무리별 행).
    같은 에피소드로 시작했고 건넘 무리도 같을 때만 짝을 짓고, 아니면 독립 표본 차이를 낸다."""
    a, b = load(run_a), load(run_b)
    bad, tracked = nt.check_same(str(run_a), str(run_b))
    rows = None
    if not bad:
        try:
            rows = paired(a, b)
        except ValueError as e:
            bad = str(e)
    if rows is None:
        kind, why, rows = "unpaired", bad, unpaired(a, b)
    else:
        kind, why = "paired", f"same start checked on {tracked} tracked envs"
    for r in rows.values():
        r["two_se"] = two_se(r["diff_pp"], r["se_pp"])
    return kind, why, rows


def main():
    for run in sys.argv[1:3]:
        print(run)
        for g, r in rates(load(run)).items():
            print(f"  {g:15s} n {r['n']:5d}  person collision {r['person_pct']:5.1f} % (SE {r['se_pp']:.1f})")
    if len(sys.argv) > 2:
        kind, why, rows = compare(sys.argv[1], sys.argv[2])
        note = "independent difference" if kind == "unpaired" else "paired difference"
        print(f"{note} (first - second): {why}")
        for g, r in rows.items():
            n = f"{r['n']:5d}" if kind == "paired" else f"{r['n_a']:5d} / {r['n_b']:5d}"
            print(f"  {g:15s} n {n}  {r['diff_pp']:+6.2f} %p (SE {r['se_pp']:.2f})  {r['two_se']}")


if __name__ == "__main__":
    main()

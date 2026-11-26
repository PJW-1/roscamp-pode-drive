"""두 평가의 분당 w 부호 바뀜(회/분)을 견주고 차이의 SE를 낸다.

평가(play.py)가 쓴 episodes.npz의 flips(에피소드의 w 부호 바뀜 수)와 length_s(에피소드 길이)를 읽는다. 비율은 표의
w_flips_per_min과 같은 셈이다: 60 × 부호 바뀜의 합 ÷ 길이의 합. 합끼리의 비라서 차이의 SE는 에피소드를 다시 뽑아
(부트스트랩) 구한다.

두 실행이 같은 에피소드로 시작했는지는 n26_tables.check_same(추적 env의 시작 상태, 경로 표, env 묶음)으로 먼저 본다.
같으면 같은 에피소드를 두 실행에서 같이 뽑고, 같지 않으면 두 실행을 따로 뽑는다.

사용: python flip_rate.py <실행 폴더> <대조 실행 폴더>
"""
import pathlib
import sys

import numpy as np

import n26_tables as nt

N_BOOT = 2000
SEED = 0


def load(run) -> dict:
    """실행 폴더의 episodes.npz에서 부호 바뀜 수와 길이를 읽는다. env 번호 순서로 정렬한다
    (평가는 끝난 순서대로 적으므로 정책이 다르면 기록 순서가 다르다)."""
    z = np.load(pathlib.Path(run) / "episodes.npz", allow_pickle=True)
    order = np.argsort(z["env"], kind="stable")
    return {"env": z["env"][order], "flips": z["flips"][order].astype(float),
            "length_s": z["length_s"][order].astype(float)}


def rate(flips, length_s) -> float:
    """분당 부호 바뀜: 60 × 부호 바뀜의 합 ÷ 길이의 합."""
    return float(60.0 * np.sum(flips) / np.sum(length_s))


def resampled(r: dict, idx) -> np.ndarray:
    """다시 뽑은 묶음마다의 분당 부호 바뀜. idx는 (묶음 수, 에피소드 수)."""
    return 60.0 * r["flips"][idx].sum(axis=1) / r["length_s"][idx].sum(axis=1)


def compare(a: dict, b: dict, paired: bool, n_boot: int = N_BOOT, seed: int = SEED) -> dict:
    """a - b의 분당 부호 바뀜 차이와 SE. paired이면 같은 에피소드를 두 실행에서 같이 뽑는다."""
    rng = np.random.default_rng(seed)
    ia = rng.integers(0, len(a["env"]), size=(n_boot, len(a["env"])))
    if paired:
        for r in (a, b):
            if len(np.unique(r["env"])) != len(r["env"]):
                raise ValueError("more than one episode per env: later episodes cannot be checked to be the same")
        if not np.array_equal(a["env"], b["env"]):
            raise ValueError("the two runs are not the same episodes (env sets differ)")
        ib = ia
    else:
        ib = rng.integers(0, len(b["env"]), size=(n_boot, len(b["env"])))
    ra, rb = rate(a["flips"], a["length_s"]), rate(b["flips"], b["length_s"])
    return {"kind": "paired" if paired else "unpaired", "n": len(a["env"]), "n_b": len(b["env"]),
            "rate_a": ra, "rate_b": rb, "diff": ra - rb,
            "se": float(np.std(resampled(a, ia) - resampled(b, ib)))}


def compare_runs(run_a, run_b, n_boot: int = N_BOOT) -> tuple[dict, str]:
    """두 실행 폴더의 차이(a - b)와 짝을 지었는지의 까닭. 같은 에피소드로 시작했을 때만 짝을 짓는다."""
    a, b = load(run_a), load(run_b)
    bad, tracked = nt.check_same(str(run_a), str(run_b))
    if not bad:
        try:
            return compare(a, b, True, n_boot), f"same start checked on {tracked} tracked envs"
        except ValueError as e:
            bad = str(e)
    return compare(a, b, False, n_boot), bad


def main():
    r, why = compare_runs(sys.argv[1], sys.argv[2])
    out = abs(r["diff"]) > 2 * r["se"]
    print(f"w flips per min: {r['rate_a']:.2f} vs {r['rate_b']:.2f}, a - b {r['diff']:+.2f} "
          f"(SE {r['se']:.2f}, {r['kind']}: {why}, n {r['n']} and {r['n_b']}, {'outside' if out else 'inside'} 2 SE)")


if __name__ == "__main__":
    main()

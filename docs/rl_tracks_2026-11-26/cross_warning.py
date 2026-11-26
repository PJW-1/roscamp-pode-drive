"""건너는 사람과 닿은 에피소드에서, 닿기 전에 사람이 얼마나 오래 보였는지 센다 (작업 기록 21절).

평가(play.py)가 쓴 rollouts.npz와 episodes.npz에서, 추적 env의 첫 에피소드가 건넘이 경로 위였고 사람과 닿아 끝난 것을
고른다. 닿기 직전 스텝부터 거꾸로, 로봇 중심에서 사람까지의 직선이 벽·진열대·상자에 막히지 않은 스텝이 이어진 시간을
"보인 시간"으로 센다. 정책은 10도 섹터의 최솟값만 받으므로, 이 값은 정책이 알아챌 수 있던 시간보다 길게 잡히는 어림이다.

두 번째 폴더(같은 정책의 기존 평가)를 주면 결과 비율과 추적 env 안의 사람 충돌 수가 같은지도 본다.

사용: python cross_warning.py <실행 폴더> [<같은 정책의 기존 평가 폴더>]
"""
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "learning" / "store_arena"))
import timeouts  # noqa: E402

CLASSES = ("robot_stopped", "late_sight", "seen_short", "seen_long")
LATE_S, LONG_S = 1.0, 3.0  # 보인 시간이 이보다 짧으면 late_sight, 이 이상이면 seen_long
STOP_MPS = 0.02  # 끝 END_S 동안의 평균 속력이 이보다 느리면 서 있던 것으로 센다 (로봇과 사람 모두)
END_S = 0.5
MIN_N = 20  # 이보다 적으면 판정하지 않는다
PERSON_R = 0.04  # mover_spec의 person 반지름
SIGHT_LINES = 9  # 사람 폭을 이만큼의 점으로 나눠 본다 (1 cm 간격)
COLLISION = 1  # play.py의 결과 순서 (success, collision, oob, timeout)


def blocked(p, q, rect) -> bool:
    """선분 p-q가 축정렬 rect (x0, y0, x1, y1)를 지나는지."""
    t0, t1 = 0.0, 1.0
    d = np.asarray(q, dtype=float) - np.asarray(p, dtype=float)
    for axis in (0, 1):
        lo, hi = float(rect[axis]), float(rect[axis + 2])
        if abs(d[axis]) < 1e-12:
            if not (lo <= p[axis] <= hi):
                return False
            continue
        a, b = (lo - p[axis]) / d[axis], (hi - p[axis]) / d[axis]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return False
    return True


def visible(robot, person, rects, r=PERSON_R) -> bool:
    """로봇 중심에서 사람의 폭(시선에 수직인 지름) 위 SIGHT_LINES개 점 가운데 하나라도 막히지 않고 보이는지."""
    robot, person = np.asarray(robot, dtype=float), np.asarray(person, dtype=float)
    d = person - robot
    n = float(np.linalg.norm(d))
    side = np.array((-d[1], d[0])) / n * r if n > 1e-9 else np.zeros(2)
    return any(not any(blocked(robot, person + k * side, rc) for rc in rects)
               for k in np.linspace(-1.0, 1.0, SIGHT_LINES))


def kind_of(warning_s: float, robot_mps: float, person_mps: float) -> str:
    """robot_stopped는 서 있는 로봇에 움직이는 사람이 닿은 것만이다. 둘 다 서 있었으면 보인 시간으로 나눈다."""
    if robot_mps < STOP_MPS and person_mps >= STOP_MPS:
        return "robot_stopped"
    if warning_s < LATE_S:
        return "late_sight"
    return "seen_long" if warning_s >= LONG_S else "seen_short"


def person_hits(run):
    """episodes.npz에서 env 번호 -> 건넘이 경로 위였고 사람과 닿아 끝났는지. env마다 에피소드 하나여야 한다."""
    with np.load(pathlib.Path(run) / "episodes.npz", allow_pickle=True) as f:
        keys, kinds = [str(k) for k in f["keys"]], [str(k) for k in f["hit_kinds"]]
        env = f["env"]
        hit = ((f["code"] == keys.index("collision")) & (f["hit_kind"] == kinds.index("mover"))
               & f["cross_path"].astype(bool))
    if len(np.unique(env)) != len(env):
        raise ValueError("more than one episode per env: the tracked episode cannot be matched")
    return dict(zip(env.tolist(), hit.tolist(), strict=True))


def speed(xy, dt) -> float:
    """자취 (T, 2)의 평균 속력."""
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=-1).sum() / (dt * max(len(xy) - 1, 1)))


def rows(run) -> list:
    """추적 env의 첫 에피소드가 경로 위 건넘에서 사람과 닿아 끝났으면 한 줄: 보인 시간, 끝 속력, 갈래."""
    hits = person_hits(run)
    with np.load(pathlib.Path(run) / "rollouts.npz", allow_pickle=True) as f:
        z = {k: f[k] for k in f.files}
    dt = float(z["dt"])
    end = max(int(round(END_S / dt)), 2)
    out = []
    for e in range(z["xy"].shape[1]):
        if not hits.get(e, False):
            continue
        s, t = timeouts.episodes(z["done"][:, e])[0]
        if z["outcome"][t, e] != COLLISION:
            raise ValueError(f"env {e}: episodes.npz says collision but rollouts.npz does not")
        # 끝난 스텝 t의 위치는 이미 다음 에피소드의 리셋 자세라서 t - 1까지만 본다. 경로 번호는 t에서도 이 에피소드의 것이다
        last = t - 1
        centers = z["obstacle_xy"][int(z["route"][t, e])]
        rects = [tuple(b) for b in z["boxes"]] + [
            (c[0] - 0.5 * sz, c[1] - 0.5 * sz, c[0] + 0.5 * sz, c[1] + 0.5 * sz)
            for c, sz in zip(centers, z["obstacle_sizes"], strict=True) if not np.isnan(c[0])]
        on = np.flatnonzero(z["mover_on"][s, e])
        gap = np.linalg.norm(z["mover_xy"][last, e, on] - z["xy"][last, e], axis=-1)
        m = int(on[gap.argmin()])  # 닿은 사람: 끝에 가장 가까운 mover
        k = last
        while k >= s and visible(z["xy"][k, e], z["mover_xy"][k, e, m], rects):
            k -= 1
        warning_s = (last - k) * dt
        b = max(s, t - end - 1)  # 위치 end + 1개 = 구간 end개 (끝난 스텝의 구간은 기록에 없다)
        robot_mps = speed(z["xy"][b:t, e], dt)
        person_mps = speed(z["mover_xy"][b:t, e, m], dt)
        out.append({"env": e, "warning_s": warning_s, "robot_mps": robot_mps, "person_mps": person_mps,
                    "length_s": (t - s) * dt, "kind": kind_of(warning_s, robot_mps, person_mps)})
    return out


def verdict(counts: dict) -> str:
    """갈래 하나가 절반을 넘으면 그 갈래. 사람 충돌이 MIN_N보다 적으면 판정하지 않는다."""
    n = sum(counts.values())
    if n < MIN_N:
        return f"not judged (fewer than {MIN_N} tracked person collisions)"
    for c in CLASSES:
        if counts[c] > n / 2:
            return f"mostly {c}"
    return "no majority"


def check(run, base) -> dict:
    """같은 정책의 기존 평가와 결과 비율이 같은지, 추적 env 안의 사람 충돌 수가 얼마씩인지."""
    with np.load(pathlib.Path(run) / "rollouts.npz", allow_pickle=True) as f:
        tracked = f["xy"].shape[1]

    def load(d):
        rates = json.loads((pathlib.Path(d) / "eval.json").read_text(encoding="utf-8"))["rates"]
        return rates, sum(hit for env, hit in person_hits(d).items() if env < tracked)
    (ra, na), (rb, nb) = load(run), load(base)
    return {"rates_equal": ra == rb, "person_hits_in_tracked_envs": [int(na), int(nb)]}


def main():
    run = sys.argv[1]
    rs = rows(run)
    counts = {c: sum(r["kind"] == c for r in rs) for c in CLASSES}
    print(run)
    print(f"  tracked person collisions {len(rs)}: " + ", ".join(f"{c} {counts[c]}" for c in CLASSES)
          + f"  -> {verdict(counts)}")
    if rs:
        w = np.array([r["warning_s"] for r in rs])
        q = np.percentile(w, (25, 50, 75))
        print(f"  seen for (s): quartiles {q[0]:.2f} / {q[1]:.2f} / {q[2]:.2f},"
              f" under {LATE_S:.0f} s: {int((w < LATE_S).sum())}, {LONG_S:.0f} s or more: {int((w >= LONG_S).sum())},"
              f" seen for the whole episode: {sum(r['warning_s'] >= r['length_s'] - 1e-9 for r in rs)}")
        slow = [r for r in rs if r["robot_mps"] < STOP_MPS]
        print(f"  at contact: robot speed median {np.median([r['robot_mps'] for r in rs]):.3f} m/s,"
              f" person moving ({STOP_MPS} m/s or more): {sum(r['person_mps'] >= STOP_MPS for r in rs)} of {len(rs)},"
              f" robot under {STOP_MPS} m/s: {len(slow)} (person also still: {sum(r['person_mps'] < STOP_MPS for r in slow)})")
    if len(sys.argv) > 2:
        print("  against", sys.argv[2], json.dumps(check(run, sys.argv[2])))


if __name__ == "__main__":
    main()

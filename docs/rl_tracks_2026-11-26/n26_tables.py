"""n26 밤 보고서 표: 끝난 학습·평가 폴더에서 CSV와 tables.md를 만든다 (Isaac 불필요).

  python docs/rl_tracks_2026-11-26/n26_tables.py <runs dir> <out dir>

실행은 runs dir의 queue_status.txt에서 그 tag의 마지막 줄이 `<tag> exit=0`일 때만 센다.
없거나 덜 끝난 실행은 건너뛴다. 평가는 모두 play.py --episodes 1 --seed 7, 4096 env 라
env 번호로 짝짓는다.

cells.csv       끝난 평가마다 한 줄: 결과 비율, 충돌의 가장 가까운 것, 스침, 분당 w 반전,
                성공 시간 비
paired.csv      COMPARISONS의 (정책, 칸) 쌍마다 짝 차이(%p)와 SE: 성공, 충돌, 시간 초과
                (에피소드별 충돌 원인은 episodes.npz에 없어 mover·robot 충돌 짝 차이는 없다)
regression.csv  e_acd3_rand와 e_acd3_rand_chk: 결과 코드가 다른 에피소드 수와 짝 차이
training.csv    학습마다 t_ctrl과 다른 설정, 걸린 시간
curve.csv       학습마다 저장된 체크포인트 반복에서 metrics.csv 값과 행동 std (v, w)
scenarios.csv   corner·tee·entry 평가와 같은 정책의 rand 줄: 결과, 충돌 원인, 만남 지표
"""

import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning" / "store_arena"))
from acd_tables import noise, r4, same_episodes  # noqa: E402
from ceiling import load  # noqa: E402
from report_tables import paired, write  # noqa: E402

# ---- 비교 목록: (정책 a, 칸 a, 정책 b, 칸 b), 차이는 a - b ----
BASE_PAIRS = [("std", "ctrl"), ("ctrl", "acd3"), ("sysid", "ctrl"), ("dyn", "ctrl"),
              ("gam", "std"), ("std", "acd3"), ("sysid", "acd3"), ("dyn", "acd3"),
              ("gam", "acd3")]
COMPARISONS = (
    [(a, c, b, c) for c in ("rand", "head") for a, b in BASE_PAIRS]
    + [("sysid", "realrand", "ctrl", "realrand"), ("ctrl", "realrand", "acd3", "realrand")]
    # 응답 범위가 한 값이면 난수를 쓰지 않아 realrand와 rand는 같은 에피소드다
    + [(p, "realrand", p, "rand") for p in ("acd3", "ctrl", "sysid")]
)
REGRESSION = ("e_acd3_rand", "e_acd3_rand_chk")
SCENARIOS = ("corner", "tee", "entry")
HIT_KINDS = ("wall_shelf", "box", "mover", "robot")
CELL_RE = re.compile(r"^e_([a-z0-9]+)_(.+)$")


# ---- 큐 상태: tag의 마지막 줄이 exit=0인 실행만 ----
def finished(runs):
    last = {}
    p = Path(runs) / "queue_status.txt"
    if not p.exists():
        return {}
    for line in p.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if parts:  # 형식이 깨진 줄도 그 tag의 마지막 줄로 친다
            wall = next((x[7:] for x in parts if x.startswith("wall_s=")), "")
            last[parts[0]] = (len(parts) >= 2 and parts[1] == "exit=0", wall)
    return {t: w for t, (ok, w) in last.items() if ok and (Path(runs) / t).is_dir()}


def evals(runs, done, skipped):
    """{(정책, 칸): 폴더} — eval.json과 episodes.npz를 읽을 수 있는 끝난 평가.

    못 읽는 실행은 skipped에 적는다.
    """
    out = {}
    for tag in sorted(done):
        m = CELL_RE.match(tag)
        if not m:
            continue
        d = str(Path(runs) / tag)
        try:
            json.load(open(d + "/eval.json", encoding="utf-8"))["rates"]
            z = np.load(d + "/episodes.npz")
            for k in set(z.files) | {"env", "code", "keys", "length_s"}:  # 항목마다 끝까지 읽어 봄
                z[k]
        except Exception as e:  # 잘린 파일: 이 실행만 빼고 표는 계속 만든다
            skipped.append(f"{tag} ({type(e).__name__})")
            continue
        out[(m.group(1), m.group(2))] = d
    return out


def pct(x):
    return "" if x is None else round(100 * float(x), 2)


# ---- 1. 칸 표 ----
def cell_row(d, pol, cell):
    j = json.load(open(d + "/eval.json", encoding="utf-8"))
    rt, hit = j["rates"], j.get("collisions_by_nearest", {})
    row = {"policy": pol, "cell": cell, "episodes": j["episodes"],
           "success_pct": pct(rt["success"]), "collision_pct": pct(rt["collision"]),
           "timeout_pct": pct(rt["timeout"])}
    for k in HIT_KINDS:
        row[f"coll_{k}"] = hit.get(k, "")
    row["graze_only"] = j.get("collisions_graze_only", "")
    row["w_flips_per_min"] = j.get("w_flips_per_min", "")
    row["success_time_over_path_at_vmax"] = j.get("success_time_over_path_at_vmax_mean", "")
    return row


# ---- 2. 짝 비교 ----
def outcome_diff(da, db, key):
    """에피소드별 (a가 key) - (b가 key)의 평균과 SE, %p."""
    ea, ca, ka = load(da)
    eb, cb, kb = load(db)
    if not np.array_equal(ea, eb):
        raise ValueError(f"{da} and {db} do not count the same envs")
    d = (ca == ka.index(key)).astype(float) - (cb == kb.index(key)).astype(float)
    return 100 * d.mean(), 100 * d.std(ddof=1) / np.sqrt(len(d))


def check_same(da, db):
    """같은 에피소드로 시작했는지: rollouts.npz 첫 스텝과 env 번호. (문제, 확인한 추적 env 수).

    시작 상태는 추적 env(--track, 기본 16개)만 저장되므로 그 env들만 확인된다.
    """
    try:
        if not same_episodes(da, db):
            return "different start episodes", 0
        n = min(np.load(da + "/rollouts.npz")["route"].shape[1],
                np.load(db + "/rollouts.npz")["route"].shape[1])
    except Exception as e:  # rollouts.npz가 없거나 잘림
        return f"start not verified ({type(e).__name__})", 0
    if not np.array_equal(load(da)[0], load(db)[0]):
        return "different env sets", 0
    return "", n


def same_label(da, db):
    bad, n = check_same(da, db)
    return bad or f"yes ({n} tracked envs)", bool(bad)


def pair_row(ev, pa, ca, pb, cb):
    da, db = ev[(pa, ca)], ev[(pb, cb)]
    row = {"policy": pa, "cell": ca, "vs_policy": pb, "vs_cell": cb, "n": len(load(da)[0])}
    row["same_episodes"], bad = same_label(da, db)
    if bad:
        return row
    m, se = paired(da, db)
    row.update(success_diff_pp=r4(m), success_se_pp=r4(se), success_noise=noise(m, se))
    for key in ("collision", "timeout"):
        m, se = outcome_diff(da, db, key)
        row.update({f"{key}_diff_pp": r4(m), f"{key}_se_pp": r4(se), f"{key}_noise": noise(m, se)})
    return row


def pairs(ev):
    return [pair_row(ev, *c) for c in COMPARISONS if (c[0], c[1]) in ev and (c[2], c[3]) in ev]


# ---- 3. 회귀 확인 ----
def regression(runs, ev):
    a, b = REGRESSION
    ka, kb = CELL_RE.match(a).groups(), CELL_RE.match(b).groups()
    if ka not in ev or kb not in ev:
        return []
    da, db = ev[ka], ev[kb]
    row = {"run": b, "vs": a}
    row["same_episodes"], bad = same_label(da, db)
    if bad:
        return [row]
    ea, ca, keys = load(da)
    _, cb, _ = load(db)
    za, zb = np.load(da + "/episodes.npz"), np.load(db + "/episodes.npz")
    oa, ob = np.argsort(za["env"], kind="stable"), np.argsort(zb["env"], kind="stable")
    m, se = paired(db, da)
    row.update(episodes=len(ea), code_differs=int((ca != cb).sum()),
               length_differs=int((za["length_s"][oa] != zb["length_s"][ob]).sum()),
               success_diff_pp=r4(m), success_se_pp=r4(se), success_noise=noise(m, se))
    for i, k in enumerate(keys):  # 바뀐 방향: a의 결과별로 b에서 다른 결과가 된 수
        row[f"from_{k}_changed"] = int(((ca == i) & (cb != i)).sum())
    return [row]


# ---- 4. 학습 표 ----
def flat(d, pre=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flat(v, f"{pre}{k}."))
        else:
            out[pre + k] = json.dumps(v) if isinstance(v, list) else v
    return out


SKIP = ("format", "started", "argv", "set", "git.", "resume", "agent.run_name",
        "agent.experiment_name", "agent.load_run", "agent.load_checkpoint",
        "routes", "env_build_s", "learn_s", "env_steps", "cuda_max", "run_config_sha256")


def settings(d):
    """(출처, 평탄화한 설정): run_config.json, 없으면 run.json."""
    for name in ("run_config.json", "run.json"):
        p = Path(d) / name
        if p.exists():
            f = flat(json.load(open(p, encoding="utf-8")))
            return name, {k: v for k, v in f.items() if not k.startswith(SKIP)}
    return "", {}


def diff_settings(sa, sb):
    """sa에서 sb와 다른 값. 출처가 다르면 양쪽에 있는 키만."""
    (na, a), (nb, b) = sa, sb
    keys = sorted(set(a) | set(b)) if na == nb else sorted(set(a) & set(b))
    return "; ".join(f"{k}={a.get(k, '-')} (ctrl {b.get(k, '-')})"
                     for k in keys if a.get(k) != b.get(k))


def std_of(path):
    import torch

    c = torch.load(path, weights_only=False, map_location="cpu")
    return [round(float(x), 4) for x in c["actor_state_dict"]["distribution.std_param"]]


def start_iter(d):
    m = re.search(r"model_(\d+)\.pt$", str(settings(d)[1].get("init", "")))
    return int(m.group(1)) if m else 0


def training(runs, done):
    rows, curve = [], []
    trains = [t for t in sorted(done) if t.startswith("t_")]
    ctrl = settings(Path(runs) / "t_ctrl") if "t_ctrl" in done else None
    for tag in trains:
        d = Path(runs) / tag
        src, s = settings(d)
        rows.append({"run": tag, "source": src, "wall_s": done[tag],
                     "differs_from_t_ctrl": "" if tag == "t_ctrl" or ctrl is None
                     else diff_settings((src, s), ctrl) or "(none)"})
        met = {}
        if (d / "metrics.csv").exists():
            with open(d / "metrics.csv", encoding="utf-8") as f:
                met = {int(r["iteration"]): r for r in csv.DictReader(f)}
        start = start_iter(d)
        models = sorted((int(p.stem.split("_")[1]), p) for p in d.glob("model_*.pt"))
        points = [(0, None)] + [(it - start, p) for it, p in models]
        for rel, p in points:
            row = {"run": tag, "iteration": rel, "abs_iteration": start + rel}
            r = met.get(rel, {})  # metrics.csv의 반복은 1부터라 시작 줄은 비움
            for k in ("success_rate", "collision_rate", "timeout_rate", "success_time_s"):
                row[k] = r.get(k, "")
            if p is not None:
                row["std_v"], row["std_w"] = std_of(p)
            else:  # 시작 std: --init_std가 0보다 크면 그 값, 아니면 init 체크포인트
                init_std = json.loads(str(s.get("init_std", "[0, 0]")))
                init = s.get("init", "")
                if min(init_std) > 0:
                    row["std_v"], row["std_w"] = init_std
                elif init and Path(init).exists():
                    row["std_v"], row["std_w"] = std_of(init)
            curve.append(row)
    return rows, curve


# ---- 5. 시나리오 표 ----
def scenario_row(d, pol, cell):
    j = json.load(open(d + "/eval.json", encoding="utf-8"))
    rt, hit = j["rates"], j.get("collisions_by_nearest", {})
    n = j["episodes"]
    row = {"policy": pol, "cell": cell, "episodes": n, "success_pct": pct(rt["success"]),
           "collision_pct": pct(rt["collision"]), "timeout_pct": pct(rt["timeout"])}
    for k in HIT_KINDS:
        row[f"coll_{k}"] = hit.get(k, "")
    row["robot_coll_pct"] = round(100 * hit["robot"] / n, 2) if "robot" in hit else ""
    enc = j.get("robot_encounter") or {}
    g, e = enc.get("min_robot_gap_m", {}), enc.get("encounter_s", {})
    row.update(gap_episodes=g.get("episodes_with_robot", ""), gap_mean_m=g.get("mean", ""),
               gap_p10_m=g.get("p10", ""), gap_p50_m=g.get("p50", ""), gap_min_m=g.get("min", ""),
               share_touching=g.get("share_touching", ""),
               encounter_episodes=e.get("episodes", ""), encounter_s_mean=e.get("mean", ""),
               deadlocks=enc.get("deadlocks", ""), deadlock_rate=enc.get("deadlock_rate", ""))
    return row


def scenarios(ev):
    rows = []
    for pol in sorted({p for p, c in ev if c in SCENARIOS}):
        base = scenario_row(ev[(pol, "rand")], pol, "rand") if (pol, "rand") in ev else None
        if base:
            rows.append(base)
        for cell in SCENARIOS:
            if (pol, cell) not in ev:
                continue
            row = scenario_row(ev[(pol, cell)], pol, cell)
            if base and row["robot_coll_pct"] != "" and base["robot_coll_pct"] != "":
                # 다른 에피소드라 짝 없이 두 비율의 독립 SE
                p1, n1 = row["robot_coll_pct"] / 100, row["episodes"]
                p2, n2 = base["robot_coll_pct"] / 100, base["episodes"]
                m = 100 * (p1 - p2)
                se = 100 * np.sqrt(p1 * (1 - p1) / n1 + p2 * (1 - p2) / n2)
                row.update(robot_coll_vs_rand_pp=r4(m), robot_coll_vs_rand_se_pp=r4(se),
                           robot_coll_vs_rand_noise=noise(m, se))
            rows.append(row)
    return rows


# ---- Markdown ----
HEAD = {
    "policy": "정책", "cell": "칸", "episodes": "에피소드", "success_pct": "성공 %",
    "collision_pct": "충돌 %", "timeout_pct": "시간 초과 %", "coll_wall_shelf": "충돌 벽·선반",
    "coll_box": "충돌 상자", "coll_mover": "충돌 사람·카트", "coll_robot": "충돌 로봇",
    "graze_only": "스침만", "w_flips_per_min": "w 반전 회/분",
    "success_time_over_path_at_vmax": "성공 시간/최고속 경로 시간",
    "vs_policy": "대조 정책", "vs_cell": "대조 칸", "n": "짝 수", "same_episodes": "같은 에피소드",
    "success_diff_pp": "성공 차 %p", "success_se_pp": "성공 SE", "success_noise": "성공 2SE",
    "collision_diff_pp": "충돌 차 %p", "collision_se_pp": "충돌 SE", "collision_noise": "충돌 2SE",
    "timeout_diff_pp": "시간 초과 차 %p", "timeout_se_pp": "시간 초과 SE",
    "timeout_noise": "시간 초과 2SE", "run": "실행", "vs": "기준", "code_differs": "결과 다른 수",
    "length_differs": "길이 다른 수", "source": "설정 출처", "wall_s": "걸린 시간 s",
    "differs_from_t_ctrl": "t_ctrl과 다른 설정", "iteration": "반복", "abs_iteration": "누적 반복",
    "success_rate": "성공률", "collision_rate": "충돌률", "timeout_rate": "시간 초과율",
    "success_time_s": "성공 시간 s", "std_v": "std v", "std_w": "std w",
    "robot_coll_pct": "로봇 충돌 %", "robot_coll_vs_rand_pp": "로봇 충돌 rand 대비 %p",
    "robot_coll_vs_rand_se_pp": "그 SE", "robot_coll_vs_rand_noise": "2SE",
    "gap_episodes": "로봇 있던 에피소드", "gap_mean_m": "최소 간격 평균 m", "gap_p10_m": "p10 m",
    "gap_p50_m": "p50 m", "gap_min_m": "최소 m", "share_touching": "닿은 비율",
    "encounter_episodes": "만남 에피소드", "encounter_s_mean": "만남 시각 평균 s",
    "deadlocks": "교착", "deadlock_rate": "교착률",
}


def md_table(rows):
    if not rows:
        return "(해당하는 끝난 실행 없음)\n"
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    out = ["| " + " | ".join(HEAD.get(k, k) for k in keys) + " |",
           "|" + "---|" * len(keys)]
    out += ["| " + " | ".join(str(r.get(k, "")) for k in keys) + " |" for r in rows]
    return "\n".join(out) + "\n"


SECTIONS = [
    ("cells", "1. 평가 칸", ""),
    ("paired", "2. 짝 비교 (a - 대조)",
     "에피소드별 짝 차이와 SE. 2SE가 outside면 |차이| > 2 SE. "
     "같은 에피소드 확인은 시작 상태가 저장된 추적 env(괄호 안 수)와 env 번호만 본다. "
     "에피소드별 충돌 원인(가장 가까운 것)은 episodes.npz에 저장되지 않아 "
     "사람·카트, 로봇 충돌의 짝 차이는 넣지 않았다."),
    ("regression", "3. 회귀 확인 (같은 체크포인트, 같은 seed, 병합 코드)", ""),
    ("training", "4. 학습 설정", "걸린 시간은 queue_status.txt의 wall_s."),
    ("curve", "4b. 학습 곡선",
     "반복은 이 학습 안에서의 반복(0은 시작). "
     "metrics.csv에는 보상과 평균 에피소드 길이가 없어 성공·충돌·시간 초과율과 성공 시간만 적는다. "
     "std는 저장된 model_<누적 반복>.pt의 distribution.std_param."),
    ("scenarios", "5. 시나리오",
     "로봇 충돌 rand 대비 차이는 다른 에피소드라 짝이 아니며, SE는 두 비율의 독립 SE다."),
]


def main(argv):
    runs, out = argv[0], Path(argv[1])
    out.mkdir(parents=True, exist_ok=True)
    done = finished(runs)
    skipped = []
    ev = evals(runs, done, skipped)
    train_rows, curve = training(runs, done)
    tables = {
        "cells": [cell_row(d, p, c) for (p, c), d in sorted(ev.items())],
        "paired": pairs(ev),
        "regression": regression(runs, ev),
        "training": train_rows,
        "curve": curve,
        "scenarios": scenarios(ev),
    }
    names = ", ".join(sorted(done)) or "없음"
    md = ["# n26 표\n", f"실행 폴더 `{runs}`, 끝난 실행(exit=0): {names}\n"]
    if skipped:
        md.append(f"파일을 읽지 못해 뺀 실행: {', '.join(skipped)}\n")
    for name, title, note in SECTIONS:
        write(out / f"{name}.csv", tables[name])
        md += [f"## {title}\n", (note + "\n") if note else "", md_table(tables[name])]
    (out / "tables.md").write_text("\n".join(md), encoding="utf-8")
    print(f"wrote {out / 'tables.md'}")
    return tables


if __name__ == "__main__":
    main(sys.argv[1:])

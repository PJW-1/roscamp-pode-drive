"""묶음 17(같은 설정을 seed만 4로 바꿔 다시 학습)의 판정 기준 1~6번을 적용해 Markdown으로 적는다.

판정 기준과 문장은 worklog_rl_tracks_2026-11-26.md의 22절 그대로다. 성공·충돌의 짝 차이는 n26_tables.py가 만든
results/paired.csv에서 읽고, demo1에서 건넘이 경로 위인 에피소드의 사람 충돌(3번 (e))은 cross_person.compare로,
분당 w 부호 바뀜(5번)은 flip_rate.compare_runs로 평가 폴더에서 직접 계산한다. 차이는 모두 앞 - 뒤다.

"2 SE 밖"은 |차이| > 2 × SE다(cross_person.two_se와 같은 정의). 줄이나 평가 폴더가 없으면 그 항목에 "아직 없음"을
적고, 같은 에피소드 짝이 아닌 비교와 일부만 있는 비교에는 판정 문장을 쓰지 않는다.

사용: python seed_spread.py <평가 실행들이 든 폴더> <results 폴더>
"""
import csv
import pathlib
import statistics
import sys

import numpy as np

import cross_person as cp
import flip_rate as fr
import n26_tables as nt

# ---- 22절의 판정 문장 ----
S1_IN = "이 칸에서는 seed를 바꿔도 평가 오차 안이다"
S1_OUT = "seed만 바꿔도 평가 오차 밖으로 달라진다"
S3_KEEP = "seed를 바꿔도 유지된다"
S3_FLIP = "seed에 따라 뒤집힌다"
S3_SHRINK = "평가 오차 안으로 줄어드는 조합이 있다"
S3F_NONE = "seed를 바꿔도 차이가 보이지 않는다"
S4_KEEP = "seed를 바꿔도 유지된다"
S4_GONE = "seed 4에서는 보이지 않는다"
S5_UP = "seed 4에서도 t_ctrl보다 는다"
S5_FLAT = "seed 4에서는 t_ctrl과의 차이가 보이지 않는다"
S5_DOWN = "seed 4에서는 t_ctrl보다 준다"
S5_SEED_DIFF = "seed에 따라 부호 바뀜이 달라진다"
S5_SEED_SAME = "두 seed의 차이는 보이지 않는다"

# ---- 값이 없을 때의 표시 ----
MISSING = "아직 없음"
NOT_PAIRED = "같은 에피소드 짝 아님"

# ---- 비교 목록 ----
# 설정마다 (seed 3의 정책, seed 4의 정책)
SETTINGS = (("t_std", "std", "stds4"), ("t_demo_hist", "demohist", "demohists4"))
CELLS = ("demo1", "aisle", "rand", "head")
# t_demo_hist - t_std의 seed 조합: (이름, t_demo_hist 쪽 정책, t_std 쪽 정책)
COMBOS = (("3-3", "demohist", "std"), ("3-4", "demohist", "stds4"),
          ("4-3", "demohists4", "std"), ("4-4", "demohists4", "stds4"))
# 3번의 비교: (기호, 이름, 칸, 값의 출처, 규칙, 22절의 seed 3 값). 출처 "person"은 cross_person의 경로 위 무리
CHECKS = (("a", "aisle 성공", "aisle", "success", "ae", 4.30),
          ("b", "rand 성공", "rand", "success", "ae", -1.90),
          ("c", "head 성공", "head", "success", "ae", -1.42),
          ("d", "demo1 충돌", "demo1", "collision", "ae", 2.22),
          ("e", "demo1 건넘 경로 위 사람 충돌", "demo1", "person", "ae", -2.59),
          ("f", "demo1 성공", "demo1", "success", "f", 0.32))
# 4번: 칸과 22절의 seed 3 값(t_std - t_ctrl 성공)
STD_CTRL = (("demo1", 9.16), ("aisle", 12.55))
# 6번: 18절의 2×2에서 갈린 크기
SPLITS = (("head에서 이력의 몫", (-3.27, 3.03)), ("demo1 충돌에서 이력의 몫", (3.64, -2.27)))


# ---- 판정 문장 고르기 (입력은 차이와 SE뿐) ----
def outside(diff: float, se: float) -> bool:
    """2 SE 밖인지. cross_person.two_se의 정의를 그대로 쓴다."""
    return cp.two_se(diff, se) == "outside 2 SE"


def verdict_seed(diff: float, se: float) -> str:
    """1번: 한 칸의 seed 차이(성공)."""
    return S1_OUT if outside(diff, se) else S1_IN


def verdict_kept(vals) -> str:
    """3번 (a)~(e): 조합 넷의 (차이, SE). 부호는 2 SE 밖인 조합끼리만 견준다."""
    out = [d for d, se in vals if outside(d, se)]
    if any(d > 0 for d in out) and any(d < 0 for d in out):
        return S3_FLIP
    if len(out) == len(vals):
        return S3_KEEP
    return S3_SHRINK


def verdict_flat(vals) -> str:
    """3번 (f): 넷 모두 2 SE 안일 때만 문장을 쓰고, 아니면 빈 문자열(조합별 값만 적는다)."""
    return S3F_NONE if not any(outside(d, se) for d, se in vals) else ""


def judge(rule: str, vals) -> str:
    """3번의 규칙을 고른다: "ae"는 (a)~(e)의 규칙, "f"는 (f)의 규칙."""
    return verdict_kept(vals) if rule == "ae" else verdict_flat(vals)


def verdict_std_ctrl(diff: float, se: float) -> str:
    """4번: t_std_s4 - t_ctrl의 성공이 +2 SE를 넘는지."""
    return S4_KEEP if diff > 2 * se else S4_GONE


def verdict_flip_ctrl(diff: float, se: float) -> str:
    """5번 앞: t_std_s4 - t_ctrl의 분당 w 부호 바뀜."""
    if diff > 2 * se:
        return S5_UP
    if diff < -2 * se:
        return S5_DOWN
    return S5_FLAT


def verdict_flip_seed(diff: float, se: float) -> str:
    """5번 뒤: t_std_s4 - t_std의 분당 w 부호 바뀜."""
    return S5_SEED_DIFF if outside(diff, se) else S5_SEED_SAME


def spread(diffs) -> tuple[float, float]:
    """2번: |seed 차이|의 중앙값과 최댓값."""
    a = [abs(d) for d in diffs]
    return statistics.median(a), max(a)


def combined(vals, rule: str) -> str:
    """조합 넷의 판정 문장. 값이 모두 있을 때만 판정하고, 아니면 왜 판정하지 않는지 적는다."""
    nums = [v for v in vals if not isinstance(v, str)]
    if len(nums) == len(vals):
        return judge(rule, nums) or "판정 문장 없음 (조합별 값만 적는다)"
    if any(v.startswith(NOT_PAIRED) for v in vals if isinstance(v, str)):
        return "같은 에피소드 짝이 아닌 조합이 있어 판정하지 않는다"
    if not nums:
        return MISSING
    return f"일부만 있어 판정하지 않는다 ({len(nums)}/{len(vals)})"


# ---- 값 읽기: (차이, SE) 또는 값이 없는 까닭(문자열) ----
def read_paired(results) -> dict:
    """results/paired.csv를 (정책, 칸, 대조 정책, 대조 칸)으로 찾게 읽는다. 파일이 없으면 빈 dict."""
    p = pathlib.Path(results) / "paired.csv"
    if not p.exists():
        return {}
    with open(p, encoding="utf-8") as f:
        return {(r["policy"], r["cell"], r["vs_policy"], r["vs_cell"]): r for r in csv.DictReader(f)}


def table_value(table: dict, a: str, b: str, cell: str, col: str):
    """paired.csv에서 a - b의 col(success 또는 collision) 짝 차이와 SE."""
    r = table.get((a, cell, b, cell))
    if r is None:
        return MISSING
    if not r.get("same_episodes", "").startswith("yes"):
        return f"{NOT_PAIRED} ({r.get('same_episodes', '')})"
    d, se = r.get(f"{col}_diff_pp", ""), r.get(f"{col}_se_pp", "")
    if d == "" or se == "":
        return MISSING
    return finite(float(d), float(se))


def finite(diff: float, se: float):
    """차이와 SE가 모두 유한하면 그 둘, 아니면 판정하지 않는 까닭."""
    if not (np.isfinite(diff) and np.isfinite(se)):
        return f"값이 유한하지 않음 ({diff}, SE {se})"
    return diff, se


def eval_runs(runs) -> dict:
    """{(정책, 칸): 폴더}: queue_status.txt에서 끝난(exit=0) 평가만."""
    out = {}
    for tag in nt.finished(runs):
        m = nt.CELL_RE.match(tag)
        if m:
            out[(m.group(1), m.group(2))] = str(pathlib.Path(runs) / tag)
    return out


def person_value(ev: dict, a: str, b: str):
    """demo1에서 건넘이 경로 위인 에피소드의 사람 충돌 a - b (cross_person.compare)."""
    if (a, "demo1") not in ev or (b, "demo1") not in ev:
        return MISSING
    try:
        kind, why, rows = cp.compare(ev[(a, "demo1")], ev[(b, "demo1")])
    except Exception as e:  # 잘린 파일이나 빠진 항목: 이 비교만 비운다
        return f"읽지 못함 ({type(e).__name__})"
    if kind != "paired":
        return f"{NOT_PAIRED} ({why})"
    r = rows["cross_on_path"]
    if not (np.isfinite(r["diff_pp"]) and np.isfinite(r["se_pp"])):
        return f"SE 없음 (n {r['n']})"
    return r["diff_pp"], r["se_pp"]


def flip_value(ev: dict, a: str, b: str):
    """rand 칸의 분당 w 부호 바뀜 a - b (flip_rate.compare_runs)."""
    if (a, "rand") not in ev or (b, "rand") not in ev:
        return MISSING
    try:
        r, why = fr.compare_runs(ev[(a, "rand")], ev[(b, "rand")])
    except Exception as e:  # 잘린 파일이나 빠진 항목: 이 비교만 비운다
        return f"읽지 못함 ({type(e).__name__})"
    if r["kind"] != "paired":
        return f"{NOT_PAIRED} ({why})"
    return finite(r["diff"], r["se"])


# ---- Markdown ----
def fmt(v) -> str:
    """값 칸: "+1.23 (SE 0.45, 2 SE 밖)" 또는 값이 없는 까닭."""
    if isinstance(v, str):
        return v
    d, se = v
    return f"{d:+.2f} (SE {se:.2f}, 2 SE {'밖' if outside(d, se) else '안'})"


def single(v, verdict) -> str:
    """값 하나의 판정 문장. 값이 없으면 문장 없이 그 까닭."""
    return v if isinstance(v, str) else verdict(*v)


def section1(table: dict) -> tuple[list, list]:
    """1번 표와 2번에 쓸 성공 seed 차이 여덟 개(없으면 문자열)."""
    md = ["## 1. 칸마다의 seed 차이 (seed 4 - seed 3)\n",
          "| 설정 | 칸 | 성공 %p | 충돌 %p | 판정 |", "|---|---|---|---|---|"]
    succ = []
    for name, s3, s4 in SETTINGS:
        for cell in CELLS:
            sv = table_value(table, s4, s3, cell, "success")
            cv = table_value(table, s4, s3, cell, "collision")
            succ.append(sv)
            md.append(f"| {name} | {cell} | {fmt(sv)} | {fmt(cv)} | {single(sv, verdict_seed)} |")
    return md, succ


def section2(succ: list) -> tuple[list, float | None]:
    """2번: 여덟 개가 모두 있을 때만 중앙값과 최댓값."""
    nums = [v[0] for v in succ if not isinstance(v, str)]
    md = ["\n## 2. 실행 사이 차이의 어림 (|seed 차이|, 성공 %p)\n"]
    if len(nums) < len(succ):
        md.append(f"{MISSING} (여덟 개 가운데 {len(nums)}개만 있음)")
        return md, None
    med, mx = spread(nums)
    md.append(f"중앙값 {med:.2f} %p, 최댓값 {mx:.2f} %p (설정마다 seed 둘이라 거친 값)")
    return md, mx


def section3(table: dict, ev: dict) -> list:
    """3번: t_demo_hist - t_std를 seed 조합 넷으로."""
    md = ["\n## 3. 앞의 판정 다시 보기 (t_demo_hist - t_std, 조합은 t_demo_hist의 seed - t_std의 seed)\n",
          "| 비교 | 22절의 seed 3 값 | " + " | ".join(c for c, _, _ in COMBOS) + " | 판정 |",
          "|---|---|" + "---|" * len(COMBOS) + "---|"]
    for key, label, cell, src, rule, ref in CHECKS:
        vals = [person_value(ev, a, b) if src == "person" else table_value(table, a, b, cell, src)
                for _, a, b in COMBOS]
        md.append(f"| ({key}) {label} | {ref:+.2f} | " + " | ".join(fmt(v) for v in vals)
                  + f" | {combined(vals, rule)} |")
    return md


def section4(table: dict) -> list:
    """4번: t_std_s4 - t_ctrl의 성공, demo1과 aisle."""
    md = ["\n## 4. std 되돌리기의 효과 (t_std_s4 - t_ctrl, 성공 %p)\n",
          "| 칸 | 22절의 seed 3 값 | seed 4 | 판정 |", "|---|---|---|---|"]
    for cell, ref in STD_CTRL:
        v = table_value(table, "stds4", "ctrl", cell, "success")
        md.append(f"| {cell} | {ref:+.2f} | {fmt(v)} | {single(v, verdict_std_ctrl)} |")
    return md


def section5(ev: dict) -> list:
    """5번: rand의 분당 w 부호 바뀜, t_std_s4를 t_ctrl과 t_std에 견준다."""
    md = ["\n## 5. w 부호 바뀜 (rand, 회/분)\n"]
    for label, b, verdict in (("t_std_s4 - t_ctrl", "ctrl", verdict_flip_ctrl),
                              ("t_std_s4 - t_std", "std", verdict_flip_seed)):
        v = flip_value(ev, "stds4", b)
        md.append(f"- {label}: {fmt(v)}" + ("" if isinstance(v, str) else f". {verdict(*v)}"))
    return md


def section6(mx: float | None) -> list:
    """6번: 18절의 2×2에서 갈린 크기와 2번의 최댓값. 판정 문장은 없다."""
    md = ["\n## 6. 18절의 2×2와 나란히 (판정 문장 없음)\n"]
    md += [f"- {label}: {a:+.2f}, {b:+.2f}" for label, (a, b) in SPLITS]
    md.append(f"- 2번의 최댓값: {MISSING if mx is None else f'{mx:.2f}'}")
    return md


def main(argv) -> str:
    runs, results = argv[0], argv[1]
    table, ev = read_paired(results), eval_runs(runs)
    md = ["# 묶음 17: seed 차이 판정 (22절 기준)\n",
          f"평가 폴더 `{runs}`, 표 `{pathlib.Path(results) / 'paired.csv'}`. 차이는 앞 - 뒤, "
          "2 SE 밖은 |차이| > 2 × SE.\n"]
    m1, succ = section1(table)
    m2, mx = section2(succ)
    md += m1 + m2 + section3(table, ev) + section4(table) + section5(ev) + section6(mx)
    text = "\n".join(md) + "\n"
    print(text)
    return text


if __name__ == "__main__":
    main(sys.argv[1:])

import csv
import pathlib

import numpy as np
import pytest
import seed_spread as ss

COLS = ("policy", "cell", "vs_policy", "vs_cell", "n", "same_episodes", "success_diff_pp", "success_se_pp",
        "success_noise", "collision_diff_pp", "collision_se_pp", "collision_noise")
KEYS = np.array(["success", "collision", "oob", "timeout"])
KINDS = np.array(["wall_shelf", "box", "mover", "robot"])


def row(a, b, cell, s=(0.0, 1.0), c=(0.0, 1.0), same="yes (16 tracked envs)"):
    return {"policy": a, "cell": cell, "vs_policy": b, "vs_cell": cell, "n": 4096, "same_episodes": same,
            "success_diff_pp": s[0], "success_se_pp": s[1], "collision_diff_pp": c[0], "collision_se_pp": c[1]}


def write_paired(results, rows):
    results.mkdir(exist_ok=True)
    with open(results / "paired.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, COLS)
        w.writeheader()
        w.writerows(rows)


def fake_eval(runs, tag, hit, flips):
    """사람 충돌 여부(hit)와 분당 부호 바뀜을 정한 평가. 모든 에피소드는 건넘이 경로 위다."""
    d = runs / tag
    d.mkdir(parents=True)
    hit = np.array(hit, bool)
    n = len(hit)
    np.savez(d / "episodes.npz", keys=KEYS, hit_kinds=KINDS, env=np.arange(n), code=np.where(hit, 1, 0),
             hit_kind=np.where(hit, 2, 0), cross=np.ones(n, bool), cross_path=np.ones(n, bool),
             flips=np.full(n, float(flips)), length_s=np.full(n, 60.0))


def status(runs, tags):
    (runs / "queue_status.txt").write_text("".join(f"{t} exit=0 wall_s=1\n" for t in tags), encoding="utf-8")


def sentences():
    return [v for k, v in vars(ss).items() if k.startswith("S") and k[1:2].isdigit()]


# ---- 3번 (a)~(e)의 규칙 ----
def test_kept_when_all_four_are_outside_with_the_same_sign():
    assert ss.verdict_kept([(3, 1), (4, 1), (2.5, 1), (5, 1)]) == ss.S3_KEEP
    assert ss.verdict_kept([(-3, 1), (-4, 1), (-2.5, 1), (-5, 1)]) == ss.S3_KEEP


def test_flips_when_outside_combos_disagree_in_sign():
    assert ss.verdict_kept([(3, 1), (-3, 1), (0.1, 1), (4, 1)]) == ss.S3_FLIP
    # 넷 모두 밖이어도 부호가 갈리면 유지가 아니다
    assert ss.verdict_kept([(3, 1), (-3, 1), (4, 1), (5, 1)]) == ss.S3_FLIP


def test_shrinks_when_a_combo_is_inside_and_outside_ones_agree():
    assert ss.verdict_kept([(3, 1), (1, 1), (4, 1), (5, 1)]) == ss.S3_SHRINK


def test_sign_of_an_inside_combo_is_not_used():
    assert ss.verdict_kept([(3, 1), (-1, 1), (4, 1), (5, 1)]) == ss.S3_SHRINK
    assert ss.verdict_kept([(-3, 1), (1.9, 1), (-4, 1), (-5, 1)]) == ss.S3_SHRINK


def test_zero_difference_is_inside_and_has_no_sign():
    assert ss.verdict_kept([(3, 1), (0, 1), (4, 1), (5, 1)]) == ss.S3_SHRINK
    assert ss.verdict_kept([(3, 1), (0, 0), (4, 1), (5, 1)]) == ss.S3_SHRINK
    assert ss.verdict_kept([(0, 1), (0, 1), (0, 1), (0, 1)]) == ss.S3_SHRINK


def test_exactly_two_se_is_inside():
    assert ss.verdict_kept([(2, 1), (3, 1), (3, 1), (3, 1)]) == ss.S3_SHRINK
    assert ss.verdict_kept([(-2, 1), (3, 1), (3, 1), (3, 1)]) == ss.S3_SHRINK


# ---- 3번 (f)의 규칙 ----
def test_f_says_no_difference_only_when_all_four_are_inside():
    assert ss.verdict_flat([(0.3, 0.6), (-1.1, 0.6), (0, 0.6), (1.2, 0.6)]) == ss.S3F_NONE
    assert ss.verdict_flat([(0.3, 0.6), (-1.1, 0.6), (0, 0.6), (1.3, 0.6)]) == ""


def test_f_does_not_use_the_a_to_e_rule():
    flips = [(3, 1), (-3, 1), (4, 1), (5, 1)]
    kept = [(3, 1), (4, 1), (2.5, 1), (5, 1)]
    inside = [(1, 1), (-1, 1), (0.5, 1), (0, 1)]
    assert ss.judge("ae", flips) == ss.S3_FLIP and ss.judge("f", flips) == ""
    assert ss.judge("ae", kept) == ss.S3_KEEP and ss.judge("f", kept) == ""
    assert ss.judge("ae", inside) == ss.S3_SHRINK and ss.judge("f", inside) == ss.S3F_NONE
    # 표에서도 (f) 줄에는 (a)~(e)의 문장이 나오지 않는다
    assert ss.combined(flips, "f") == "판정 문장 없음 (조합별 값만 적는다)"


# ---- 1번, 4번, 5번 ----
def test_cell_seed_sentence():
    assert ss.verdict_seed(1.9, 1) == ss.S1_IN
    assert ss.verdict_seed(2, 1) == ss.S1_IN
    assert ss.verdict_seed(-2.1, 1) == ss.S1_OUT and ss.verdict_seed(2.1, 1) == ss.S1_OUT


def test_std_ctrl_sentence_needs_more_than_plus_two_se():
    assert ss.verdict_std_ctrl(2.1, 1) == ss.S4_KEEP
    assert ss.verdict_std_ctrl(2, 1) == ss.S4_GONE
    assert ss.verdict_std_ctrl(-5, 1) == ss.S4_GONE


def test_flip_rate_sentences():
    assert ss.verdict_flip_ctrl(2.1, 1) == ss.S5_UP
    assert ss.verdict_flip_ctrl(0.5, 1) == ss.S5_FLAT
    assert ss.verdict_flip_ctrl(-2, 1) == ss.S5_FLAT and ss.verdict_flip_ctrl(2, 1) == ss.S5_FLAT
    assert ss.verdict_flip_ctrl(-2.1, 1) == ss.S5_DOWN
    assert ss.verdict_flip_seed(-2.1, 1) == ss.S5_SEED_DIFF and ss.verdict_flip_seed(2.1, 1) == ss.S5_SEED_DIFF
    assert ss.verdict_flip_seed(2, 1) == ss.S5_SEED_SAME


# ---- 2번 ----
def test_spread_is_median_and_max_of_absolute_differences():
    med, mx = ss.spread([-5, 1, 2, -3, 0.5, 0.2, -4, 0.1])
    assert med == pytest.approx(1.5) and mx == 5


def test_spread_waits_for_all_eight():
    md, mx = ss.section2([(1.0, 1.0)] * 7 + [ss.MISSING])
    assert mx is None and ss.MISSING in "\n".join(md)
    md, mx = ss.section2([(d, 1.0) for d in (-5, 1, 2, -3, 0.5, 0.2, -4, 0.1)])
    assert mx == 5 and "중앙값 1.50 %p, 최댓값 5.00 %p" in "\n".join(md)


# ---- 값이 없거나 일부만 있거나 짝이 아닐 때 ----
def test_nothing_yet_does_not_crash_and_writes_no_sentence(tmp_path):
    out = ss.main([str(tmp_path / "no_runs"), str(tmp_path / "no_results")])
    assert out.count(ss.MISSING) > 20
    assert not [s for s in sentences() if s in out]


def test_partial_combos_get_no_sentence(tmp_path):
    write_paired(tmp_path, [row("demohist", "std", c, s=(4, 0.5), c=(4, 0.5)) for c in ss.CELLS])
    lines = ss.section3(ss.read_paired(tmp_path), {})
    body = [ln for ln in lines if ln.startswith("| (")]
    assert len(body) == 6
    for ln in body[:4] + body[5:]:
        assert ln.endswith("| 일부만 있어 판정하지 않는다 (1/4) |")
        assert "+4.00 (SE 0.50, 2 SE 밖)" in ln
    # (e)는 평가 폴더가 없어 넷 모두 없다
    assert body[4].endswith(f"| {ss.MISSING} |")
    assert not [s for s in sentences() if s in "\n".join(lines)]


def test_rows_not_on_the_same_episodes_are_reported_and_not_judged(tmp_path):
    bad = "different start episodes"
    rows = [row(a, b, c, s=(4, 0.5), c=(4, 0.5)) for _, a, b in ss.COMBOS for c in ss.CELLS]
    rows = [dict(r, same_episodes=bad, success_diff_pp="", success_se_pp="")
            if (r["policy"], r["vs_policy"]) == ("demohists4", "stds4") else r for r in rows]
    rows += [row("stds4", "std", c, same=bad) | {"success_diff_pp": ""} for c in ss.CELLS]
    rows += [row("stds4", "ctrl", "demo1", same=bad)]
    write_paired(tmp_path, rows)
    t = ss.read_paired(tmp_path)
    assert ss.table_value(t, "stds4", "std", "rand", "success") == f"{ss.NOT_PAIRED} ({bad})"
    s1 = "\n".join(ss.section1(t)[0])
    assert f"| {ss.NOT_PAIRED} ({bad}) |" in s1 and ss.S1_IN not in s1 and ss.S1_OUT not in s1
    s3 = [ln for ln in ss.section3(t, {}) if ln.startswith("| (a)")][0]
    assert f"{ss.NOT_PAIRED} ({bad})" in s3
    assert s3.endswith("| 같은 에피소드 짝이 아닌 조합이 있어 판정하지 않는다 |")
    s4 = "\n".join(ss.section4(t))
    assert ss.S4_KEEP not in s4 and ss.S4_GONE not in s4 and ss.NOT_PAIRED in s4
    # 짝이 아닌 줄이 낀 2번은 값을 내지 않는다
    assert ss.section2(ss.section1(t)[1])[1] is None


def test_non_finite_values_get_no_sentence(tmp_path, monkeypatch):
    write_paired(tmp_path, [row("stds4", "std", "demo1", s=(0.5, "nan")),
                            row("stds4", "ctrl", "demo1", s=("nan", 1.0))])
    t = ss.read_paired(tmp_path)
    assert isinstance(ss.table_value(t, "stds4", "std", "demo1", "success"), str)
    s1, s4 = "\n".join(ss.section1(t)[0]), "\n".join(ss.section4(t))
    assert ss.S1_IN not in s1 and ss.S4_GONE not in s4
    monkeypatch.setattr(ss.fr, "compare_runs", lambda a, b: ({"kind": "paired", "diff": 0.1, "se": float("nan")}, ""))
    s5 = "\n".join(ss.section5({("stds4", "rand"): "a", ("ctrl", "rand"): "b", ("std", "rand"): "c"}))
    assert not [s for s in sentences() if s in s5]


def test_sentences_are_the_words_of_section_22():
    text = pathlib.Path(__file__).resolve().parents[1] / "worklog_rl_tracks_2026-11-26.md"
    text = text.read_text(encoding="utf-8")
    sec = text[text.index("## 22. 묶음 17"):text.index("## 9a.")]
    flat = " ".join(ln.strip() for ln in sec.splitlines())
    # 상수마다 22절의 그 조건 뒤에 오는 문장이어야 한다 (다른 갈래의 문장과 바뀌면 떨어진다)
    words = {"S1_IN": '2 SE 안이면 "이 칸에서는 seed를 바꿔도 평가 오차 안이다"',
             "S1_OUT": '2 SE 밖이면 "seed만 바꿔도 평가 오차 밖으로 달라진다"',
             "S3_KEEP": '넷 모두 2 SE 밖이고 부호가 같으면 "seed를 바꿔도 유지된다"',
             "S3_FLIP": '부호가 서로 다른 것이 있으면 "seed에 따라 뒤집힌다"',
             "S3_SHRINK": '2 SE 밖인 조합끼리는 부호가 같다) "평가 오차 안으로 줄어드는 조합이 있다"',
             "S3F_NONE": '넷 모두 2 SE 안이면 "seed를 바꿔도 차이가 보이지 않는다"',
             "S4_KEEP": '+2 SE를 넘는 칸은 "seed를 바꿔도 유지된다"',
             "S4_GONE": '넘지 못하는 칸은 "seed 4에서는 보이지 않는다"',
             "S5_UP": '+2 SE를 넘으면 "seed 4에서도 t_ctrl보다 는다"',
             "S5_FLAT": '2 SE 안이면 "seed 4에서는 t_ctrl과의 차이가 보이지 않는다"',
             "S5_DOWN": '-2 SE보다 낮으면 "seed 4에서는 t_ctrl보다 준다"',
             "S5_SEED_DIFF": '2 SE 밖이면 "seed에 따라 부호 바뀜이 달라진다"',
             "S5_SEED_SAME": '안이면 "두 seed의 차이는 보이지 않는다"'}
    assert sorted(words) == sorted(k for k in vars(ss) if k.startswith("S") and k[1:2].isdigit())
    for name, w in words.items():
        assert w in flat, name
        assert w.endswith(f'"{getattr(ss, name)}"'), name
    # seed 조합은 t_demo_hist의 seed - t_std의 seed
    assert [c[1:] for c in ss.COMBOS] == [("demohist", "std"), ("demohist", "stds4"),
                                          ("demohists4", "std"), ("demohists4", "stds4")]
    assert ss.SETTINGS == (("t_std", "std", "stds4"), ("t_demo_hist", "demohist", "demohists4"))


def test_eval_comparisons_not_on_the_same_episodes_are_not_judged(tmp_path, monkeypatch):
    for tag in ("e_demohist_demo1", "e_std_demo1", "e_stds4_rand", "e_ctrl_rand"):
        fake_eval(tmp_path, tag, [0, 1] * 20, 3)
    status(tmp_path, ["e_demohist_demo1", "e_std_demo1", "e_stds4_rand", "e_ctrl_rand"])
    monkeypatch.setattr(ss.nt, "check_same", lambda a, b: ("different start episodes", 0))
    ev = ss.eval_runs(tmp_path)
    assert ss.person_value(ev, "demohist", "std") == f"{ss.NOT_PAIRED} (different start episodes)"
    s5 = "\n".join(ss.section5(ev))
    assert f"t_std_s4 - t_ctrl: {ss.NOT_PAIRED} (different start episodes)" in s5
    assert not [s for s in sentences() if s in s5]


def test_unfinished_eval_is_treated_as_missing(tmp_path):
    fake_eval(tmp_path, "e_stds4_rand", [0] * 10, 3)
    fake_eval(tmp_path, "e_ctrl_rand", [0] * 10, 3)
    (tmp_path / "queue_status.txt").write_text("e_stds4_rand exit=0\ne_stds4_rand exit=1\ne_ctrl_rand exit=0\n",
                                               encoding="utf-8")
    assert ss.flip_value(ss.eval_runs(tmp_path), "stds4", "ctrl") == ss.MISSING


# ---- main을 끝까지 ----
def test_main_writes_the_sentences(tmp_path, monkeypatch):
    results, runs = tmp_path / "results", tmp_path / "runs"
    # 1번: stds4 - std의 demo1만 2 SE 밖. 2번의 |차이|: 3, 0.5 ×3, 1, 0.2, 0.1, 0.4
    s1 = {("stds4", "demo1"): 3, ("stds4", "aisle"): 0.5, ("stds4", "rand"): -0.5, ("stds4", "head"): 0.5,
          ("demohists4", "demo1"): 1, ("demohists4", "aisle"): -0.2, ("demohists4", "rand"): 0.1,
          ("demohists4", "head"): 0.4}
    rows = [row(a, b, c, s=(s1[(a, c)], 1)) for _, b, a in ss.SETTINGS for c in ss.CELLS]
    # 3번: (a) 유지, (b) 뒤집힘, (c) 줄어듦, (d) 충돌 유지, (f) 넷 모두 안
    rand = {"3-3": -2.5, "3-4": 2.5, "4-3": -3, "4-4": 0.1}
    head = {"3-3": -2.5, "3-4": -0.1, "4-3": -3, "4-4": 1.0}
    for name, a, b in ss.COMBOS:
        rows += [row(a, b, "aisle", s=(4, 0.5)), row(a, b, "rand", s=(rand[name], 1)),
                 row(a, b, "head", s=(head[name], 1)), row(a, b, "demo1", s=(0.3, 0.6), c=(2.5, 0.6))]
    # 4번: demo1은 유지, aisle은 보이지 않는다
    rows += [row("stds4", "ctrl", "demo1", s=(5, 1)), row("stds4", "ctrl", "aisle", s=(-5, 1))]
    write_paired(results, rows)
    # 3번 (e): 이력 정책은 사람 충돌 0 %, std는 50 %. 5번: stds4와 std는 분당 4, ctrl은 분당 1
    for pol, hit, flips in (("demohist", 0, 1), ("demohists4", 0, 1), ("std", 1, 4), ("stds4", 1, 4)):
        fake_eval(runs, f"e_{pol}_demo1", [0, hit] * 20, flips)
    for pol, flips in (("stds4", 4), ("std", 4), ("ctrl", 1)):
        fake_eval(runs, f"e_{pol}_rand", [0] * 40, flips)
    status(runs, [p.name for p in runs.iterdir()])
    monkeypatch.setattr(ss.nt, "check_same", lambda a, b: ("", 16))
    out = ss.main([str(runs), str(results)])
    lines = out.splitlines()

    def line(prefix):
        return [ln for ln in lines if ln.startswith(prefix)][0]

    assert line("| t_std | demo1 |").endswith(f"| {ss.S1_OUT} |")
    assert line("| t_demo_hist | head |").endswith(f"| {ss.S1_IN} |")
    assert "중앙값 0.50 %p, 최댓값 3.00 %p" in out
    assert line("| (a)").endswith(f"| {ss.S3_KEEP} |")
    assert line("| (b)").endswith(f"| {ss.S3_FLIP} |")
    assert line("| (c)").endswith(f"| {ss.S3_SHRINK} |")
    assert line("| (d)").endswith(f"| {ss.S3_KEEP} |")
    assert line("| (e)").endswith(f"| {ss.S3_KEEP} |") and "-50.00" in line("| (e)")
    assert line("| (f)").endswith(f"| {ss.S3F_NONE} |")
    assert line("| demo1 |").endswith(f"| {ss.S4_KEEP} |")
    assert line("| aisle |").endswith(f"| {ss.S4_GONE} |")
    assert line("- t_std_s4 - t_ctrl:").endswith(ss.S5_UP) and "+3.00" in line("- t_std_s4 - t_ctrl:")
    assert line("- t_std_s4 - t_std:").endswith(ss.S5_SEED_SAME)
    assert "- 2번의 최댓값: 3.00" in out and "-3.27, +3.03" in out and "+3.64, -2.27" in out

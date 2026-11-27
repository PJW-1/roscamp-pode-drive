"""gz_gate17.py가 39절의 관문 숫자를 저장된 결과에서 그대로 다시 세는지 본다."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import gz_gate17 as gg  # noqa: E402

RESULTS = pathlib.Path(__file__).parent / "results" / "gz_demo_sym17"


def test_gate_numbers_of_batch_17():
    rows, meets = gg.read([RESULTS / "a", RESULTS / "b"])
    cells = gg.per_cell(rows)
    f_ok, tot, d_arr, d_col, verdict = gg.gate(cells)
    assert f_ok
    assert tot[gg.CTRL] == [25, 16, 19] and tot[gg.SYM] == [25, 18, 10]
    assert (d_arr, d_col) == (2, -9)
    assert verdict == "이 판 수로는 차이가 보이지 않는다"
    assert cells[("g_demo", gg.SYM)]["ok"] == 10 and cells[("g_demo", gg.CTRL)]["ok"] == 3


def test_meet_counts_of_batch_17():
    _, meets = gg.read([RESULTS / "a", RESULTS / "b"])
    m = gg.meet_counts(meets)
    for sc in ("k_cross", "j_cross"):
        for c in ("RLShimDemoHist", "RLShimDemoHistSym"):
            assert m[(sc, c)] == [5, 5, 0, 5]
    assert m[("g_demo", "RLShimDemoHistSym")] == [10, 10, 10, 0]


def test_gate_says_worse_when_collisions_rise():
    cells = {(s, c): {"n": 5, "ok": 3, "corridor": 5, "col": 1}
             for s in gg.SCENARIOS for c in (gg.CTRL, gg.SYM)}
    cells[("f_cart", gg.SYM)]["col"] = 0
    cells[("g_demo", gg.SYM)]["col"] = 5
    cells[("k_cross", gg.SYM)]["col"] = 5
    assert gg.gate(cells)[4] == "Gazebo에서는 나빠졌다"

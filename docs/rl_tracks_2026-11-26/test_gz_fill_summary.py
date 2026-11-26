"""gz_fill_summary.py의 셈을 저장소에 있는 두 묶음의 결과 파일로 고정한다."""
import pathlib

import gz_fill_summary as gs

RESULTS = pathlib.Path(__file__).parent / "results"
CONTROL, FILL = "FollowPathRLShimDemoHist", "FollowPathRLShimDemoHistFill"


def test_fisher_matches_hand_values():
    # 5/5 대 2/5: 한쪽 꼬리 C(5,5)C(5,2)/C(10,7) = 10/120, 양쪽은 그 두 배
    assert abs(gs.fisher_two_sided(5, 0, 2, 3) - 20 / 120) < 1e-12
    assert abs(gs.fisher_two_sided(3, 2, 3, 2) - 1.0) < 1e-12


def test_batch_15_counts():
    s = gs.summarize(gs.read_folder(RESULTS / "gz_demo_fill15"))
    assert (s[CONTROL]["ok"], s[CONTROL]["n"]) == (5, 5)
    assert (s[FILL]["ok"], s[FILL]["n"], s[FILL]["aborted"]) == (2, 5, 3)
    assert (s[FILL]["narrow"], s[FILL]["narrow_touch"]) == (4, 3)
    assert (s[CONTROL]["narrow"], s[CONTROL]["narrow_touch"]) == (2, 0)
    assert s[FILL]["contact_deg"] == [63, 63, 64]


def test_batch_16_counts_and_the_merged_table():
    runs15 = gs.read_folder(RESULTS / "gz_demo_fill15")
    runs16 = gs.read_folder(RESULTS / "gz_demo_fill16")
    s = gs.summarize(runs16)
    assert (s[CONTROL]["ok"], s[FILL]["ok"]) == (6, 7)
    assert (s[CONTROL]["cart_touch"], s[FILL]["cart_touch"]) == (2, 2)
    assert (s[CONTROL]["person_touch"], s[FILL]["person_touch"]) == (9, 3)
    merged = {(arm, (0, rep)): v for (arm, rep), v in runs15.items()}
    merged.update({(arm, (1, rep)): v for (arm, rep), v in runs16.items()})
    m = gs.summarize(merged)
    assert (m[CONTROL]["ok"], m[CONTROL]["n"], m[FILL]["ok"], m[FILL]["n"]) == (11, 15, 9, 15)
    assert (m[CONTROL]["narrow"], m[FILL]["narrow"]) == (5, 9)
    p = gs.fisher_two_sided(m[CONTROL]["ok"], 4, m[FILL]["ok"], 6)
    assert abs(p - 0.70) < 0.005

import bearing_before_hit as bb
import pytest


def test_bearing_is_zero_ahead_ninety_to_the_side_and_180_behind():
    assert bb.abs_bearing_deg((0, 0), (0.1, 0), (0.5, 0)) == pytest.approx(0)
    assert bb.abs_bearing_deg((0, 0), (0.1, 0), (0.1, 0.3)) == pytest.approx(90)
    assert bb.abs_bearing_deg((0, 0), (0.1, 0), (0.1, -0.3)) == pytest.approx(90)
    assert bb.abs_bearing_deg((0, 0), (0.1, 0), (-0.4, 0)) == pytest.approx(180)
    assert bb.abs_bearing_deg((0, 0), (0, 0.1), (0.2, 0.3)) == pytest.approx(45)


def test_bearing_turns_around_when_the_robot_was_reversing():
    assert bb.abs_bearing_deg((0, 0), (0.1, 0), (0.5, 0), reverse=True) == pytest.approx(180)
    assert bb.abs_bearing_deg((0, 0), (0.1, 0), (-0.4, 0), reverse=True) == pytest.approx(0)


def test_bearing_is_unknown_when_the_robot_barely_moved():
    assert bb.abs_bearing_deg((0, 0), (0.004, 0), (1, 0)) is None


def test_bins_split_at_30_60_90_with_the_edge_in_the_lower_bin():
    assert bb.bins([0, 30, 30.1, 60, 61, 90, 91, 180]) == [2, 2, 2, 2]
    assert bb.bins([]) == [0, 0, 0, 0]


def test_reversing_reads_only_the_commands_that_made_the_displacement():
    v = [-1.0, 0.1, 0.1, 0.1]  # 위치 [t - w]는 명령 [t - w]가 끝난 뒤의 자리다: 그 명령은 변위에 들지 않는다
    assert bb.reversing(v, t=3, w=3) is False
    assert bb.reversing([0.1, -0.1, -0.1, -0.1], t=3, w=3) is True
    assert bb.reversing([0.1, 0.1, -0.05, -0.05], t=3, w=2) is True
    assert bb.reversing([0.1, 0.1, 0.1, -1.0], t=3, w=3) is True  # 스텝 t의 명령은 변위에 든다

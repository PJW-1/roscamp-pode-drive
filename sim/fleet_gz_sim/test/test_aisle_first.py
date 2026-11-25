"""통로 먼저 규칙(aisle_first.py): ROS 없이 상태 전이만 본다."""
from fleet_gz_sim.aisle_first import (
    AISLE,
    DETOUR,
    NONE,
    OBSTACLES_OFF,
    OBSTACLES_ON,
    Rules,
    State,
    log_line,
    navigating,
    step,
)

RULES = Rules()


def run(samples, state=None):
    """samples: (시각, xy, 주행 중) 목록. (마지막 상태, 할 일 목록)."""
    state = state or State()
    actions = []
    for now, xy, moving in samples:
        state, action = step(state, now, xy, moving, RULES)
        actions.append(action)
    return state, actions


def still(t0, t1, xy=(1.0, 0.5), dt=0.5, moving=True):
    return [(t0 + i * dt, xy, moving) for i in range(int(round((t1 - t0) / dt)) + 1)]


def test_standing_still_while_navigating_turns_the_detour_on_after_stall_s():
    state, actions = run(still(0.0, 4.5))
    assert state.mode == AISLE and set(actions) == {NONE}
    state, actions = run([(5.0, (1.0, 0.5), True)], state)
    assert state.mode == DETOUR and actions == [OBSTACLES_ON]
    assert state.stall_xy == (1.0, 0.5)
    assert log_line(state, (1.0, 0.5)) == (
        "aisle_first: detour on at (1.00, 0.50) after 5.0 s without progress")


def test_small_creeping_does_not_reset_but_real_progress_does():
    # 0.04 m만 움직이면 진척이 아니다
    state, actions = run([(0.0, (1.0, 0.5), True), (3.0, (1.04, 0.5), True),
                          (5.0, (1.03, 0.5), True)])
    assert actions[-1] == OBSTACLES_ON
    # 0.06 m 움직이면 기준이 그 자리로 옮겨 다시 센다
    state, actions = run([(0.0, (1.0, 0.5), True), (3.0, (1.06, 0.5), True),
                          (5.0, (1.06, 0.5), True), (7.9, (1.06, 0.5), True)])
    assert state.mode == AISLE and set(actions) == {NONE}
    assert state.ref == (1.06, 0.5) and state.ref_t == 3.0
    state, actions = run([(8.0, (1.06, 0.5), True)], state)
    assert actions == [OBSTACLES_ON]


def test_time_while_not_navigating_does_not_count():
    samples = still(0.0, 3.0) + still(3.5, 20.0, moving=False) + still(20.5, 24.0)
    state, actions = run(samples)
    assert state.mode == AISLE and set(actions) == {NONE}
    # 주행이 다시 시작되면 그때부터 센다
    state, actions = run([(25.5, (1.0, 0.5), True)], state)
    assert actions == [OBSTACLES_ON]


def test_unknown_position_never_triggers():
    state, actions = run([(t, None, True) for t in range(0, 30)])
    assert state.mode == AISLE and set(actions) == {NONE}
    # 위치를 모르는 동안에는 시간이 지나도 켜지지 않는다
    state, actions = run(still(0.0, 4.0) + [(t, None, True) for t in range(5, 20)])
    assert state.mode == AISLE and set(actions) == {NONE}


def test_a_short_position_gap_keeps_the_stall_already_counted():
    state, actions = run(still(0.0, 4.0) + [(4.5, None, True), (5.0, (1.0, 0.5), True)])
    assert actions[-2:] == [NONE, OBSTACLES_ON]


def detour_state():
    state, actions = run(still(0.0, 5.0))
    assert actions[-1] == OBSTACLES_ON
    return state


def test_detour_ends_when_the_robot_is_resume_m_from_where_it_stalled():
    state, actions = run([(6.0, (1.0, 0.85), True), (7.0, (1.0, 0.90), True)], detour_state())
    assert state.mode == DETOUR and actions == [NONE, NONE]
    state, actions = run([(8.0, (1.0, 0.91), True)], state)
    assert state.mode == AISLE and actions == [OBSTACLES_OFF]
    assert log_line(state, (1.0, 0.91)) == (
        "aisle_first: back to aisle mode (moved 0.41 m) at (1.00, 0.91)")
    # 통로 모드로 돌아온 자리에서 다시 센다
    assert state.ref == (1.0, 0.91) and state.ref_t == 8.0


def test_detour_ends_when_navigation_ends():
    state, actions = run([(6.0, (1.0, 0.5), True), (7.0, (1.0, 0.5), False)], detour_state())
    assert state.mode == AISLE and actions == [NONE, OBSTACLES_OFF]
    assert log_line(state, (1.0, 0.5)).startswith(
        "aisle_first: back to aisle mode (navigation ended)")
    # 다음 주행은 새 기준으로 시작한다
    state, actions = run(still(8.0, 12.5), state)
    assert set(actions) == {NONE}


def test_detour_stays_on_while_stalled_or_position_unknown():
    state, actions = run(still(6.0, 60.0) + [(61.0, None, True)], detour_state())
    assert state.mode == DETOUR and set(actions) == {NONE}


def test_navigating_means_a_plan_within_the_timeout():
    assert not navigating(None, 10.0, RULES)
    assert navigating(7.5, 10.0, RULES)
    assert not navigating(7.4, 10.0, RULES)
    assert not navigating(11.0, 10.0, RULES)  # 시계가 되돌아가면(시뮬 재시작) 주행 아님


def test_stall_stays_below_the_controller_progress_timeout():
    assert RULES.stall_s < 10.0 and RULES.progress_m == 0.05

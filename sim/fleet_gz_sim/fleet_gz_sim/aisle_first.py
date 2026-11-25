"""통로 먼저(시뮬 시험): 전역 costmap의 장애물 층을 끈 채 통로 안에서 지나가 보고, 못 지나가면 켠다.
ROS 없이 규칙만. 시각은 밖에서 넣는다(노드 시계라 sim time을 따른다).
  AISLE: 장애물 층 끔(시작). 주행 중에 stall_s 동안 progress_m 넘게 못 움직이면 DETOUR.
  DETOUR: 장애물 층 켬. 멈춘 곳에서 resume_m 넘게 벗어나거나 주행이 끝나거나 새 goal이 오면 AISLE.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

AISLE = "aisle"
DETOUR = "detour"
NONE = "none"
OBSTACLES_ON = "obstacles_on"
OBSTACLES_OFF = "obstacles_off"


@dataclass(frozen=True)
class Rules:
    plan_timeout_s: float = 2.5
    progress_m: float = 0.05
    stall_s: float = 5.0
    resume_m: float = 0.40
    goal_change_m: float = 0.20


@dataclass(frozen=True)
class State:
    """ref, ref_t: 진척을 재는 기준 위치와 그때 시각. stall_xy: 멈춘 곳(DETOUR에서만). reason: 마지막 모드 변경의 까닭."""
    mode: str = AISLE
    ref: tuple[float, float] | None = None
    ref_t: float = 0.0
    stall_xy: tuple[float, float] | None = None
    reason: str = ""


def navigating(last_plan_t: float | None, now: float, rules: Rules = Rules()) -> bool:
    """plan_timeout_s 안에 전역 경로(plan)를 받았으면 주행 중이다."""
    return last_plan_t is not None and 0.0 <= now - last_plan_t <= rules.plan_timeout_s


def goal_changed(prev: tuple[float, float] | None, goal: tuple[float, float] | None,
                 rules: Rules = Rules()) -> bool:
    """전역 경로의 끝점이 goal_change_m 넘게 옮겨졌으면 새 goal이다. 처음 받은 경로와 빈 경로는 아니다."""
    return prev is not None and goal is not None and math.dist(prev, goal) > rules.goal_change_m


def step(state: State, now: float, xy: tuple[float, float] | None, moving: bool,
         rules: Rules = Rules(), new_goal: bool = False) -> tuple[State, str]:
    """(새 상태, 할 일) 할 일은 NONE, OBSTACLES_ON, OBSTACLES_OFF."""
    # ---- 새 goal: 통로 모드에서 다시 시작한다 (앞 goal의 우회가 다음 goal로 넘어가지 않게) ----
    if new_goal:
        fresh = State(ref=xy, ref_t=now, reason="new goal")
        return fresh, (OBSTACLES_OFF if state.mode == DETOUR else NONE)
    # ---- DETOUR: 벗어났거나 주행이 끝났으면 통로 모드로 ----
    if state.mode == DETOUR:
        if not moving:
            return State(reason="navigation ended"), OBSTACLES_OFF
        if xy is not None and state.stall_xy is not None:
            away = math.dist(xy, state.stall_xy)
            if away > rules.resume_m:
                return State(ref=xy, ref_t=now, reason=f"moved {away:.2f} m"), OBSTACLES_OFF
        return state, NONE
    # ---- AISLE: 주행 중이 아니면 기준을 버리고, 위치를 모르면 판단을 미룬다 ----
    if not moving:
        return replace(state, ref=None), NONE
    if xy is None:
        return state, NONE
    if state.ref is None or now < state.ref_t or math.dist(xy, state.ref) > rules.progress_m:
        return replace(state, ref=xy, ref_t=now), NONE
    stalled = now - state.ref_t
    if stalled >= rules.stall_s:
        return (State(mode=DETOUR, stall_xy=xy,
                      reason=f"after {stalled:.1f} s without progress"), OBSTACLES_ON)
    return state, NONE


def log_line(state: State, xy: tuple[float, float] | None) -> str:
    """모드가 바뀐 뒤 남기는 INFO 한 줄(벤치마크가 이 문구를 찾는다)."""
    if state.mode == DETOUR:
        x, y = state.stall_xy
        return f"aisle_first: detour on at ({x:.2f}, {y:.2f}) {state.reason}"
    where = "" if xy is None else f" at ({xy[0]:.2f}, {xy[1]:.2f})"
    return f"aisle_first: back to aisle mode ({state.reason}){where}"

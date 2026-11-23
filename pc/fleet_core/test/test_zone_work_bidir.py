"""WORK 양방향 진입(배치 v7_bidir, 2026-11-23 시뮬 시험).

- 들어갈 쪽: 줄마다 맨 앞 빈 자리의 길 값(거리 + 회전 extra_m) + 그 줄에 앞선 로봇 수 x dock_wait_estimate_s.
  빈 경기장에서는 남쪽 줄(Q1), 남쪽 줄에 로봇이 서 있으면 북쪽 줄(M_E).
- 줄에 선 로봇은 자기 줄로만 W에 들어간다(M_E에서 Q2, Q1을 지나 들어가지 않는다).
- 도크 차례는 줄 앞자리끼리 먼저 바란 로봇이 먼저다.
- 나갈 때: 지금 방향에서 도는 시간을 길 값에 더하고(exit_turn_cost), 자기 줄에 선 로봇이 쥔 자리로는
  나가지 않는다(마주 봄 금지). 그 길밖에 없으면 W에서 기다린다.
"""
import math
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from zone_harness import DT, Harness

NORTH, SOUTH = -math.pi / 2, math.pi / 2


def mode(order):
    return replace(load_params(), use_order_model=order)


ORDER = pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])


def targets(world, robot):
    return [c.target_id for c in world.legs(robot) if c.mode != "retreat"]


def left_by(world, robot):
    """W에서 나간 쪽: 급행(J_E를 받았다)이면 W_X, 아니면 처음 간 지점."""
    if any(e.code == "ZONE_GRANTED" and e.robot == robot and "J_E" in e.detail
           for e in world.events):
        return "W_X"
    return targets(world, robot)[0]


def stands_at(world, robot, node):
    bot = world.bots[robot]
    return lambda: (world.view(robot)["node"] == node and bot.cmd is None
                    and (bot.run is None or bot.run.waiting))


def work_busy(starts, robots, order, yaw=SOUTH, faces=None):
    """pode1이 W에서 일한다(풀어 줄 때까지). robots가 그 차례로 WORK를 바란다.
    yaw: pode1이 W에서 보는 방향(SOUTH = 북쪽을 본다, 남쪽에서 들어왔다)."""
    world = Harness(layout="v7_bidir", starts=starts, params=mode(order))
    world.bots["pode1"].yaw = yaw
    for robot, heading in (faces or {}).items():
        world.bots[robot].yaw = heading
    world.report()
    busy = world.call("WORK", "pode1")
    calls = []
    for robot in robots:
        world.now += DT / 2
        calls.append(world.call("WORK", robot))
    assert world.run_until(lambda: world.result_of(busy) is not None, 30.0), world.log[-10:]
    return world, calls


@ORDER
def test_an_empty_yard_joins_the_south_line(order):
    world, _ = work_busy({"pode1": "W", "pode2": "P-2", "pode3": "O_OUT"}, ["pode3"], order)
    assert world.run_until(stands_at(world, "pode3", "Q1"), 90.0), world.log[-10:]
    assert "M_E" not in targets(world, "pode3")
    assert not world.problems


@ORDER
def test_with_the_south_line_taken_a_robot_waits_on_the_north_side(order):
    world, (south, north) = work_busy({"pode1": "W", "pode2": "Q1", "pode3": "O_OUT"},
                                      ["pode2", "pode3"], order)
    assert world.run_until(stands_at(world, "pode3", "M_E"), 90.0), world.log[-10:]
    assert world.view("pode2")["node"] == "Q1"
    assert any(e.code == "DOCK_QUEUED" and e.robot == "pode3" and "M_E" in e.detail
               for e in world.events), world.log[-10:]
    # 먼저 바란 남쪽 앞자리가 먼저 들어가고, pode1은 북쪽(M_E)에 선 로봇 쪽으로 나가지 않는다
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(south) is not None, 90.0), world.log[-10:]
    assert world.result_of(south).robot_id == "pode2" and world.result_of(north) is None
    assert "M_E" not in targets(world, "pode1"), world.log[-20:]
    assert math.degrees(world.bots["pode2"].yaw) == pytest.approx(90.0)
    # 그다음 북쪽 줄이 자기 줄(work_in_north)로 들어가 남쪽을 보고 선다
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(north) is not None, 120.0), world.log[-10:]
    assert targets(world, "pode3")[-1] == "W" and "Q1" not in targets(world, "pode3")
    assert "Q2" not in targets(world, "pode3")
    assert math.degrees(world.bots["pode3"].yaw) == pytest.approx(-90.0)
    assert not world.problems


@ORDER
def test_the_line_that_asked_first_docks_first(order):
    world, (north, south) = work_busy({"pode1": "W", "pode2": "Q1", "pode3": "M_E"},
                                      ["pode3", "pode2"], order)
    world.run(5.0)
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(north) is not None, 90.0), world.log[-10:]
    assert world.result_of(south) is None
    assert targets(world, "pode3") == ["W"], "by its own line, not through Q2 and Q1"
    assert not world.problems


@ORDER
@pytest.mark.parametrize(("yaw", "way"), [(SOUTH, "W_X"), (NORTH, "M_E")],
                         ids=["faces_north", "faces_south"])
def test_leaving_w_prices_the_turn_from_where_it_faces(order, yaw, way):
    """북쪽을 본 로봇은 급행(돌기 0), 남쪽을 본 로봇은 M_E 쪽(104도, 급행은 180도)으로 나간다."""
    world, _ = work_busy({"pode1": "W", "pode2": "P-2", "pode3": "P-1"}, [], order, yaw=yaw)
    world.traffic.on_release("pode1")
    world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: len(targets(world, "pode1")) > 0, 30.0), world.log[-10:]
    assert left_by(world, "pode1") == way, world.log[-10:]
    assert not world.problems


@ORDER
def test_a_robot_facing_south_does_not_leave_towards_a_robot_waiting_at_m_e(order):
    """남쪽을 본 로봇에게는 M_E 쪽이 싸지만 거기서 기다리는 로봇과 마주 본다. 급행으로 나간다."""
    world, _ = work_busy({"pode1": "W", "pode2": "M_E", "pode3": "P-1"}, ["pode2"], order,
                         yaw=NORTH)
    world.run(3.0)
    world.traffic.on_release("pode1")
    world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: len(targets(world, "pode1")) > 0, 30.0), world.log[-10:]
    assert left_by(world, "pode1") == "W_X", world.log[-10:]
    assert "M_E" not in targets(world, "pode1")
    assert not world.problems


@ORDER
def test_with_the_express_shut_it_waits_at_w_instead_of_facing_the_waiting_robot(order):
    """급행을 막았다고 표시하고(20 s) M_E에 줄 선 로봇이 있으면 나갈 길이 없다: W에서 기다린다.
    막힘이 풀리면 급행으로 나가고, M_E의 로봇이 북쪽에서 들어온다(교착 없음)."""
    world, (north,) = work_busy({"pode1": "W", "pode2": "M_E", "pode3": "P-1"}, ["pode2"], order,
                                yaw=NORTH)
    world.run(3.0)
    world.traffic._blocked[("W", "W_X")] = world.now + 20.0
    world.traffic.on_release("pode1")
    world.call("OUTBOUND", "pode1")
    world.run(10.0)
    assert targets(world, "pode1") == [] and world.view("pode1")["node"] == "W", world.log[-10:]
    assert world.run_until(lambda: world.result_of(north) is not None, 120.0), world.log[-20:]
    assert left_by(world, "pode1") == "W_X" and "M_E" not in targets(world, "pode1")
    assert math.degrees(world.bots["pode2"].yaw) == pytest.approx(-90.0)
    assert not world.problems


@ORDER
def test_a_robot_in_the_north_line_never_goes_round_through_the_south_line(order):
    """M_E -> W가 막혔다고 표시돼도 M_E의 로봇은 마당을 돌아 Q2, Q1로 들어가지 않는다(자기 줄로만)."""
    world, (north,) = work_busy({"pode1": "W", "pode2": "M_E", "pode3": "P-1"}, ["pode2"], order)
    world.run(3.0)
    world.traffic._blocked[("M_E", "W")] = world.now + 20.0
    world.traffic.on_release("pode1")
    world.call("OUTBOUND", "pode1")
    world.run(15.0)
    assert not {"Q1", "Q2", "O_IN", "E-3"} & set(targets(world, "pode2")), world.log[-20:]
    assert world.run_until(lambda: world.result_of(north) is not None, 120.0), world.log[-20:]
    assert targets(world, "pode2") == ["W"]
    assert not world.problems


def test_the_horizon_to_the_dock_keeps_the_north_line_while_driving_to_m_e():
    """order 모델: M_E로 가는 동안 M_E -> W가 막혔다고 표시돼도 horizon을 남쪽 줄로 바꾸지 않는다."""
    world, (_, north) = work_busy({"pode1": "W", "pode2": "Q1", "pode3": "O_OUT"},
                                  ["pode2", "pode3"], True)
    assert world.run_until(lambda: world.traffic._robots["pode3"].route is not None
                           and world.traffic._robots["pode3"].route.goal == "W", 30.0)
    world.traffic._blocked[("M_E", "W")] = world.now + 60.0
    world.run(40.0)
    changed = [e.detail for e in world.events
               if e.code == "HORIZON_CHANGED" and e.robot == "pode3"]
    assert not any("Q1" in d or "Q2" in d for d in changed), changed
    assert not {"Q1", "Q2"} & set(targets(world, "pode3")), world.log[-20:]
    assert not world.problems

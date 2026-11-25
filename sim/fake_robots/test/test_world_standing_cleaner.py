"""서 있는 청소 로봇은 몸 폭 캡슐로 본다: 34.75 cm 통로에서 옆을 지나간다(v7)."""
import math

import pytest
from fake_robots.safety_meter import SafetyMeter
from fake_robots.world.body import CLEANER_HALF_M, capsule_gap, spine
from fake_robots.world.invariants import TOUCH, Invariants
from fake_robots.world.robot import SimRobot
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from world_helpers import nav, place, quiet_world, results

R = load_params().robot_radius_m
AISLE_Y = 0.3475  # 남쪽 통로 북쪽 끝(진열대)
CLEAN_X = 1.40


def _pass(cleaner_y, yaw, cleaners=("clean",)):
    """E-3에서 Q2로 가는 nav_goal. (결과, 청소 로봇 옆을 지났나, 캡슐까지 최소 거리, 세계)."""
    world = quiet_world("v7", robots={"pode1": "P-1", "clean": "P-2"}, cleaners=cleaners)
    place(world, "pode1", 0.64, 0.24)
    place(world, "clean", CLEAN_X, cleaner_y, yaw)
    world.send(nav(world, "pode1", 1, xy=(1.66, 0.15)))
    body = spine(CLEAN_X, cleaner_y, yaw, R)
    beside, least = False, math.inf
    while not results(world, "on_move_result") and world.t < 60:
        world.step()
        p = world.robots["pode1"].pose
        beside |= abs(p[0] - CLEAN_X) <= 0.02 and p[1] < AISLE_Y
        least = min(least, capsule_gap(p, body))
    done = results(world, "on_move_result")
    return (done[0][2:] if done else None), beside, least, world


@pytest.mark.parametrize("cleaner_y", [0.2615, 0.086], ids=["by-shelf", "by-wall"])
def test_a_carrier_passes_a_cleaner_standing_along_the_aisle(cleaner_y):
    outcome, beside, least, world = _pass(cleaner_y, 0.0)
    assert outcome == (True, "")
    assert beside
    assert least >= R
    assert not [v for v in world.violations if v.code == TOUCH]
    assert world.guard_stops == 0


def test_a_cleaner_standing_across_the_aisle_closes_it():
    _, beside, least, world = _pass(0.174, math.pi / 2)
    assert not beside
    assert least >= R
    assert not [v for v in world.violations if v.code == TOUCH]


def test_a_standing_carrier_by_the_shelf_still_closes_the_aisle():
    _, beside, _, world = _pass(0.2615, 0.0, cleaners=())
    assert not beside
    assert not [v for v in world.violations if v.code == TOUCH]


def test_capsule_gap_is_half_width_sideways_and_radius_lengthwise():
    along = spine(1.0, 0.5, 0.0, R)
    assert capsule_gap((1.0, 0.5 + CLEANER_HALF_M + 0.01), along) == pytest.approx(0.01)
    assert capsule_gap((1.0 + R + 0.01, 0.5), along) == pytest.approx(0.01)
    assert capsule_gap((1.0 - R - 0.01, 0.5), along) == pytest.approx(0.01)
    across = spine(1.0, 0.5, math.pi / 2, R)
    assert capsule_gap((1.0, 0.5 + R + 0.01), across) == pytest.approx(0.01)
    assert capsule_gap((1.0 + CLEANER_HALF_M + 0.01, 0.5), across) == pytest.approx(0.01)
    assert capsule_gap((1.0, 0.5), along) == pytest.approx(-CLEANER_HALF_M)


def test_touch_uses_the_capsule_only_while_the_cleaner_stands():
    site, params = load_fleet_map("v7"), load_params()
    clean = SimRobot("clean", 1.40, 0.2615)
    carrier = SimRobot("pode1", 1.40, 0.11)  # 중심 0.1515 < 2r

    def codes(cleaners):
        inv = Invariants(site, params, cleaners)
        inv.check(0.1, None, {"clean": clean, "pode1": carrier}, {}, {})
        return [v.code for v in inv.violations if v.code == TOUCH]
    assert codes(("clean",)) == []
    carrier.y = 0.2615 - CLEANER_HALF_M - R + 0.005
    assert codes(("clean",)) == [TOUCH]
    carrier.y = 0.11
    assert codes(()) == [TOUCH]
    clean.run = object()  # 움직이는 청소 로봇은 원이다
    assert codes(("clean",)) == [TOUCH]


def test_safety_meter_counts_near_from_the_capsule_of_a_narrow_robot():
    site = load_fleet_map("v7")
    m = SafetyMeter(site, R, narrow=lambda robot: robot == "clean")
    m.on_pose("clean", 1.40, 0.2615, 0.0)
    m.on_pose("pode1", 1.40, 0.11)
    m.tick(0.0)
    assert m.near == 0
    m.on_pose("pode1", 1.40, 0.2615 - CLEANER_HALF_M - R + 0.005)
    m.tick(0.1)
    assert m.near == 1
    wide = SafetyMeter(site, R)
    wide.on_pose("clean", 1.40, 0.2615, 0.0)
    wide.on_pose("pode1", 1.40, 0.11)
    wide.tick(0.0)
    assert wide.near == 1

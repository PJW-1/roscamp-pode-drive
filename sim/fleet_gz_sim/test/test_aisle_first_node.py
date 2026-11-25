"""aisle_first 노드: 가짜 set_parameters와 clear 서비스를 같은 프로세스에 띄우고
모드가 바뀔 때 요청이 어떤 차례로 오는지 본다(ROS 2 rclpy와 nav2_msgs가 있을 때만).
위치는 TF 대신 고정값을 넣고, 이 프로세스만의 namespace 안에서 돈다.
"""
import os
import time
from dataclasses import replace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.srv import ClearEntireCostmap  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rcl_interfaces.msg import SetParametersResult  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

NAMESPACE = f"/aisle_first_test_{os.getpid()}"


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def spin_until(executor, done, timeout_s, also=lambda: None):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and not done():
        also()
        executor.spin_once(timeout_sec=0.05)
    return done()


def test_mode_changes_set_the_layer_then_clear_in_order(ros):
    from fleet_gz_sim.aisle_first_node import AisleFirst
    calls = []
    fake = rclpy.create_node("fake_costmap", namespace=NAMESPACE)

    def on_set(request, response):
        calls.append(("set", request.parameters[0].name, request.parameters[0].value.bool_value))
        response.results = [SetParametersResult(successful=True)]
        return response

    def on_clear(_request, response):
        calls.append(("clear",))
        return response

    plan = fake.create_publisher(Path, "plan", 10)
    node = AisleFirst(namespace=NAMESPACE, parameter_overrides=[
        Parameter("stall_s", value=0.6), Parameter("plan_timeout_s", value=0.5),
        Parameter("tick_s", value=0.05)])
    node.position = lambda: (1.0, 0.5)
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(node)
    try:
        # 서비스가 아직 없으면 기다렸다가 다음 박자에 낸다
        assert spin_until(executor, lambda: node.state.mode == "detour", 5.0,
                          also=lambda: plan.publish(Path()))
        assert calls == [] and node.ops == [("set", True)]
        fake.create_service(SetParameters, "global_costmap/global_costmap/set_parameters", on_set)
        fake.create_service(ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap",
                            on_clear)
        assert spin_until(executor, lambda: calls, 5.0, also=lambda: plan.publish(Path()))
        assert calls == [("set", "obstacle_layer.enabled", True)]
        # plan이 끊기면 주행 끝: 층을 끄고 나서 지운다
        assert spin_until(executor, lambda: len(calls) == 3 and not node.ops, 5.0)
        assert calls[1:] == [("set", "obstacle_layer.enabled", False), ("clear",)]
        assert node.state.mode == "aisle" and node.ops == []
    finally:
        executor.shutdown()
        node.destroy_node()
        fake.destroy_node()


def path_to(x, y):
    msg = Path()
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y = x, y
    msg.poses = [PoseStamped(), pose]
    return msg


def test_a_plan_to_another_goal_turns_the_layer_off_and_clears(ros):
    from fleet_gz_sim.aisle_first_node import AisleFirst
    calls = []
    fake = rclpy.create_node("fake_costmap", namespace=NAMESPACE)

    def on_set(request, response):
        calls.append(("set", request.parameters[0].value.bool_value))
        response.results = [SetParametersResult(successful=True)]
        return response

    def on_clear(_request, response):
        calls.append(("clear",))
        return response

    fake.create_service(SetParameters, "global_costmap/global_costmap/set_parameters", on_set)
    fake.create_service(ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap",
                        on_clear)
    plan = fake.create_publisher(Path, "plan", 10)
    node = AisleFirst(namespace=NAMESPACE, parameter_overrides=[
        Parameter("stall_s", value=1.0), Parameter("plan_timeout_s", value=5.0),
        Parameter("tick_s", value=0.05)])
    node.position = lambda: (1.0, 0.5)
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(node)
    try:
        assert spin_until(executor, lambda: calls == [("set", True)], 5.0,
                          also=lambda: plan.publish(path_to(1.6, 0.55)))
        assert node.state.mode == "detour"
        # 새 goal 뒤에 다시 멈춤으로 판정되어 요청 차례가 바뀌지 않게 멈춤 판정을 길게 둔다
        node.rules = replace(node.rules, stall_s=600.0)
        # 같은 goal의 재계획은 그대로 두고, 끝점이 다른 경로가 오면 층을 끄고 지운다
        assert spin_until(executor, lambda: len(calls) >= 3, 5.0,
                          also=lambda: plan.publish(path_to(0.49, 0.625)))
        assert calls[1:] == [("set", False), ("clear",)]
        assert node.state.mode == "aisle" and node.state.reason == "new goal"
    finally:
        executor.shutdown()
        node.destroy_node()
        fake.destroy_node()

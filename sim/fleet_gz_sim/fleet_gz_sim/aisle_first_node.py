"""aisle_first 노드(시뮬 시험): 통로 안에서 못 지나가면 전역 costmap의 장애물 층을 켠다.
  ros2 run fleet_gz_sim aisle_first --ros-args -r __ns:=/pode1 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=true
규칙은 aisle_first.py. 위치는 TF(map -> base_footprint), 주행 여부는 planner가 내는 plan으로 본다.
"""
from __future__ import annotations

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from fleet_gz_sim.aisle_first import (
    OBSTACLES_OFF,
    OBSTACLES_ON,
    Rules,
    State,
    goal_changed,
    log_line,
    navigating,
    step,
)

# ---- 이름 ----
PLAN = "plan"
SET_PARAMETERS = "global_costmap/global_costmap/set_parameters"
CLEAR = "global_costmap/clear_entirely_global_costmap"
LAYER_ENABLED = "obstacle_layer.enabled"


def _enabled(value: bool) -> ParameterMsg:
    return ParameterMsg(name=LAYER_ENABLED,
                        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))


class AisleFirst(Node):
    def __init__(self, **kwargs):
        super().__init__("aisle_first", **kwargs)
        declare = self.declare_parameter
        self.rules = Rules(plan_timeout_s=float(declare("plan_timeout_s", 2.5).value),
                           progress_m=float(declare("progress_m", 0.05).value),
                           stall_s=float(declare("stall_s", 5.0).value),
                           resume_m=float(declare("resume_m", 0.40).value),
                           goal_change_m=float(declare("goal_change_m", 0.20).value))
        self.global_frame = declare("global_frame", "map").value
        self.base_frame = declare("robot_base_frame", "base_footprint").value
        self.state = State()
        self.last_plan_t = None
        self.goal = None  # 마지막으로 받은 전역 경로의 끝점
        self.new_goal = False
        self.ops = []  # 차례로 낼 요청: ("set", bool) 또는 ("clear",)
        self.inflight = None  # (요청, future)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.set_client = self.create_client(SetParameters, SET_PARAMETERS)
        self.clear_client = self.create_client(ClearEntireCostmap, CLEAR)
        self.create_subscription(Path, PLAN, self._on_plan, 10)
        self.create_timer(float(declare("tick_s", 0.2).value), self.tick)
        self.get_logger().info(f"aisle_first: {self.rules}, {self.global_frame} -> "
                               f"{self.base_frame}")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_plan(self, msg):
        self.last_plan_t = self.now_s()
        if not msg.poses:
            return
        end = msg.poses[-1].pose.position
        if goal_changed(self.goal, (end.x, end.y), self.rules):
            self.new_goal = True
        self.goal = (end.x, end.y)

    def position(self):
        try:
            t = self.tf.lookup_transform(self.global_frame, self.base_frame, Time(),
                                         timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    # ---- 한 박자: 규칙을 돌리고 밀린 요청을 하나씩 낸다 ----
    def tick(self):
        now = self.now_s()
        xy = self.position()
        before = self.state.mode
        new_goal, self.new_goal = self.new_goal, False
        self.state, action = step(self.state, now, xy,
                                  navigating(self.last_plan_t, now, self.rules), self.rules,
                                  new_goal=new_goal)
        if action == OBSTACLES_ON:
            self.ops = [("set", True)]
        elif action == OBSTACLES_OFF:
            self.ops = [("set", False), ("clear",)]
        if self.state.mode != before:
            self.get_logger().info(log_line(self.state, xy))
        self._send()

    def _send(self):
        if self.inflight is not None:
            op, future = self.inflight
            if not future.done():
                return
            self.inflight = None
            if self._succeeded(op, future) and self.ops and self.ops[0] == op:
                self.ops.pop(0)
        if not self.ops:
            return
        op = self.ops[0]
        client = self.clear_client if op[0] == "clear" else self.set_client
        if not client.service_is_ready():
            return  # 다음 박자에 다시
        if op[0] == "clear":
            future = client.call_async(ClearEntireCostmap.Request())
        else:
            future = client.call_async(SetParameters.Request(parameters=[_enabled(op[1])]))
        self.inflight = (op, future)

    def _succeeded(self, op, future) -> bool:
        if future.exception() is not None or future.result() is None:
            self.get_logger().warn(f"aisle_first: {op} failed, retrying")
            return False
        if op[0] == "set" and not all(r.successful for r in future.result().results):
            reasons = [r.reason for r in future.result().results]
            self.get_logger().warn(f"aisle_first: {LAYER_ENABLED}={op[1]} refused: {reasons}")
            return False
        return True


def main():
    rclpy.init()
    node = AisleFirst()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""다른 Pode의 몸을 이 로봇의 costmap peer 층에 점구름으로 내는 노드(임시, 주행 시험용).

- 입력: 다른 로봇마다 /<로봇>/state(map 좌표 자세). 위치를 모르는 보고는 쓰지 않고, 조용해진 로봇의 자리는 그대로 둔다.
- 출력: peer_obstacles(PointCloud2, map 틀), peer_sweep(모든 빔 inf인 LaserScan, base_footprint 틀).
"""
import math

import rclpy
from fleet_interfaces.msg import RobotState
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .body import (
    CENTRE_M,
    DISC_SHAPE,
    FOOTPRINT,
    FOOTPRINT_SHAPE,
    MARGIN_M,
    RADIUS_M,
    SHAPES,
    body_points,
    footprint_points,
    lane_points,
    others,
)

STATE_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)  # 계약의 STATE
POINT_Z_M = 0.05  # 점 높이 (m)
FAR_POINT = (1000.0, 1000.0, POINT_Z_M)  # 점구름이 비지 않게 늘 함께 내는 먼 점
# sweep 시각을 이만큼 과거로 적는다: 지금 시각의 TF가 아직 없을 수 있다
SWEEP_DELAY = Duration(seconds=0.2)


class PeerObstacles(Node):
    def __init__(self, **kwargs):
        super().__init__("peer_obstacles", **kwargs)
        declare = self.declare_parameter
        self.robot_id = declare("robot_id", "").value or self.get_namespace().strip("/")
        self.radius_m = float(declare("radius_m", RADIUS_M).value)
        # 선(IDLE) 로봇을 찍는 모양: disc는 달릴 때와 같은 원, footprint는 몸 발자국
        self.standing_shape = declare("standing_shape", DISC_SHAPE).value
        if self.standing_shape not in SHAPES:
            raise ValueError(f"standing_shape must be one of {SHAPES}, got {self.standing_shape!r}")
        # 선 채로 띠(peer_lane)를 늘이는 로봇 이름들과 띠 길이 (m). 0이면 내지 않는다
        self.lane_robots = others(declare("lane_robots", "").value, self.robot_id)
        self.lane_m = float(declare("lane_m", 0.0).value)
        self.centre_m = float(declare("centre_m", CENTRE_M).value)
        self.step_m = float(declare("step_m", 0.02).value)
        self.map_frame = declare("map_frame", "map").value
        self.base_frame = declare("base_frame", "base_footprint").value
        beams = int(declare("sweep_beams", 1440).value)
        sweep_range_m = float(declare("sweep_range_m", 4.0).value)
        peers = others(declare("robots", "").value, self.robot_id)
        self.poses = {}  # 로봇 이름 -> 마지막으로 보고한 (x, y, yaw)
        self.standing = set()  # 선 채로 보고한 로봇 이름
        self.me = None  # 이 로봇이 마지막으로 보고한 (x, y, yaw)
        if self.lane_robots and self.lane_m > 0.0:
            self.create_subscription(RobotState, f"/{self.robot_id}/state", self.on_me, STATE_QOS)
        for name in peers:
            self.create_subscription(RobotState, f"/{name}/state",
                                     lambda msg, name=name: self.on_state(name, msg), STATE_QOS)
        self.cloud_pub = self.create_publisher(PointCloud2, "peer_obstacles", 5)
        self.lane_pub = self.create_publisher(PointCloud2, "peer_lane", 5)
        self.sweep_pub = self.create_publisher(LaserScan, "peer_sweep", 5)
        self.sweep = LaserScan()
        self.sweep.header.frame_id = self.base_frame
        self.sweep.angle_min = -math.pi
        self.sweep.angle_increment = 2.0 * math.pi / beams
        self.sweep.angle_max = math.pi - self.sweep.angle_increment
        self.sweep.range_min = 0.0
        self.sweep.range_max = sweep_range_m
        self.sweep.ranges = [math.inf] * beams
        self.create_timer(1.0 / float(declare("rate_hz", 5.0).value), self.publish)
        self.get_logger().info(f"{self.robot_id}: bodies of {peers}, radius {self.radius_m} m, "
                               f"standing as {self.standing_shape}, lane {self.lane_m} m "
                               f"of {sorted(self.lane_robots)}")

    def on_state(self, name: str, msg: RobotState):
        pose = (msg.pose.x, msg.pose.y, msg.pose.theta)
        known = msg.state != RobotState.STATE_LOCALIZING and pose != (0.0, 0.0, 0.0)
        if known and all(math.isfinite(value) for value in pose):
            self.poses[name] = pose
            if msg.state == RobotState.STATE_IDLE:
                self.standing.add(name)
            else:
                self.standing.discard(name)

    def on_me(self, msg: RobotState):
        pose = (msg.pose.x, msg.pose.y, msg.pose.theta)
        known = msg.state != RobotState.STATE_LOCALIZING and pose != (0.0, 0.0, 0.0)
        if known and all(math.isfinite(value) for value in pose):
            self.me = pose

    def body(self, name: str) -> list:
        """name의 몸 자리 점들: 선 로봇은 standing_shape, 달리는 로봇은 원."""
        pose = self.poses[name]
        if name in self.standing and self.standing_shape == FOOTPRINT_SHAPE:
            return footprint_points(*pose, self.step_m, centre_m=self.centre_m)
        return body_points(*pose, self.radius_m, self.step_m, self.centre_m)

    def lane(self) -> list:
        """선 lane_robots의 몸에서 이 로봇 쪽으로 늘인 띠의 점들."""
        if self.me is None:
            return []
        half = (max(v[1] for v in FOOTPRINT) + MARGIN_M
                if self.standing_shape == FOOTPRINT_SHAPE else self.radius_m)
        return [point for name in self.lane_robots if name in self.standing and name in self.poses
                for point in lane_points(self.poses[name], self.me, self.lane_m, half, self.step_m)]

    def publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        points = [(x, y, POINT_Z_M) for name in self.poses for x, y in self.body(name)]
        points.append(FAR_POINT)
        header = Header(stamp=stamp, frame_id=self.map_frame)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        lane = [(x, y, POINT_Z_M) for x, y in self.lane()]
        lane.append(FAR_POINT)
        self.lane_pub.publish(point_cloud2.create_cloud_xyz32(header, lane))
        if now.nanoseconds > SWEEP_DELAY.nanoseconds:  # 시뮬레이션 시계가 막 0에서 시작했을 때
            self.sweep.header.stamp = (now - SWEEP_DELAY).to_msg()
        else:
            self.sweep.header.stamp = stamp
        self.sweep_pub.publish(self.sweep)


def make_fleet(robots: str, use_sim_time: bool = False, context=None,
               standing_shape: str = DISC_SHAPE, lane_robots: str = "", lane_m: float = 0.0) -> list:
    """robots의 로봇마다 그 namespace 안에 PeerObstacles 노드 하나를 만든다."""
    return [PeerObstacles(namespace=f"/{name}", context=context, use_global_arguments=False,
                          parameter_overrides=[Parameter("robot_id", value=name),
                                               Parameter("robots", value=robots),
                                               Parameter("standing_shape", value=standing_shape),
                                               Parameter("lane_robots", value=lane_robots),
                                               Parameter("lane_m", value=float(lane_m)),
                                               Parameter("use_sim_time", value=use_sim_time)])
            for name in others(robots, "")]


def main():
    rclpy.init()
    node = PeerObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

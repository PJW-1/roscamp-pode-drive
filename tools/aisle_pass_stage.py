"""통로 교차 시연 준비: 청소 로봇을 남쪽 통로 진열대 쪽에 붙여 세운다(Gazebo).

  python3 tools/aisle_pass_stage.py [로봇] [x,y,yaw]     기본: pode3 1.40,0.278,0

gz.launch.py(cleaners:=<로봇>)를 띄운 터미널과 같은 환경(ROS_DOMAIN_ID, GZ_PARTITION)에서 띄운 직후에 돌린다.
관제가 뜨기 전에 옮겨야 청소 로봇이 집 밖에 선 것으로 보고 그 자리에 서 있는다(CLEAN_NOT_HOME).
그 뒤 운반 로봇이 남쪽 통로로 Q2에 갈 때 선 청소 로봇 옆을 지난다.
Gazebo 자리 옮기기(set_pose)와 AMCL 처음 자세(/<로봇>/initialpose)를 함께 쓴다.
"""
import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

DEFAULT_ROBOT, DEFAULT_POSE = "pode3", (1.40, 0.278, 0.0)
TRUTH_WAIT_S = 180.0  # Gazebo가 뜨기를 기다리는 시간
SETTLE_M = 0.03  # AMCL 자세가 놓은 자리에서 이 안이면 자리를 잡은 것으로 본다
SETTLE_S = 60.0  # 처음 자세를 되풀이해 보내는 시간


def sh(cmd: str, timeout: float = 10.0) -> str:
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def pose_request(name: str, x: float, y: float, z: float, yaw: float) -> str:
    """gz set_pose 서비스에 넣는 요청 글."""
    return (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')


def initial_pose(x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.0004
    msg.pose.covariance[35] = 0.001
    return msg


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    name = args[0] if args else DEFAULT_ROBOT
    x, y, yaw = (tuple(float(v) for v in args[1].split(",")) if len(args) > 1 else DEFAULT_POSE)
    rclpy.init()
    node = rclpy.create_node("aisle_pass_stage")
    truth, amcl = [], []
    node.create_subscription(Odometry, f"/{name}/sim/truth",
                             lambda m: truth.append(m.pose.pose.position), qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, f"/{name}/amcl_pose",
                             lambda m: amcl.append(m.pose.pose.position), 5)
    pub = node.create_publisher(PoseWithCovarianceStamped, f"/{name}/initialpose", 1)

    def spin(seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    start = time.time()
    while not truth and time.time() - start < TRUTH_WAIT_S:
        spin(0.5)
    if not truth:
        print(f"no /{name}/sim/truth: is Gazebo up in this environment?")
        return 1
    service = re.search(r"/world/[^/\s]+/set_pose\b", sh("gz service -l"))
    if service is None:
        print("no set_pose service: check GZ_PARTITION")
        return 1
    out = sh(f"gz service -s {service.group(0)} --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
             f"--timeout 3000 --req '{pose_request(name, x, y, truth[-1].z, yaw)}'")
    print(f"set_pose ({x}, {y}, {yaw}) -> {out.strip()}")
    msg, end, good = initial_pose(x, y, yaw), time.time() + SETTLE_S, 0
    while time.time() < end and good < 6:
        pub.publish(msg)
        spin(0.5)
        near = bool(amcl) and math.hypot(amcl[-1].x - x, amcl[-1].y - y) <= SETTLE_M
        good = good + 1 if near else 0
    print("AMCL stands on the spot" if good >= 6 else "AMCL did not settle on the spot")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if good >= 6 else 1


if __name__ == "__main__":
    sys.exit(main())

// 플러그인 시험: 같은 값의 RPP를 따로 띄워 같은 입력에서 cmd_vel을 견준다
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "pode_follow_wall/follow_wall.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"

namespace
{

// 보호 멤버를 시험에서 본다
class FollowWallProbe : public pode_follow_wall::FollowWall
{
public:
  void feedScan(const sensor_msgs::msg::LaserScan & s)
  {
    scanCallback(std::make_shared<sensor_msgs::msg::LaserScan>(s));
  }
  void feedMap(const nav_msgs::msg::OccupancyGrid & g)
  {
    mapCallback(std::make_shared<nav_msgs::msg::OccupancyGrid>(g));
  }
  double filtered() const {return filtered_error_;}
  bool shifted() const {return shifted_;}
};

// RPP가 허용치를 묻는 goal checker(두 controller에 같은 것)
class StubGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & vel) override
  {
    pose.position.x = 0.05;
    pose.position.y = 0.05;
    vel = geometry_msgs::msg::Twist();
    return true;
  }
};

constexpr double kLaserX = -0.02;  // 몸 중심 뒤 라이다
constexpr double kPathY = 2.0;

// 두 RPP(따로 띄운 것, FollowWall 안의 것)에 같은 값을 준다
std::vector<rclcpp::Parameter> rppParams(const std::string & ns)
{
  return {
    {ns + ".desired_linear_vel", 0.12},
    {ns + ".lookahead_dist", 0.10},
    {ns + ".min_lookahead_dist", 0.10},
    {ns + ".max_lookahead_dist", 0.25},
    {ns + ".lookahead_time", 0.8},
    {ns + ".use_velocity_scaled_lookahead_dist", true},
    {ns + ".use_rotate_to_heading", true},
    {ns + ".rotate_to_heading_min_angle", 0.70},
    {ns + ".allow_reversing", false},
    {ns + ".use_collision_detection", true},
    {ns + ".max_robot_pose_search_dist", 1.4875},
    {ns + ".approach_velocity_scaling_dist", 0.10},
    {ns + ".min_approach_linear_velocity", 0.08},
    {ns + ".transform_tolerance", 0.2},
  };
}

geometry_msgs::msg::Quaternion yawQ(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

struct Pose
{
  double x, y, yaw;
};

}  // namespace

class FollowWallTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::vector<rclcpp::Parameter> params = rppParams("RPP");
    const auto fw = rppParams("FollowWall.rpp");
    params.insert(params.end(), fw.begin(), fw.end());
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("fw_test", opts);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    costmap_->on_configure(rclcpp_lifecycle::State());

    // 라이다는 180도 돌아 있다(rplidar_link와 같게)
    geometry_msgs::msg::TransformStamped laser;
    laser.header.frame_id = "base_link";
    laser.child_frame_id = "laser";
    laser.transform.translation.x = kLaserX;
    laser.transform.rotation = yawQ(M_PI);
    tf_->setTransform(laser, "test", true);

    rpp_ = std::make_shared<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>();
    rpp_->configure(node_, "RPP", tf_, costmap_);
    rpp_->activate();
    fw_ = std::make_shared<FollowWallProbe>();
    fw_->configure(node_, "FollowWall", tf_, costmap_);
    fw_->activate();

    // 동쪽으로 가는 줄
    path_.header.frame_id = "map";
    for (int i = 0; i <= 40; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = 1.0 + 0.05 * i;
      p.pose.position.y = kPathY;
      p.pose.orientation.w = 1.0;
      path_.poses.push_back(p);
    }
    rpp_->setPlan(path_);
    fw_->setPlan(path_);
  }

  void TearDown() override
  {
    fw_->deactivate();
    fw_->cleanup();
    rpp_->deactivate();
    rpp_->cleanup();
  }

  // 지도: 칸 0.01 m, 왼쪽 벽 면 wall_y(없으면 빈 지도)
  nav_msgs::msg::OccupancyGrid makeMap(double wall_y) const
  {
    nav_msgs::msg::OccupancyGrid g;
    g.header.frame_id = "map";
    g.info.resolution = 0.01;
    g.info.width = 500;
    g.info.height = 400;
    g.info.origin.orientation.w = 1.0;
    g.data.assign(500 * 400, 0);
    if (std::isfinite(wall_y)) {
      const int row = static_cast<int>(std::lround(wall_y / 0.01));
      for (int r = row; r < row + 3; ++r) {
        for (int c = 0; c < 500; ++c) {
          g.data[r * 500 + c] = 100;
        }
      }
    }
    return g;
  }

  // 참 자세에서 본 scan(laser 틀): 가로 벽 y = wall_y
  sensor_msgs::msg::LaserScan makeScan(const Pose & truth, double wall_y) const
  {
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id = "laser";
    s.header.stamp = node_->now();
    s.angle_min = -M_PI;
    s.angle_increment = 2.0 * M_PI / 500.0;
    s.angle_max = s.angle_min + 499 * s.angle_increment;
    s.range_min = 0.05;
    s.range_max = 12.0;
    const double ly = truth.y + std::sin(truth.yaw) * kLaserX;
    for (int i = 0; i < 500; ++i) {
      const double th = truth.yaw + M_PI + s.angle_min + i * s.angle_increment;
      double r = std::numeric_limits<double>::infinity();
      const double sn = std::sin(th);
      if (std::isfinite(wall_y) && std::abs(sn) > 1e-9) {
        const double t = (wall_y - ly) / sn;
        if (t > 0.0) {
          r = t;
        }
      }
      s.ranges.push_back(static_cast<float>(r));
    }
    return s;
  }

  // 추정 자세를 TF와 pose로 넣는다
  geometry_msgs::msg::PoseStamped setPose(const Pose & est)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = est.x;
    t.transform.translation.y = est.y;
    t.transform.rotation = yawQ(est.yaw);
    tf_->setTransform(t, "test", true);
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = est.x;
    p.pose.position.y = est.y;
    p.pose.orientation = yawQ(est.yaw);
    return p;
  }

  struct Cmds
  {
    geometry_msgs::msg::Twist fw;
    geometry_msgs::msg::Twist rpp;
  };

  // 한 주기: scan(있으면)을 넣고 두 controller를 부른다
  Cmds step(const Pose & est, const Pose & truth, double scan_wall_y, bool with_scan = true)
  {
    const auto pose = setPose(est);
    if (with_scan) {
      fw_->feedScan(makeScan(truth, scan_wall_y));
    }
    geometry_msgs::msg::Twist vel;
    vel.linear.x = 0.12;
    Cmds c;
    c.fw = fw_->computeVelocityCommands(pose, vel, &checker_).twist;
    c.rpp = rpp_->computeVelocityCommands(pose, vel, &checker_).twist;
    return c;
  }

  // 줄을 따라 가며 매 주기 두 출력이 같은지 본다
  void expectSameAsRpp(double map_wall_y, double scan_wall_y, double lateral, bool with_scan)
  {
    fw_->feedMap(makeMap(map_wall_y));
    int cycles = 0, moving = 0, turning = 0;
    for (double x = 1.05; x < 2.6; x += 0.006) {
      const double yaw = 0.08 * std::sin(3.0 * x);
      const Pose p{x, kPathY + lateral, yaw};
      const auto c = step(p, p, scan_wall_y, with_scan);
      ASSERT_DOUBLE_EQ(c.fw.linear.x, c.rpp.linear.x) << "x=" << x;
      ASSERT_DOUBLE_EQ(c.fw.angular.z, c.rpp.angular.z) << "x=" << x;
      EXPECT_FALSE(fw_->shifted());
      ++cycles;
      // 견줄 값이 0만이 아닌지(실제로 움직이고 돈다)
      moving += c.rpp.linear.x > 0.05 ? 1 : 0;
      turning += std::abs(c.rpp.angular.z) > 0.01 ? 1 : 0;
    }
    EXPECT_GT(cycles, 200);
    EXPECT_GT(moving, cycles / 2);
    EXPECT_GT(turning, cycles / 4);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController> rpp_;
  std::shared_ptr<FollowWallProbe> fw_;
  nav_msgs::msg::Path path_;
  StubGoalChecker checker_;
};

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();

// ---- 벽이 없거나 보이지 않는 줄: RPP와 같은 출력 ----

TEST_F(FollowWallTest, NoScanNoMapSameAsRpp)
{
  // scan과 지도가 아직 없다
  int cycles = 0;
  for (double x = 1.05; x < 2.6; x += 0.006) {
    const Pose p{x, kPathY + 0.01, 0.05};
    const auto c = step(p, p, kNone, false);
    ASSERT_DOUBLE_EQ(c.fw.linear.x, c.rpp.linear.x);
    ASSERT_DOUBLE_EQ(c.fw.angular.z, c.rpp.angular.z);
    ++cycles;
  }
  EXPECT_GT(cycles, 200);
}

TEST_F(FollowWallTest, NoWallSameAsRpp)
{
  expectSameAsRpp(kNone, kNone, 0.012, true);
}

TEST_F(FollowWallTest, WallNotInMapSameAsRpp)
{
  // 라이다에만 벽(사람, 상자): 지도와 맞지 않아 보정하지 않는다
  expectSameAsRpp(kNone, kPathY + 0.08, 0.0, true);
}

TEST_F(FollowWallTest, WallOutsideWindowSameAsRpp)
{
  // 벽이 0.5 m 떨어져 창(0.3 m) 밖
  expectSameAsRpp(kPathY + 0.5, kPathY + 0.5, 0.0, true);
}

TEST_F(FollowWallTest, StaleScanSameAsRpp)
{
  // 위치 추정 오차가 있어도 scan이 오래되면 보정하지 않는다
  fw_->feedMap(makeMap(kPathY + 0.08));
  auto old = makeScan({1.05, kPathY - 0.02, 0.0}, kPathY + 0.08);
  old.header.stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  fw_->feedScan(old);
  for (double x = 1.05; x < 2.0; x += 0.006) {
    const Pose p{x, kPathY, 0.0};
    const auto c = step(p, p, kNone, false);
    ASSERT_DOUBLE_EQ(c.fw.angular.z, c.rpp.angular.z);
  }
}

TEST_F(FollowWallTest, FollowErrorOnlySameAsRpp)
{
  // 추정 자세가 맞고 로봇만 경로에서 1.5 cm 벗어났다: e = 0이라 RPP 그대로
  expectSameAsRpp(kPathY + 0.08, kPathY + 0.08, 0.015, true);
}

// ---- 위치 추정 오차: 벽 쪽으로 옮긴다 ----

TEST_F(FollowWallTest, LocalizationErrorShiftsTowardWall)
{
  // 지도 벽 면은 경로 왼쪽 0.08 m, 실제로는 추정보다 3 cm 멀다(e = +0.03, 상한 0.02)
  fw_->feedMap(makeMap(kPathY + 0.08));
  std::vector<double> shifts;
  auto sub = node_->create_subscription<pode_follow_wall::msg::FollowWallState>(
    "FollowWall/state", 100,
    [&](pode_follow_wall::msg::FollowWallState::ConstSharedPtr m) {
      if (m->wall_seen) {
        EXPECT_NEAR(m->error, 0.03, 1e-3);
        EXPECT_EQ(m->side, 1);
      }
      shifts.push_back(m->applied_shift);
    });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_->get_node_base_interface());
  for (int i = 0; i < 40 && node_->count_publishers("FollowWall/state") == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  Cmds last;
  for (int k = 0; k < 60; ++k) {
    const double x = 1.05 + 0.006 * k;
    const Pose est{x, kPathY, 0.0};
    const Pose truth{x, kPathY - 0.03, 0.0};
    last = step(est, truth, kPathY + 0.08);
    exec.spin_some();
  }
  // 거른 값은 e를 따라가고, 보정은 상한에서 멈춘다
  EXPECT_NEAR(fw_->filtered(), 0.03, 0.002);
  EXPECT_TRUE(fw_->shifted());
  // 벽(왼쪽)으로 옮긴 경로라 RPP보다 왼쪽으로 돈다
  EXPECT_GT(last.fw.angular.z, last.rpp.angular.z + 1e-3);
  EXPECT_GT(last.fw.angular.z, 0.0);

  for (int i = 0; i < 20; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_FALSE(shifts.empty());
  for (double s : shifts) {
    EXPECT_GE(s, 0.0);
    EXPECT_LE(s, 0.02 + 1e-12);
  }
  EXPECT_DOUBLE_EQ(shifts.back(), 0.02);
}

TEST_F(FollowWallTest, NegativeErrorShiftsAwayFromWall)
{
  // 실제로는 1.5 cm 더 가깝다: 벽에서 먼 쪽(오른쪽)으로
  fw_->feedMap(makeMap(kPathY + 0.08));
  Cmds last;
  for (int k = 0; k < 60; ++k) {
    const double x = 1.05 + 0.006 * k;
    last = step({x, kPathY, 0.0}, {x, kPathY + 0.015, 0.0}, kPathY + 0.08);
  }
  EXPECT_NEAR(fw_->filtered(), -0.015, 0.002);
  EXPECT_LT(last.fw.angular.z, last.rpp.angular.z - 1e-3);
}

TEST_F(FollowWallTest, WallLostReturnsToRpp)
{
  // 보정하다가 벽이 사라지면 원래 경로로 돌아가 RPP와 같아진다
  fw_->feedMap(makeMap(kPathY + 0.08));
  double x = 1.05;
  for (int k = 0; k < 40; ++k, x += 0.006) {
    step({x, kPathY, 0.0}, {x, kPathY - 0.02, 0.0}, kPathY + 0.08);
  }
  ASSERT_TRUE(fw_->shifted());
  for (int k = 0; k < 100; ++k, x += 0.006) {
    const Pose p{x, kPathY + 0.005, 0.02};
    const auto c = step(p, p, kNone);
    ASSERT_DOUBLE_EQ(c.fw.linear.x, c.rpp.linear.x) << k;
    ASSERT_DOUBLE_EQ(c.fw.angular.z, c.rpp.angular.z) << k;
    EXPECT_FALSE(fw_->shifted());
    EXPECT_DOUBLE_EQ(fw_->filtered(), 0.0);
  }
}

TEST_F(FollowWallTest, PassThroughSpeedLimit)
{
  fw_->feedMap(makeMap(kNone));
  fw_->setSpeedLimit(0.06, false);
  rpp_->setSpeedLimit(0.06, false);
  for (double x = 1.05; x < 1.5; x += 0.006) {
    const Pose p{x, kPathY, 0.0};
    const auto c = step(p, p, kNone);
    ASSERT_DOUBLE_EQ(c.fw.linear.x, c.rpp.linear.x);
    EXPECT_LE(c.fw.linear.x, 0.06 + 1e-12);
  }
}

// controller_server처럼 이름으로 플러그인을 띄울 수 있다
TEST(Plugin, LoadsByName)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  EXPECT_NO_THROW({
    auto c = loader.createSharedInstance("pode_follow_wall::FollowWall");
    EXPECT_NE(c, nullptr);
  });
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "pode_rl_controller/observation.hpp"
#include "pode_rl_controller/policy_spec.hpp"

using pode_rl_controller::Point2;
using pode_rl_controller::PolicySpec;

namespace
{
constexpr double kObsTol = 1e-6;
constexpr double kGeomTol = 1e-9;

PolicySpec spec(const std::string & dir = POLICY_DIR)
{
  return pode_rl_controller::load_policy_spec(dir + "/policy_spec.json");
}

YAML::Node follow_fixture()
{
  return YAML::LoadFile(std::string(FIXTURE_DIR) + "/follow_cases.json");
}

std::vector<Point2> path_of(const YAML::Node & c)
{
  std::vector<Point2> path;
  for (const auto & p : c["path"]) {
    path.push_back({p[0].as<double>(), p[1].as<double>()});
  }
  return path;
}

double gap(double got, double want)
{
  const double d = std::abs(got - want);
  return std::isfinite(d) ? d : std::numeric_limits<double>::infinity();
}

double angle_gap(double got, double want)
{
  return gap(std::remainder(got - want, 2.0 * M_PI), 0.0);
}

std::vector<Point2> line(int n, double step)
{
  std::vector<Point2> path;
  for (int i = 0; i < n; ++i) {
    path.push_back({i * step, 0.0});
  }
  return path;
}
}  // namespace

// ---- 명세의 목표점 규칙 ----
TEST(PolicySpec, GoalRuleComesFromTheSpec)
{
  const PolicySpec s = spec();
  EXPECT_EQ(s.goal_rule.rule, "lookahead");
  EXPECT_DOUBLE_EQ(s.goal_rule.lookahead_m, 0.40);
  EXPECT_EQ(s.goal_rule.follow_window, 20);
  const auto f = follow_fixture();
  EXPECT_DOUBLE_EQ(f["lookahead_m"].as<double>(), s.goal_rule.lookahead_m);
  EXPECT_EQ(f["follow_window"].as<int>(), s.goal_rule.follow_window);
  EXPECT_EQ(spec(PATH_END_POLICY_DIR).goal_rule.rule, "path_end");
}

// ---- 경로 위 앞보기 점: route_spec.follow와 같은가 ----
TEST(Follow, MatchesRouteSpec)
{
  const PolicySpec s = spec();
  double worst_point = 0.0;
  int steps = 0;
  for (const auto & c : follow_fixture()["cases"]) {
    const auto path = path_of(c);
    size_t prog = 0;
    for (const auto & st : c["steps"]) {
      const Point2 robot{st["xy"][0].as<double>(), st["xy"][1].as<double>()};
      prog = pode_rl_controller::follow_progress(path, prog, robot, s.goal_rule.follow_window);
      ASSERT_EQ(prog, st["prog"].as<size_t>()) << c["name"].as<std::string>() << " step " << steps;
      const Point2 la =
        path[pode_rl_controller::lookahead_index(path, prog, s.goal_rule.lookahead_m)];
      worst_point = std::max(
        {worst_point, gap(la.x, st["lookahead_xy"][0].as<double>()),
          gap(la.y, st["lookahead_xy"][1].as<double>())});
      ++steps;
    }
  }
  std::printf("[follow] %d steps, lookahead point max |diff| %.3g m\n", steps, worst_point);
  EXPECT_GT(steps, 40);
  EXPECT_LE(worst_point, kGeomTol);
}

TEST(Follow, PathGoalMatchesTaskSpecInThePlanFrame)
{
  const PolicySpec s = spec();
  double worst = 0.0;
  int steps = 0;
  for (const auto & c : follow_fixture()["cases"]) {
    const auto path = path_of(c);
    size_t prog = 0;
    for (const auto & st : c["steps"]) {
      const Point2 robot{st["xy"][0].as<double>(), st["xy"][1].as<double>()};
      const auto g =
        pode_rl_controller::path_goal(path, prog, robot, st["yaw"].as<double>(), s.goal_rule);
      ASSERT_EQ(prog, st["prog"].as<size_t>());
      worst = std::max(
        {worst, gap(g.dist, st["goal_dist"].as<double>()),
          angle_gap(g.angle, st["goal_angle"].as<double>())});
      ++steps;
    }
  }
  std::printf("[follow] path_goal over %d steps, goal max |diff| %.3g\n", steps, worst);
  EXPECT_GT(steps, 40);
  EXPECT_LE(worst, kGeomTol);
}

TEST(Follow, PathEndRuleTakesTheLastPoint)
{
  const auto path = line(81, 0.025);
  size_t prog = 0;
  const auto g = pode_rl_controller::path_goal(
    path, prog, {0.0, 1.0}, 0.0, pode_rl_controller::GoalRule{"path_end", 0.0, 0});
  EXPECT_NEAR(g.dist, std::hypot(2.0, 1.0), kGeomTol);
  EXPECT_NEAR(g.angle, std::atan2(-1.0, 2.0), kGeomTol);
  EXPECT_EQ(prog, 0u);
}

TEST(Follow, PathShorterThanLookaheadGivesItsEnd)
{
  const auto path = line(9, 0.025);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 0, 0.40), 8u);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 4, 0.40), 8u);
}

TEST(Follow, EndOfPathGivesItsEnd)
{
  const auto path = line(81, 0.025);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 0, 0.40), 16u);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 70, 0.40), 80u);
  EXPECT_EQ(pode_rl_controller::lookahead_index(path, 80, 0.40), 80u);
  EXPECT_EQ(pode_rl_controller::follow_progress(path, 80, {5.0, 0.0}, 20), 80u);
}

TEST(Follow, ProgressMovesAtMostTheWindowAndNeverBack)
{
  const auto path = line(81, 0.025);
  EXPECT_EQ(pode_rl_controller::follow_progress(path, 10, {1.5, 0.0}, 20), 30u);
  EXPECT_EQ(pode_rl_controller::follow_progress(path, 30, {0.0, 0.0}, 20), 30u);
}

// ---- 목표와 속도 관측: task_spec과 같은가 ----
TEST(GoalAndVelocity, MatchTaskSpec)
{
  const PolicySpec s = spec();
  const auto rays = std::vector<float>(s.lidar.rays, static_cast<float>(s.lidar.obs_max));
  double worst_polar = 0.0;
  double worst_obs = 0.0;
  int steps = 0;
  for (const auto & c : follow_fixture()["cases"]) {
    for (const auto & st : c["steps"]) {
      ++steps;
      const Point2 robot{st["xy"][0].as<double>(), st["xy"][1].as<double>()};
      const Point2 la{st["lookahead_xy"][0].as<double>(), st["lookahead_xy"][1].as<double>()};
      const auto g = pode_rl_controller::goal_in_robot_frame(robot, st["yaw"].as<double>(), la);
      worst_polar = std::max(
        {worst_polar, gap(g.dist, st["goal_dist"].as<double>()),
          angle_gap(g.angle, st["goal_angle"].as<double>())});
      const auto obs = pode_rl_controller::build_observation(
        rays, g.dist, g.angle, st["v"].as<double>(), st["w"].as<double>(), s);
      const auto want = st["obs_goal_vel"].as<std::vector<double>>();
      ASSERT_EQ(want.size(), 5u);
      for (size_t k = 0; k < want.size(); ++k) {
        worst_obs = std::max(worst_obs, gap(obs[s.lidar.sectors + k], want[k]));
      }
    }
  }
  std::printf("[goal] polar max |diff| %.3g, goal and velocity obs max |diff| %.3g\n",
    worst_polar, worst_obs);
  EXPECT_GT(steps, 40);
  EXPECT_LE(worst_polar, kGeomTol);
  EXPECT_LE(worst_obs, kObsTol);
}

// ---- 라이다가 yaw pi로 달린 Gazebo 배치 ----
TEST(Lidar, YawPiMountPutsTheLidarBackAtRayZero)
{
  const PolicySpec s = spec();
  const int n = 640;
  pode_rl_controller::Scan scan{-M_PI, 2.0 * M_PI / (n - 1), 0.05, 12.0,
    std::vector<float>(n, std::numeric_limits<float>::infinity())};
  scan.ranges[0] = 0.5f;
  scan.ranges[160] = 0.8f;
  scan.ranges[320] = 1.2f;
  const auto rays = pode_rl_controller::scan_to_rays(scan, M_PI, s);
  EXPECT_FLOAT_EQ(rays[180], 0.5f);
  EXPECT_FLOAT_EQ(rays[270], 0.8f);
  EXPECT_FLOAT_EQ(rays[0], 1.2f);
  EXPECT_FLOAT_EQ(rays[90], 2.0f);
  const auto obs = pode_rl_controller::build_observation(rays, 1.0, 0.0, 0.0, 0.0, s);
  EXPECT_FLOAT_EQ(obs[18], 0.25f);
  EXPECT_FLOAT_EQ(obs[27], 0.4f);
  EXPECT_FLOAT_EQ(obs[0], 0.6f);
  EXPECT_FLOAT_EQ(obs[9], 1.0f);
  const auto unturned = pode_rl_controller::scan_to_rays(scan, 0.0, s);
  EXPECT_FLOAT_EQ(unturned[180], 1.2f);
  EXPECT_FLOAT_EQ(unturned[0], 0.5f);
}

// ---- 뒤가 가까우면 후진 막기 ----
TEST(ReverseGuard, BlocksReverseOnlyWhenTheRearIsClose)
{
  const PolicySpec s = spec();
  std::vector<float> rays(s.lidar.rays, static_cast<float>(s.lidar.obs_max));
  const pode_rl_controller::Twist2 back{-0.05, 0.8};
  const pode_rl_controller::Twist2 ahead{0.2, -0.3};
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s).v, -0.05);
  rays[0] = 0.10f;
  const auto blocked = pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s);
  EXPECT_DOUBLE_EQ(blocked.v, 0.0);
  EXPECT_DOUBLE_EQ(blocked.w, 0.8);
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(ahead, rays, 0.15, 30.0, s).v, 0.2);
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.0, 30.0, s).v, -0.05);
  rays[0] = 0.30f;
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s).v, -0.05);
}

TEST(ReverseGuard, LooksOnlyWithinTheRearHalfWidth)
{
  const PolicySpec s = spec();
  std::vector<float> rays(s.lidar.rays, static_cast<float>(s.lidar.obs_max));
  const pode_rl_controller::Twist2 back{-0.05, 0.0};
  rays[90] = 0.05f;
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s).v, -0.05);
  rays[329] = 0.05f;
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s).v, -0.05);
  rays[330] = 0.05f;
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s).v, 0.0);
  rays[330] = 2.0f;
  rays[30] = 0.05f;
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, rays, 0.15, 30.0, s).v, 0.0);
}

// ---- 정책 v_max와 단일 출처 속도 ----
TEST(NavSpeed, PolicyCheckedAgainstTheOneSource)
{
  const double nav = pode_rl_controller::load_nav_speed(NAV_SPEED_FILE);
  EXPECT_GT(nav, 0.0);
  EXPECT_EQ(pode_rl_controller::speed_mismatch(nav, nav), "");
  EXPECT_NE(pode_rl_controller::speed_mismatch(nav + 0.01, nav), "");
}

// ---- 출력 평활(지수 이동 평균) ----
TEST(Smoothing, StepInputMatchesTheClosedForm)
{
  pode_rl_controller::TargetSmoother ema(0.25, 0.5);
  const pode_rl_controller::Twist2 u{0.1, 1.0};
  for (int n = 1; n <= 30; ++n) {
    const auto y = ema.step(u, {0.0, 0.0});
    EXPECT_NEAR(y.v, 0.1 * (1.0 - std::pow(0.75, n)), 1e-12) << n;
    EXPECT_NEAR(y.w, 1.0 - std::pow(0.5, n), 1e-12) << n;
  }
}

TEST(Smoothing, StartsFromTheGivenCommandAfterReset)
{
  pode_rl_controller::TargetSmoother ema(1.0, 0.25);
  const pode_rl_controller::Twist2 u{0.08, -0.6};
  for (int n = 0; n < 7; ++n) {
    ema.step({0.12, 0.9}, {0.0, 0.0});
  }
  ema.reset();
  const pode_rl_controller::Twist2 s{0.03, 0.4};
  for (int n = 1; n <= 20; ++n) {
    const auto y = ema.step(u, n == 1 ? s : pode_rl_controller::Twist2{9.0, 9.0});
    EXPECT_EQ(y.v, u.v) << n;
    EXPECT_NEAR(y.w, u.w + (s.w - u.w) * std::pow(0.75, n), 1e-12) << n;
  }
  pode_rl_controller::TargetSmoother fresh(1.0, 0.25);
  ema.reset();
  for (int n = 0; n < 5; ++n) {
    const auto a = ema.step(u, s);
    const auto b = fresh.step(u, s);
    EXPECT_EQ(a.v, b.v);
    EXPECT_EQ(a.w, b.w);
  }
}

TEST(Smoothing, AlphaOutsideZeroOneIsRefused)
{
  using pode_rl_controller::TargetSmoother;
  for (const double bad : {0.0, -0.1, 1.01, std::nan("")}) {
    EXPECT_THROW(TargetSmoother(bad, 1.0), std::runtime_error) << bad;
    EXPECT_THROW(TargetSmoother(1.0, bad), std::runtime_error) << bad;
  }
  EXPECT_NO_THROW(TargetSmoother(1.0, 1.0));
  EXPECT_NO_THROW(TargetSmoother(0.25, 1e-3));
}

// ---- 스캔 가리기: 몸이 보이는 빔을 "돌아온 것 없음"으로 ----
TEST(ScanMask, MaskedBeamsBecomeNoReturnInTheBaseFrame)
{
  const PolicySpec s = spec();
  const int n = 720;
  pode_rl_controller::Scan scan{-M_PI, 2.0 * M_PI / n, 0.05, 12.0, std::vector<float>(n, 0.5f)};
  const auto mask = pode_rl_controller::parse_scan_mask({149.5, 153.5});
  pode_rl_controller::mask_scan(scan, M_PI, mask);
  int inf = 0;
  for (int i = 0; i < n; ++i) {
    if (std::isinf(scan.ranges[i])) {
      ++inf;
      EXPECT_GE(i, 299);
      EXPECT_LE(i, 307);
    } else {
      EXPECT_EQ(scan.ranges[i], 0.5f) << i;
    }
  }
  EXPECT_GE(inf, 7);
  EXPECT_LE(inf, 9);
  const auto rays = pode_rl_controller::scan_to_rays(scan, M_PI, s);
  for (int k = 0; k < s.lidar.rays; ++k) {
    const float want = (k >= 330 && k <= 333) ? static_cast<float>(s.lidar.obs_max) : 0.5f;
    EXPECT_EQ(rays[k], want) << k;
  }
}

TEST(ScanMask, RangesMeetAcrossTheBack)
{
  const int n = 720;
  pode_rl_controller::Scan scan{-M_PI, 2.0 * M_PI / n, 0.05, 12.0, std::vector<float>(n, 0.5f)};
  pode_rl_controller::mask_scan(
    scan, 0.0, pode_rl_controller::parse_scan_mask({170.0, 180.0, -180.0, -170.0}));
  EXPECT_TRUE(std::isinf(scan.ranges[0]));
  EXPECT_TRUE(std::isinf(scan.ranges[1]));
  EXPECT_TRUE(std::isinf(scan.ranges[719]));
  EXPECT_EQ(scan.ranges[30], 0.5f);
  EXPECT_EQ(scan.ranges[690], 0.5f);
  EXPECT_EQ(scan.ranges[360], 0.5f);
}

TEST(ScanMask, MalformedListsAreRefused)
{
  using pode_rl_controller::parse_scan_mask;
  EXPECT_TRUE(parse_scan_mask({}).empty());
  EXPECT_EQ(parse_scan_mask({149.5, 153.5, -10.0, 10.0}).size(), 2u);
  for (const auto & bad : std::vector<std::vector<double>>{
      {150.0}, {150.0, 153.0, 10.0}, {153.0, 150.0}, {150.0, 150.0}, {170.0, 190.0},
      {-200.0, -170.0}, {std::nan(""), 10.0}, {0.0, std::numeric_limits<double>::infinity()}})
  {
    EXPECT_THROW(parse_scan_mask(bad), std::runtime_error) << bad.size();
  }
}

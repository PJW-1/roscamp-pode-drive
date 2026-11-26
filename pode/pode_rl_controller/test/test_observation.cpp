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
  for (const auto & one : std::vector<std::vector<double>>{{170.0, 180.0}, {-180.0, -170.0}}) {
    pode_rl_controller::Scan back{-M_PI, 2.0 * M_PI / n, 0.05, 12.0, std::vector<float>(n, 0.5f)};
    pode_rl_controller::mask_scan(back, 0.0, pode_rl_controller::parse_scan_mask(one));
    EXPECT_TRUE(std::isinf(back.ranges[0])) << one[0];
  }
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

// ---- 가린 섹터 채우기(scan_mask_fill) ----
namespace
{
using pode_rl_controller::Scan;

// 빔 360개, -180도부터 1도 간격(lidar yaw 0): 빔 k가 레이 k이고 섹터 s는 빔 10s..10s+9다
Scan grid360(float r)
{
  return {-M_PI, 2.0 * M_PI / 360, 0.05, 12.0, std::vector<float>(360, r)};
}

void set_sector(Scan & scan, int s, float r)
{
  std::fill(scan.ranges.begin() + 10 * s, scan.ranges.begin() + 10 * s + 10, r);
}

struct Filled
{
  std::vector<float> rays;
  std::vector<float> before;
  std::vector<float> after;
  pode_rl_controller::MaskedSectors flags;
};

// 플러그인과 같은 차례: 가리기 -> 레이 -> 관측 -> 관측의 섹터 칸 채우기
Filled run_fill(Scan scan, double yaw, const std::vector<double> & mask_deg, const PolicySpec & s)
{
  const auto mask = pode_rl_controller::parse_scan_mask(mask_deg);
  pode_rl_controller::mask_scan(scan, yaw, mask);
  Filled f;
  f.rays = pode_rl_controller::scan_to_rays(scan, yaw, s);
  auto obs = pode_rl_controller::build_observation(f.rays, 1.0, 0.5, 0.1, -0.2, s);
  const auto plain = obs;
  f.before.assign(obs.begin(), obs.begin() + s.lidar.sectors);
  f.flags = pode_rl_controller::masked_sectors(scan, yaw, mask, s);
  pode_rl_controller::fill_masked_sectors(obs, f.flags, 1.0f);
  f.after.assign(obs.begin(), obs.begin() + s.lidar.sectors);
  // 섹터 칸 밖(목표와 속도)은 그대로다
  EXPECT_EQ(obs.size(), plain.size());
  for (size_t k = s.lidar.sectors; k < obs.size(); ++k) {
    EXPECT_EQ(obs[k], plain[k]) << k;
  }
  return f;
}

// 채워지는 섹터(empty)만 값이 바뀔 수 있다. 나머지는 한 값도 바뀌지 않는다
void expect_kept_outside(const Filled & f, const std::vector<int> & empty)
{
  for (size_t s = 0; s < f.after.size(); ++s) {
    const bool is_empty = std::find(empty.begin(), empty.end(), static_cast<int>(s)) != empty.end();
    EXPECT_EQ(f.flags.touched[s] && !f.flags.usable[s], is_empty) << s;
    if (!is_empty) {
      EXPECT_EQ(f.after[s], f.before[s]) << s;
    }
  }
}

struct Sides
{
  float left;
  float right;
  float want;
};
}  // namespace

TEST(ScanMaskFill, AFullyMaskedSectorReadsTheSmallerNeighbour)
{
  const PolicySpec s = spec();
  ASSERT_EQ(s.lidar.rays, 360);
  ASSERT_EQ(s.lidar.sectors, 36);
  ASSERT_DOUBLE_EQ(s.lidar.obs_max, 2.0);
  // 가림 -150.5..-140.5는 섹터 3(빔 -150..-141)을 통째로 가린다
  for (const auto & c : std::vector<Sides>{{1.2f, 0.9f, 0.45f}, {0.7f, 1.1f, 0.35f}}) {
    Scan scan = grid360(0.5f);
    set_sector(scan, 2, c.left);
    set_sector(scan, 4, c.right);
    const auto f = run_fill(scan, 0.0, {-150.5, -140.5}, s);
    EXPECT_FLOAT_EQ(f.before[3], 1.0f);
    EXPECT_FLOAT_EQ(f.after[3], c.want) << c.left << " " << c.right;
    expect_kept_outside(f, {3});
  }
}

TEST(ScanMaskFill, AnEmptyNeighbourIsSkipped)
{
  const PolicySpec s = spec();
  // 가림 -160.5..-140.5는 섹터 2와 3을 통째로 가린다
  // 3의 왼쪽은 2를 건너 1, 2의 오른쪽은 3을 건너 4다
  for (const auto & c : std::vector<Sides>{{0.8f, 1.5f, 0.4f}, {1.9f, 0.6f, 0.3f}}) {
    Scan scan = grid360(1.7f);
    set_sector(scan, 1, c.left);
    set_sector(scan, 4, c.right);
    const auto f = run_fill(scan, 0.0, {-160.5, -140.5}, s);
    EXPECT_FLOAT_EQ(f.after[2], c.want);
    EXPECT_FLOAT_EQ(f.after[3], c.want);
    expect_kept_outside(f, {2, 3});
  }
}

TEST(ScanMaskFill, NoneWithinFourGivesTheLimitAndFilledValuesAreNotReused)
{
  const PolicySpec s = spec();
  // 가림 -170.5..-80.5는 섹터 1..9를 통째로 가린다. 0은 0.3, 10은 0.4, 나머지는 0.5
  Scan scan = grid360(0.5f);
  set_sector(scan, 0, 0.3f);
  set_sector(scan, 10, 0.4f);
  const auto f = run_fill(scan, 0.0, {-170.5, -80.5}, s);
  // 1..4는 왼쪽 넷 안에 0이 있고, 6..9는 오른쪽 넷 안에 10이 있다
  for (int k = 1; k <= 4; ++k) {
    EXPECT_FLOAT_EQ(f.after[k], 0.15f) << k;
  }
  // 5는 양쪽 넷 안에 채우지 않은 섹터가 없다: 채워진 4나 6의 값을 쓰지 않고 상한이다
  EXPECT_FLOAT_EQ(f.after[5], 1.0f);
  for (int k = 6; k <= 9; ++k) {
    EXPECT_FLOAT_EQ(f.after[k], 0.2f) << k;
  }
  expect_kept_outside(f, {1, 2, 3, 4, 5, 6, 7, 8, 9});
}

TEST(ScanMaskFill, TheSearchWrapsAcrossSectorZero)
{
  const PolicySpec s = spec();
  // 가림 169.5..180, -180..-170.5는 섹터 35와 0을 통째로 가린다
  for (const auto & c : std::vector<Sides>{{0.6f, 1.4f, 0.3f}, {1.6f, 0.7f, 0.35f}}) {
    Scan scan = grid360(1.9f);
    set_sector(scan, 34, c.left);
    set_sector(scan, 1, c.right);
    const auto f = run_fill(scan, 0.0, {169.5, 180.0, -180.0, -170.5}, s);
    EXPECT_FLOAT_EQ(f.after[0], c.want);
    EXPECT_FLOAT_EQ(f.after[35], c.want);
    expect_kept_outside(f, {0, 35});
  }
}

TEST(ScanMaskFill, AnUnmaskedSectorWithNoReturnStaysAtTheLimit)
{
  const PolicySpec s = spec();
  Scan scan = grid360(0.5f);
  set_sector(scan, 18, std::numeric_limits<float>::infinity());
  set_sector(scan, 2, 0.8f);
  const auto f = run_fill(scan, 0.0, {-150.5, -140.5}, s);
  EXPECT_FLOAT_EQ(f.after[18], 1.0f);
  EXPECT_FLOAT_EQ(f.after[3], 0.25f);
  expect_kept_outside(f, {3});
}

TEST(ScanMaskFill, AMaskedSectorWithAUsableRayIsKept)
{
  const PolicySpec s = spec();
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::nanf("");
  // 가림 -158.5..-140.5: 섹터 3은 통째로, 섹터 2는 빔 -160, -159만 남는다
  struct Case
  {
    float b160;
    float b159;
    bool usable2;
    float want2;
    float want3;
  };
  for (const auto & c : std::vector<Case>{
      {3.5f, inf, true, 1.0f, 0.4f},  // 2 m보다 먼 유효 반환만 있다: 상한 그대로
      {0.3f, nan, true, 0.15f, 0.15f},  // 가까운 반환
      {0.01f, 15.0f, false, 0.25f, 0.25f},  // range 밖뿐이라 쓸 수 없다: 1의 값으로 채운다
      {inf, nan, false, 0.25f, 0.25f}})
  {
    Scan scan = grid360(0.5f);
    set_sector(scan, 2, 1.3f);
    set_sector(scan, 4, 0.8f);
    scan.ranges[20] = c.b160;
    scan.ranges[21] = c.b159;
    const auto f = run_fill(scan, 0.0, {-158.5, -140.5}, s);
    EXPECT_TRUE(f.flags.touched[2]);
    EXPECT_EQ(f.flags.usable[2], c.usable2) << c.b160;
    EXPECT_FLOAT_EQ(f.after[2], c.want2) << c.b160;
    EXPECT_FLOAT_EQ(f.after[3], c.want3) << c.b160;
    expect_kept_outside(f, c.usable2 ? std::vector<int>{3} : std::vector<int>{2, 3});
  }
}

TEST(ScanMaskFill, TouchedFollowsTheChosenBeam)
{
  const PolicySpec s = spec();
  // 빔 720개, lidar yaw 180도, 정수 도에서 어긋난 격자. 빔마다 다른 값으로 레이가 고른 빔을 읽는다
  const int n = 720;
  Scan scan{-3.1241, 2.0 * M_PI / 719, 0.05, 12.0, std::vector<float>(n)};
  for (int i = 0; i < n; ++i) {
    scan.ranges[i] = 0.1f + 0.001f * static_cast<float>(i);
  }
  const double yaw = M_PI;
  const auto rays = pode_rl_controller::scan_to_rays(scan, yaw, s);
  std::vector<int> chosen_by(n, -1);
  for (int k = 0; k < s.lidar.rays; ++k) {
    const int i = static_cast<int>(std::lround((rays[k] - 0.1f) / 0.001f));
    ASSERT_GE(i, 0);
    ASSERT_LT(i, n);
    chosen_by[i] = k;
  }
  auto beam_deg = [&](int i) {
      return std::remainder(
        (yaw + scan.angle_min + i * scan.angle_increment) * (180.0 / M_PI), 360.0);
    };
  // 섹터 4 안에서 고른 빔 하나와 고르지 않은 빔 하나를 찾는다
  int picked = -1;
  int skipped = -1;
  for (int i = 0; i < n; ++i) {
    const double d = beam_deg(i);
    if (d > -134.0 && d < -132.0) {
      (chosen_by[i] >= 0 ? picked : skipped) = i;
    }
  }
  ASSERT_GE(picked, 0);
  ASSERT_GE(skipped, 0);
  const int sector = chosen_by[picked] / 10;
  EXPECT_EQ(sector, 4);
  // 고르지 않은 빔만 가리면 어느 섹터도 가림에 걸리지 않고 아무것도 바뀌지 않는다
  {
    const double d = beam_deg(skipped);
    const std::vector<double> mask{d - 0.1, d + 0.1};
    Scan masked = scan;
    pode_rl_controller::mask_scan(masked, yaw, pode_rl_controller::parse_scan_mask(mask));
    EXPECT_EQ(
      std::count_if(
        masked.ranges.begin(), masked.ranges.end(), [](float r) {return std::isinf(r);}), 1);
    const auto f = run_fill(scan, yaw, mask, s);
    for (int k = 0; k < s.lidar.sectors; ++k) {
      EXPECT_FALSE(f.flags.touched[k]) << k;
    }
    EXPECT_EQ(f.after, f.before);
  }
  // 고른 빔을 가리면 그 레이의 섹터만 가림에 걸리고, 남은 레이가 있어 바뀌지 않는다
  {
    const double d = beam_deg(picked);
    const auto f = run_fill(scan, yaw, {d - 0.1, d + 0.1}, s);
    for (int k = 0; k < s.lidar.sectors; ++k) {
      EXPECT_EQ(f.flags.touched[k], k == sector) << k;
      EXPECT_TRUE(f.flags.usable[k]) << k;
    }
    EXPECT_EQ(f.after, f.before);
  }
}

TEST(ScanMaskFill, NoMaskChangesNothing)
{
  const PolicySpec s = spec();
  Scan scan = grid360(0.5f);
  set_sector(scan, 3, std::numeric_limits<float>::infinity());
  set_sector(scan, 7, std::nanf(""));
  const auto f = run_fill(scan, 0.0, {}, s);
  for (int k = 0; k < s.lidar.sectors; ++k) {
    EXPECT_FALSE(f.flags.touched[k]) << k;
  }
  EXPECT_EQ(f.after, f.before);
  EXPECT_FLOAT_EQ(f.after[3], 1.0f);
}

TEST(ScanMaskFill, RaysForTheRearGuardKeepTheMaskedValue)
{
  const PolicySpec s = spec();
  // 바로 뒤 섹터 0과 35를 가리고 이웃은 가깝다: 관측은 채워지지만 레이는 가린 그대로(상한)다
  Scan scan = grid360(0.1f);
  const std::vector<double> mask{169.5, 180.0, -180.0, -170.5};
  const auto f = run_fill(scan, 0.0, mask, s);
  EXPECT_FLOAT_EQ(f.after[0], 0.05f);
  Scan masked = scan;
  pode_rl_controller::mask_scan(masked, 0.0, pode_rl_controller::parse_scan_mask(mask));
  EXPECT_EQ(f.rays, pode_rl_controller::scan_to_rays(masked, 0.0, s));
  for (int k = 0; k < 10; ++k) {
    EXPECT_FLOAT_EQ(f.rays[k], 2.0f) << k;
    EXPECT_FLOAT_EQ(f.rays[350 + k], 2.0f) << k;
  }
  const pode_rl_controller::Twist2 back{-0.05, 0.0};
  EXPECT_DOUBLE_EQ(pode_rl_controller::block_reverse(back, f.rays, 0.15, 9.5, s).v, -0.05);
}

TEST(ScanMaskFill, FullyMaskedSectorsAreNamed)
{
  using pode_rl_controller::fully_masked_sectors;
  using pode_rl_controller::parse_scan_mask;
  const PolicySpec s = spec();
  EXPECT_TRUE(fully_masked_sectors({}, s).empty());
  // 넓은 가림은 섹터 3과 32를 통째로 덮고, 이웃 섹터에는 레이가 남는다
  EXPECT_EQ(
    fully_masked_sectors(parse_scan_mask({138.0, 157.0, -152.5, -137.0}), s),
    (std::vector<int>{3, 32}));
  // 기둥 무더기만 가리는 좁은 가림은 어느 섹터도 통째로 덮지 않는다
  EXPECT_TRUE(
    fully_masked_sectors(
      parse_scan_mask({140.0, 147.0, 149.0, 155.0, -150.5, -147.5, -144.0, -139.0}), s).empty());
  // 바로 뒤에서 만나는 두 범위는 첫 섹터와 끝 섹터를 덮는다
  EXPECT_EQ(
    fully_masked_sectors(parse_scan_mask({170.0, 180.0, -180.0, -171.0}), s),
    (std::vector<int>{0, 35}));
  // 레이 하나만 남아도 통째가 아니다
  EXPECT_TRUE(fully_masked_sectors(parse_scan_mask({-149.5, -140.5}), s).empty());
}

TEST(ScanMaskFill, NeedsAMask)
{
  using pode_rl_controller::check_scan_mask_fill;
  using pode_rl_controller::parse_scan_mask;
  EXPECT_THROW(check_scan_mask_fill(true, {}), std::runtime_error);
  EXPECT_NO_THROW(check_scan_mask_fill(true, parse_scan_mask({138.0, 157.0})));
  EXPECT_NO_THROW(check_scan_mask_fill(false, {}));
  EXPECT_NO_THROW(check_scan_mask_fill(false, parse_scan_mask({138.0, 157.0})));
}

TEST(ScanMaskFill, FlagsAreTheSameBeforeAndAfterMasking)
{
  const PolicySpec s = spec();
  // 가린 빔에 가까운 유효 반환이 남아 있어도(가리기 전의 scan) 그 빔은 쓸 수 있는 레이가 아니다
  Scan scan = grid360(0.5f);
  scan.ranges[20] = std::numeric_limits<float>::infinity();
  scan.ranges[21] = std::nanf("");
  const auto mask = pode_rl_controller::parse_scan_mask({-158.5, -140.5});
  const auto plain = pode_rl_controller::masked_sectors(scan, 0.0, mask, s);
  Scan masked = scan;
  pode_rl_controller::mask_scan(masked, 0.0, mask);
  const auto after = pode_rl_controller::masked_sectors(masked, 0.0, mask, s);
  EXPECT_EQ(plain.touched, after.touched);
  EXPECT_EQ(plain.usable, after.usable);
  EXPECT_TRUE(plain.touched[2]);
  EXPECT_FALSE(plain.usable[2]);
  EXPECT_TRUE(plain.touched[3]);
  EXPECT_FALSE(plain.usable[3]);
  EXPECT_TRUE(plain.usable[4]);
}

// ---- Python과 같은 답(make_mask_fill_cases.py로 만든 기대값) ----
TEST(ScanMaskFill, MatchesPythonOnGeneratedScans)
{
  const PolicySpec s = spec();
  const YAML::Node f = YAML::LoadFile(std::string(MASK_FILL_FIXTURE));
  ASSERT_EQ(f["fill_reach"].as<int>(), pode_rl_controller::kFillReach);
  ASSERT_DOUBLE_EQ(f["obs_max"].as<double>(), s.lidar.obs_max);
  const double obs_max = s.lidar.obs_max;
  int cases = 0;
  int filled = 0;
  double worst = 0.0;
  for (const auto & c : f["cases"]) {
    const std::string name = c["name"].as<std::string>();
    Scan scan{c["angle_min"].as<double>(), c["angle_increment"].as<double>(),
      f["range_min"].as<double>(), f["range_max"].as<double>(),
      c["ranges"].as<std::vector<float>>()};
    const auto out = run_fill(scan, c["yaw"].as<double>(), c["mask"].as<std::vector<double>>(), s);
    const auto before = c["before"].as<std::vector<double>>();
    const auto after = c["after"].as<std::vector<double>>();
    const auto touched = c["touched"].as<std::vector<bool>>();
    const auto usable = c["usable"].as<std::vector<bool>>();
    ASSERT_EQ(before.size(), static_cast<size_t>(s.lidar.sectors)) << name;
    for (int k = 0; k < s.lidar.sectors; ++k) {
      EXPECT_EQ(out.flags.touched[k], touched[k]) << name << " sector " << k;
      EXPECT_EQ(out.flags.usable[k], usable[k]) << name << " sector " << k;
      // 관측은 상한으로 나눈 값이다
      worst = std::max(
        {worst, gap(out.before[k], before[k] / obs_max), gap(out.after[k], after[k] / obs_max)});
      EXPECT_NEAR(out.before[k], before[k] / obs_max, kObsTol) << name << " sector " << k;
      EXPECT_NEAR(out.after[k], after[k] / obs_max, kObsTol) << name << " sector " << k;
      filled += touched[k] && !usable[k] ? 1 : 0;
    }
    ++cases;
  }
  std::printf("[mask_fill] %d cases, %d filled sectors, max |diff| %.3g\n", cases, filled, worst);
  EXPECT_EQ(cases, 20);
  EXPECT_GT(filled, 50);
}

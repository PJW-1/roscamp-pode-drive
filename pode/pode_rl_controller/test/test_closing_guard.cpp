#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pode_rl_controller/closing_guard.hpp"

using pode_rl_controller::ClosingGuard;
using pode_rl_controller::ClosingParams;
using pode_rl_controller::Twist2;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.05;
constexpr double kLidarX = -0.017;  // 시험 스캔의 라이다 자리(base에서 앞쪽 m)
const float kInf = std::numeric_limits<float>::infinity();

struct Rect
{
  double x0, y0, x1, y1;
};

struct Circle
{
  double x, y, r;
};

struct Pose
{
  double x, y, yaw;
};

using Scene = std::function<std::pair<std::vector<Rect>, std::vector<Circle>>(int)>;

const Rect kRoom{-1.0, -1.0, 1.5, 1.2};
const Rect kBox{0.30, -0.06, 0.42, 0.10};

ClosingGuard guard(
  double stop, double half, double lookback = 0.2, double spread = 8.0, double mps = 0.04,
  double hold = 0.3, int min_rays = 1)
{
  ClosingParams p;
  p.stop_range = stop;
  p.half_width_deg = half;
  p.lookback_s = lookback;
  p.spread_deg = spread;
  p.closing_mps = mps;
  p.hold_s = hold;
  p.min_rays = min_rays;
  return ClosingGuard(p);
}

// ---- 시험용 2D 스캔(test_task_spec._scan과 같은 식) ----
std::vector<float> scan(
  double ox, double oy, double yaw, const std::vector<Rect> & rects,
  const std::vector<Circle> & circles, double far = 9.0)
{
  std::vector<float> out(360);
  const double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 360; ++i) {
    const double a = yaw + (-180.0 + i) * (kPi / 180.0);
    const double dx = std::cos(a);
    const double dy = std::sin(a);
    double best = far;
    for (const auto & r : rects) {
      const double tx0 = (r.x0 - ox) / dx;
      const double tx1 = (r.x1 - ox) / dx;
      const double ty0 = (r.y0 - oy) / dy;
      const double ty1 = (r.y1 - oy) / dy;
      const double enter = std::max(std::min(tx0, tx1), std::min(ty0, ty1));
      const double leave = std::min(std::max(tx0, tx1), std::max(ty0, ty1));
      const bool hit = leave >= std::max(enter, 0.0);
      best = std::min(best, hit ? (enter > 0.0 ? enter : leave) : inf);
    }
    for (const auto & c : circles) {
      const double fx = ox - c.x;
      const double fy = oy - c.y;
      const double b = fx * dx + fy * dy;
      const double disc = b * b - (fx * fx + fy * fy - c.r * c.r);
      const double t = -b - std::sqrt(std::max(disc, 0.0));
      best = std::min(best, disc >= 0.0 && t > 0.0 ? t : inf);
    }
    out[i] = static_cast<float>(best);
  }
  return out;
}

struct StepOut
{
  bool blocked;
  Twist2 out;
  double front_min;
};

// 자세 열을 따라 스캔을 만들어 20 Hz로 넣는다(스캔 시각 = 지금 시각)
std::vector<StepOut> drive(
  ClosingGuard & g, const std::vector<Pose> & poses, const Scene & scene,
  const Twist2 & target = {0.12, 0.3})
{
  std::vector<StepOut> out;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto & p = poses[i];
    const auto objects = scene(static_cast<int>(i));
    const double ox = p.x + kLidarX * std::cos(p.yaw);
    const double oy = p.y + kLidarX * std::sin(p.yaw);
    const auto rays = scan(ox, oy, p.yaw, objects.first, objects.second);
    const double t = i * kDt;
    g.add_scan(rays, ox, oy, p.yaw, t, t);
    const Twist2 o = g.apply(target, t);
    const double front = *std::min_element(rays.begin() + 120, rays.begin() + 241);
    out.push_back({o.v != target.v, o, front});
  }
  return out;
}

// 정면 +-8도(또는 [lo, hi) 레이)만 front_m이고 나머지는 1 m인 스캔
std::vector<float> ramp_scan(double front_m, int lo = 172, int hi = 189)
{
  std::vector<float> rays(360, 1.0f);
  for (int k = lo; k < hi; ++k) {
    rays[k] = static_cast<float>(front_m);
  }
  return rays;
}

bool step_at(ClosingGuard & g, const std::vector<float> & rays, double t, const Twist2 & target)
{
  g.add_scan(rays, 0.0, 0.0, 0.0, t, t);
  return g.apply(target, t).v != target.v;
}

std::vector<bool> bools(std::initializer_list<std::pair<int, bool>> runs)
{
  std::vector<bool> out;
  for (const auto & r : runs) {
    out.insert(out.end(), r.first, r.second);
  }
  return out;
}
}  // namespace

// ---- 옛 스캔을 지금 자세로 옮기기 ----
TEST(ClosingStatic, MovesAnOldScanIntoThePoseOfNow)
{
  std::vector<float> rays(360, 5.0f);
  rays[180] = 1.0f;
  // 0.1 m 앞으로 갔다: 그 점은 정면 0.9 m
  auto out = pode_rl_controller::static_ranges(rays, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0);
  EXPECT_NEAR(out[180], 0.9, 1e-6);
  // 왼쪽으로 10도 돌았다: 그 점은 오른쪽 10도(칸 170)
  out = pode_rl_controller::static_ranges(rays, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0 * kPi / 180.0, 0);
  EXPECT_NEAR(out[170], 1.0, 1e-6);
  EXPECT_NEAR(out[180], 5.0, 1e-6);
  // 이웃 +-2 칸의 최솟값
  out = pode_rl_controller::static_ranges(rays, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2);
  for (int k = 178; k <= 182; ++k) {
    EXPECT_NEAR(out[k], 1.0, 1e-6) << k;
  }
  EXPECT_NEAR(out[177], 5.0, 1e-6);
  EXPECT_NEAR(out[183], 5.0, 1e-6);
  // 이웃은 한 바퀴를 넘어 이어진다(칸 0은 -180도, 359는 +179도)
  std::vector<float> back(360, 5.0f);
  back[0] = 1.0f;
  out = pode_rl_controller::static_ranges(back, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2);
  for (int k : {358, 359, 0, 1, 2}) {
    EXPECT_NEAR(out[k], 1.0, 1e-6) << k;
  }
  EXPECT_NEAR(out[357], 5.0, 1e-6);
  EXPECT_NEAR(out[3], 5.0, 1e-6);
  // 옛 점이 하나도 들어오지 않은 칸은 inf: 반지름 0.5 m의 점들을 3 m 앞에서 보면 모두 뒤쪽에 있다
  std::vector<float> ring(360, 0.5f);
  out = pode_rl_controller::static_ranges(ring, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0);
  EXPECT_TRUE(std::isinf(out[180]));
  EXPECT_NEAR(out[0], 2.5, 1e-6);
}

// ---- Python 시험과 같은 장면 ----
TEST(ClosingGuard, StaysQuietInAStaticSceneWhileTheRobotDrivesAndTurns)
{
  auto g = guard(0.50, 90.0);
  double x = 0.0, y = 0.0, yaw = -0.5;
  std::vector<Pose> poses;
  for (int i = 0; i < 30; ++i) {
    poses.push_back({x, y, yaw});
    x += 0.006 * std::cos(yaw);
    y += 0.006 * std::sin(yaw);
    yaw += 0.04;
  }
  const auto steps = drive(g, poses, [](int) {
        return std::make_pair(std::vector<Rect>{kRoom, kBox}, std::vector<Circle>{});
      });
  double closest = 1e9;
  for (const auto & s : steps) {
    closest = std::min(closest, s.front_min);
    EXPECT_FALSE(s.blocked);
    EXPECT_DOUBLE_EQ(s.out.v, 0.12);
    EXPECT_DOUBLE_EQ(s.out.w, 0.3);
  }
  EXPECT_LT(closest, 0.20);
}

TEST(ClosingGuard, StopsForSomethingThatComesCloserInsideTheCone)
{
  auto person = [](double sx, double sy, double tx, double ty) {
      const double n = std::hypot(tx, ty);
      const double ux = tx / n, uy = ty / n, step = 0.10 * kDt;
      return Scene([=](int i) {
               return std::make_pair(
                 std::vector<Rect>{kRoom},
                 std::vector<Circle>{{sx + ux * step * i, sy + uy * step * i, 0.04}});
             });
    };
  const std::vector<Pose> still(60, Pose{0.0, 0.0, 0.0});
  // 앞 왼쪽에서 로봇 쪽으로 온다: 0.26 m보다 가까워진 스텝부터 막는다
  auto g = guard(0.26, 60.0);
  auto steps = drive(g, still, person(0.50, 0.15, -0.50, -0.15));
  size_t first = 0;
  while (first < steps.size() && steps[first].front_min >= 0.26) {
    ++first;
  }
  EXPECT_GT(first, 10u);
  EXPECT_LT(first, 55u);
  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].blocked, i >= first) << i;
  }
  EXPECT_DOUBLE_EQ(steps[first].out.v, 0.0);
  EXPECT_DOUBLE_EQ(steps[first].out.w, 0.3);
  // 같은 자리에서 멀어진다: 가깝지만 막지 않는다
  g = guard(0.26, 60.0);
  steps = drive(g, still, person(0.15, 0.04, 0.50, 0.15));
  EXPECT_LT(steps[0].front_min, 0.26);
  for (const auto & s : steps) {
    EXPECT_FALSE(s.blocked);
  }
  // 뒤에서 온다: 부채꼴 밖이라 막지 않고, 부채꼴을 한 바퀴로 넓히면 막는다
  const auto behind = person(-0.50, -0.15, 0.50, 0.15);
  g = guard(0.26, 60.0);
  for (const auto & s : drive(g, still, behind)) {
    EXPECT_FALSE(s.blocked);
  }
  g = guard(0.26, 180.0);
  steps = drive(g, still, behind);
  EXPECT_TRUE(std::any_of(steps.begin(), steps.end(), [](const StepOut & s) {return s.blocked;}));
}

TEST(ClosingGuard, IgnoresAStandingThingTheRobotDrivesUpTo)
{
  auto g = guard(0.26, 60.0);
  std::vector<Pose> poses;
  for (int i = 0; i < 40; ++i) {
    poses.push_back({0.006 * i, 0.0, 0.0});
  }
  const auto steps = drive(g, poses, [](int) {
        return std::make_pair(std::vector<Rect>{kRoom}, std::vector<Circle>{{0.45, 0.02, 0.04}});
      });
  EXPECT_LT(steps.back().front_min, 0.20);
  for (const auto & s : steps) {
    EXPECT_FALSE(s.blocked);
  }
}

TEST(ClosingGuard, HoldsAfterTheLastTriggerAndNeverBlocksABackwardTarget)
{
  auto g = guard(0.26, 60.0);
  const Twist2 fwd{0.12, 0.3};
  std::vector<bool> blocked;
  // 0.40 m에서 0.2 m/s로 다가오다가 스텝 20에 0.20 m에서 선다
  for (int i = 0; i < 34; ++i) {
    blocked.push_back(step_at(g, ramp_scan(0.40 - 0.01 * std::min(i, 20)), i * kDt, fwd));
  }
  // 스텝 15부터 걸리고 마지막으로 걸린 스텝 23 뒤 0.3 s(6 스텝)를 유지한다
  EXPECT_EQ(blocked, bools({{15, false}, {15, true}, {4, false}}));
  // 걸린 채로도 후진 목표와 0은 그대로 둔다
  g = guard(0.26, 60.0);
  for (int i = 0; i < 18; ++i) {
    const Twist2 back = i % 2 ? Twist2{-0.05, 0.3} : Twist2{0.0, 0.3};
    g.add_scan(ramp_scan(0.40 - 0.01 * i), 0.0, 0.0, 0.0, i * kDt, i * kDt);
    const Twist2 out = g.apply(back, i * kDt);
    EXPECT_DOUBLE_EQ(out.v, back.v) << i;
    EXPECT_DOUBLE_EQ(out.w, back.w) << i;
  }
  EXPECT_TRUE(g.holding(17 * kDt));
  // 유지는 시간으로 끝난다: 마지막으로 걸린 뒤 hold_s 안은 막고 그 뒤는 풀린다
  EXPECT_DOUBLE_EQ(g.apply(fwd, 17 * kDt + 0.3).v, 0.0);
  EXPECT_DOUBLE_EQ(g.apply(fwd, 17 * kDt + 0.31).v, 0.12);
}

TEST(ClosingGuard, TreatsRaysWithoutAReturnAsFar)
{
  const Twist2 fwd{0.12, 0.3};
  std::vector<bool> blocked;
  for (const auto & where : {std::make_pair(172, 189), std::make_pair(344, 360)}) {
    auto g = guard(0.26, 180.0);
    for (int i = 0; i < 18; ++i) {
      auto rays = ramp_scan(0.40 - 0.01 * i, where.first, where.second);
      std::fill(rays.begin() + 100, rays.begin() + 110, std::nanf(""));
      std::fill(rays.begin() + 250, rays.begin() + 260, kInf);
      std::fill(rays.begin() + 60, rays.begin() + 70, -kInf);
      blocked.push_back(step_at(g, rays, i * kDt, fwd));
    }
  }
  EXPECT_EQ(
    blocked, bools({{15, false}, {3, true}, {15, false}, {3, true}}));
  // 돌아오지 않게 된 레이는 가까워진 것이 아니다
  auto g = guard(0.26, 60.0);
  for (int i = 0; i < 12; ++i) {
    EXPECT_FALSE(step_at(g, ramp_scan(i < 6 ? 0.30 : std::nan("")), i * kDt, fwd)) << i;
  }
}

TEST(ClosingGuard, LongLookbackCatchesAPersonCrossingDeadAheadAndTheShortOneMisses)
{
  const double speed = 0.08;
  const int n = 110;
  const Scene scene = [speed](int i) {
      return std::make_pair(
        std::vector<Rect>{{-1.0, -0.4, 2.0, 0.4}},
        std::vector<Circle>{{0.22, 0.30 - speed * kDt * i, 0.04}});
    };
  const std::vector<Pose> poses(n, Pose{0.0, 0.0, 0.0});
  auto short_g = guard(0.26, 60.0);
  auto wide_g = guard(0.26, 60.0, 1.0, 4.0, 0.08);
  const auto a = drive(short_g, poses, scene);
  const auto b = drive(wide_g, poses, scene);
  int ahead = 0;
  for (int i = 0; i < n; ++i) {
    if (std::abs(0.30 - speed * kDt * i) < 0.03) {
      ++ahead;
      EXPECT_FALSE(a[i].blocked) << i;
      EXPECT_TRUE(b[i].blocked) << i;
    }
  }
  EXPECT_GT(ahead, 10);
}

TEST(ClosingGuard, MinRaysWantsThatManyNeighbouringRays)
{
  const Twist2 fwd{0.12, 0.3};
  auto stops = [&](int width, int min_rays) {
      auto g = guard(0.26, 60.0, 0.2, 8.0, 0.04, 0.3, min_rays);
      bool any = false;
      for (int i = 0; i < 18; ++i) {
        any = step_at(g, ramp_scan(0.40 - 0.01 * i, 180, 180 + width), i * kDt, fwd) || any;
      }
      return any;
    };
  EXPECT_TRUE(stops(1, 1));
  EXPECT_TRUE(stops(2, 2));
  EXPECT_TRUE(stops(3, 3));
  EXPECT_FALSE(stops(1, 2));
  EXPECT_FALSE(stops(2, 3));
  // 떨어진 레이 둘은 이웃이 아니다
  auto g = guard(0.26, 60.0, 0.2, 8.0, 0.04, 0.3, 2);
  for (int i = 0; i < 18; ++i) {
    auto rays = ramp_scan(0.40 - 0.01 * i, 170, 171);
    rays[190] = static_cast<float>(0.40 - 0.01 * i);
    EXPECT_FALSE(step_at(g, rays, i * kDt, fwd)) << i;
  }
  // 배열의 양 끝(-180도와 +179도)은 이어 보지 않는다
  g = guard(0.26, 180.0, 0.2, 8.0, 0.04, 0.3, 2);
  for (int i = 0; i < 18; ++i) {
    auto rays = ramp_scan(0.40 - 0.01 * i, 359, 360);
    rays[0] = rays[359];
    EXPECT_FALSE(step_at(g, rays, i * kDt, fwd)) << i;
  }
}

// ---- 기준 스캔을 시각으로 고르기 ----
TEST(ClosingTiming, ScansAt10HzWithControlAt20Hz)
{
  auto g = guard(0.26, 60.0);
  const Twist2 fwd{0.12, 0.3};
  std::vector<bool> blocked;
  // 스캔 k는 0.1k s에 오고 0.40 m에서 0.2 m/s로 다가오다 1 s에 0.20 m에서 선다. 제어는 0.05 s마다
  for (int j = 0; j < 40; ++j) {
    const int k = j / 2;
    const auto rays = ramp_scan(0.40 - 0.02 * std::min(k, 10));
    const size_t before = g.history_size();
    const bool hit = g.add_scan(rays, 0.0, 0.0, 0.0, 0.1 * k, j * kDt);
    if (j % 2) {
      // 같은 스캔이 다시 오면 넣지도 판정하지도 않는다
      EXPECT_FALSE(hit) << j;
      EXPECT_EQ(g.history_size(), before) << j;
    }
    blocked.push_back(g.apply(fwd, j * kDt).v == 0.0);
  }
  // 스캔 8(0.24 m)부터 걸리고 마지막은 스캔 11(1.1 s)이다. 그 뒤 0.3 s(1.4 s, 제어 28)까지 막는다
  EXPECT_EQ(blocked, bools({{16, false}, {13, true}, {11, false}}));
}

TEST(ClosingTiming, UnevenIntervalsPickTheReferenceByTimeAndDivideByTheRealGap)
{
  const Twist2 fwd{0.12, 0.3};
  // 0.03 s 간격: 0.01 m 다가온 것은 첫 두 스캔 사이뿐이다. 0.21 s의 기준은 0 s 스캔(일곱 개 전)이다
  auto g = guard(0.26, 60.0);
  std::vector<bool> hits;
  for (int k = 0; k <= 10; ++k) {
    hits.push_back(g.add_scan(ramp_scan(k == 0 ? 0.20 : 0.19), 0.0, 0.0, 0.0, 0.03 * k, 0.03 * k));
  }
  EXPECT_EQ(hits, bools({{7, false}, {1, true}, {3, false}}));
  // 0.25 s 끊긴 뒤의 스캔은 0.25 s 전 스캔과 견주고 0.25 s로 나눈다: 0.009 m / 0.25 s = 0.036 m/s
  g = guard(0.26, 60.0);
  const std::vector<double> stamps{0.0, 0.05, 0.10, 0.35};
  const std::vector<double> front{0.215, 0.212, 0.209, 0.200};
  for (size_t k = 0; k < stamps.size(); ++k) {
    EXPECT_FALSE(g.add_scan(ramp_scan(front[k]), 0.0, 0.0, 0.0, stamps[k], stamps[k])) << k;
  }
  // 같은 간격에서 0.012 m 다가오면(0.048 m/s) 걸린다
  g = guard(0.26, 60.0);
  for (size_t k = 0; k < 3; ++k) {
    g.add_scan(ramp_scan(front[k]), 0.0, 0.0, 0.0, stamps[k], stamps[k]);
  }
  EXPECT_TRUE(g.add_scan(ramp_scan(0.197), 0.0, 0.0, 0.0, 0.35, 0.35));
  EXPECT_DOUBLE_EQ(g.apply(fwd, 0.35).v, 0.0);
}

TEST(ClosingTiming, AScanGapLeavesTheReferenceTooOldToJudge)
{
  auto g = guard(0.26, 60.0);
  const Twist2 fwd{0.12, 0.3};
  // 0.5 s까지 0.40 m에 서 있다가, 스캔이 1.05 s까지 끊긴 사이 0.20 m로 왔고 0.1 m/s로 계속 온다
  for (int i = 0; i <= 10; ++i) {
    EXPECT_FALSE(step_at(g, ramp_scan(0.40), i * kDt, fwd)) << i;
  }
  std::vector<bool> hits;
  for (int m = 0; m < 8; ++m) {
    const double t = (21 + m) * kDt;
    hits.push_back(g.add_scan(ramp_scan(0.20 - 0.005 * m), 0.0, 0.0, 0.0, t, t));
  }
  // 기준(0.5 s)이 0.3 s보다 오래된 동안은 판정하지 않고, 다시 0.2 s를 채운 1.25 s부터 걸린다
  EXPECT_EQ(hits, bools({{4, false}, {4, true}}));
  // 기준보다 오래된 스캔은 버린다
  EXPECT_LE(g.history_size(), 5u);
}

TEST(ClosingTiming, ClockGoingBackClearsTheHistoryAndTheHold)
{
  auto g = guard(0.26, 60.0);
  const Twist2 fwd{0.12, 0.3};
  bool any = false;
  for (int i = 0; i < 10; ++i) {
    const double t = 1.0 + i * kDt;
    any = step_at(g, ramp_scan(0.30 - 0.01 * i), t, fwd) || any;
  }
  ASSERT_TRUE(any);
  ASSERT_TRUE(g.holding(1.45));
  // 시뮬 재시작처럼 시각이 0.2 s로 돌아간다: 그 스캔은 넣지 않고 이력과 유지를 비운다
  EXPECT_FALSE(g.add_scan(ramp_scan(0.10), 0.0, 0.0, 0.0, 0.2, 0.2));
  EXPECT_EQ(g.history_size(), 0u);
  EXPECT_FALSE(g.holding(0.2));
  EXPECT_DOUBLE_EQ(g.apply(fwd, 0.2).v, 0.12);
  // 그 뒤 스캔은 다시 0.2 s를 채워야 판정한다
  std::vector<bool> hits;
  for (int m = 0; m < 6; ++m) {
    const double t = 0.25 + m * kDt;
    hits.push_back(g.add_scan(ramp_scan(0.25 - 0.01 * m), 0.0, 0.0, 0.0, t, t));
  }
  EXPECT_EQ(hits, bools({{4, false}, {2, true}}));
}

// ---- 인자 ----
TEST(ClosingParams, DefaultIsOffAndBadValuesStop)
{
  ClosingGuard off;
  EXPECT_FALSE(off.enabled());
  const Twist2 fwd{0.12, 0.3};
  for (int i = 0; i < 18; ++i) {
    EXPECT_FALSE(off.add_scan(ramp_scan(0.40 - 0.02 * i), 0.0, 0.0, 0.0, i * kDt, i * kDt));
    EXPECT_DOUBLE_EQ(off.apply(fwd, i * kDt).v, 0.12);
  }
  EXPECT_EQ(off.history_size(), 0u);
  EXPECT_FALSE(guard(-1.0, 60.0).enabled());
  EXPECT_TRUE(guard(0.26, 60.0).enabled());
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  auto bad = [](std::function<void(ClosingParams &)> set) {
      ClosingParams p;
      set(p);
      return p;
    };
  const std::vector<ClosingParams> rejected{
    bad([&](ClosingParams & p) {p.stop_range = nan;}),
    bad([&](ClosingParams & p) {p.stop_range = inf;}),
    bad([&](ClosingParams & p) {p.half_width_deg = -1.0;}),
    bad([&](ClosingParams & p) {p.half_width_deg = 181.0;}),
    bad([&](ClosingParams & p) {p.lookback_s = 0.0;}),
    bad([&](ClosingParams & p) {p.lookback_s = nan;}),
    bad([&](ClosingParams & p) {p.spread_deg = -1.0;}),
    bad([&](ClosingParams & p) {p.closing_mps = 0.0;}),
    bad([&](ClosingParams & p) {p.closing_mps = inf;}),
    bad([&](ClosingParams & p) {p.hold_s = -0.1;}),
    bad([&](ClosingParams & p) {p.hold_s = nan;}),
    bad([&](ClosingParams & p) {p.min_rays = 0;}),
  };
  for (size_t k = 0; k < rejected.size(); ++k) {
    EXPECT_THROW(ClosingGuard{rejected[k]}, std::runtime_error) << k;
  }
  EXPECT_NO_THROW(guard(0.26, 0.0, 0.05, 0.0, 0.01, 0.0, 1));
}

// ---- Python ClosingGuard와 같은 판정(make_closing_cases.py로 만든 기대값) ----
TEST(ClosingParity, MatchesThePythonGuardOnRecordedScenes)
{
  const YAML::Node f = YAML::LoadFile(std::string(CLOSING_FIXTURE));
  ASSERT_GE(f["scenes"].size(), 3u);
  // 플러그인의 기본값은 긴 창 장면(crossing_long)의 값과 같다
  const ClosingParams def;
  ASSERT_EQ(f["scenes"][2]["name"].as<std::string>(), "crossing_long");
  const YAML::Node p0 = f["scenes"][2]["params"];
  EXPECT_DOUBLE_EQ(p0["half_width_deg"].as<double>(), def.half_width_deg);
  EXPECT_DOUBLE_EQ(p0["lookback_s"].as<double>(), def.lookback_s);
  EXPECT_DOUBLE_EQ(p0["spread_deg"].as<double>(), def.spread_deg);
  EXPECT_DOUBLE_EQ(p0["closing_mps"].as<double>(), def.closing_mps);
  EXPECT_DOUBLE_EQ(p0["hold_s"].as<double>(), def.hold_s);
  EXPECT_EQ(p0["min_rays"].as<int>(), def.min_rays);
  int hits_seen = 0;
  for (const auto & sc : f["scenes"]) {
    const std::string name = sc["name"].as<std::string>();
    const YAML::Node p = sc["params"];
    const YAML::Node steps = sc["steps"];
    auto run = [&](size_t first) {
        auto g = guard(
          p["stop_range"].as<double>(), p["half_width_deg"].as<double>(),
          p["lookback_s"].as<double>(), p["spread_deg"].as<double>(),
          p["closing_mps"].as<double>(), p["hold_s"].as<double>(), p["min_rays"].as<int>());
        std::vector<bool> hit;
        for (size_t k = 0; k < steps.size(); ++k) {
          if (k < first) {
            hit.push_back(false);
            continue;
          }
          const YAML::Node st = steps[k];
          const double t = st["stamp"].as<double>();
          const auto o = st["origin"].as<std::vector<double>>();
          hit.push_back(
            g.add_scan(
              st["rays"].as<std::vector<float>>(), o[0], o[1], st["yaw"].as<double>(), t, t));
          const double v = st["v"].as<double>();
          const Twist2 out = g.apply({v, 0.3}, t);
          EXPECT_EQ(hit.back(), st["hit"].as<bool>()) << name << " step " << k;
          EXPECT_EQ(out.v != v, st["blocked"].as<bool>()) << name << " step " << k;
          // Python의 목표는 float32라 값은 그 정밀도로 견준다(걸림과 막음은 위에서 그대로 견줌)
          EXPECT_NEAR(out.v, st["v_out"].as<double>(), 1e-6) << name << " step " << k;
          EXPECT_DOUBLE_EQ(out.w, 0.3) << name << " step " << k;
        }
        return hit;
      };
    // Python은 리셋 뒤 첫 스캔을 기준으로 쓰지 않으므로 스텝 first_step부터 넣고 견준다
    const auto hit = run(sc["first_step"].as<size_t>());
    hits_seen += static_cast<int>(std::count(hit.begin(), hit.end(), true));
  }
  EXPECT_GT(hits_seen, 20);
}

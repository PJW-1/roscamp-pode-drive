// 계산부 단위 시험: 직선 맞추기, 기대 거리, 오차 부호, 보정 상한, 경로 옮기기
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "pode_follow_wall/wall_geometry.hpp"

using pode_follow_wall::FitParams;
using pode_follow_wall::LineFit;
using pode_follow_wall::Point2;
using pode_follow_wall::Pose2;
using pode_follow_wall::WallParams;

namespace
{

constexpr double kStep = 0.0125;  // C1 각도 간격 근처

// 빈 지도(칸 0.01 m, 원점 0)
nav_msgs::msg::OccupancyGrid emptyGrid(int w = 200, int h = 200, double res = 0.01)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = res;
  g.info.width = w;
  g.info.height = h;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(static_cast<std::size_t>(w * h), 0);
  return g;
}

// 가로 벽: 칸 줄 [row, row + rows)를 점유로
void addRow(nav_msgs::msg::OccupancyGrid & g, int row, int rows = 3, int8_t value = 100)
{
  for (int r = row; r < row + rows; ++r) {
    for (unsigned c = 0; c < g.info.width; ++c) {
      g.data[r * g.info.width + c] = value;
    }
  }
}

// 지도의 가로 벽 y = wall_y를 참 자세에서 본 몸 기준 점
std::vector<Point2> scanHorizontalWall(const Pose2 & truth, double wall_y, double max_range = 2.0)
{
  std::vector<Point2> pts;
  for (double a = -M_PI; a < M_PI; a += kStep) {
    const double s = std::sin(truth.yaw + a);
    if (std::abs(s) < 1e-9) {
      continue;
    }
    const double t = (wall_y - truth.y) / s;
    if (t > 0.0 && t <= max_range) {
      pts.push_back({t * std::cos(a), t * std::sin(a)});
    }
  }
  return pts;
}

// 몸 기준 직선 y = c 위의 점(x 구간)
std::vector<Point2> linePoints(double x0, double x1, double y, double dx = 0.005)
{
  std::vector<Point2> pts;
  for (double x = x0; x <= x1 + 1e-9; x += dx) {
    pts.push_back({x, y});
  }
  return pts;
}

}  // namespace

// ---- 직선 맞추기 ----

TEST(FitLine, CleanWall)
{
  const LineFit f = pode_follow_wall::fitLine(linePoints(-0.1, 0.1, 0.08), FitParams{});
  ASSERT_TRUE(f.ok);
  EXPECT_NEAR(f.dist, 0.08, 1e-9);
  EXPECT_NEAR(f.nx, 0.0, 1e-9);
  EXPECT_NEAR(f.ny, 1.0, 1e-9);
  EXPECT_NEAR(f.rms, 0.0, 1e-9);
  EXPECT_NEAR(f.start.x, -0.1, 1e-9);
  EXPECT_NEAR(f.end.x, 0.1, 1e-6);
}

TEST(FitLine, NoiseAndOutliers)
{
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.002);
  auto pts = linePoints(-0.12, 0.12, 0.08);
  for (auto & p : pts) {
    p.y += noise(rng);
  }
  // 벽 안쪽에 떨어진 점(물체, 반사)
  for (int i = 0; i < 6; ++i) {
    pts.push_back({-0.05 + 0.02 * i, 0.03});
  }
  const LineFit f = pode_follow_wall::fitLine(pts, FitParams{});
  ASSERT_TRUE(f.ok);
  EXPECT_NEAR(f.dist, 0.08, 0.002);
  EXPECT_LT(std::abs(f.dy), std::sin(0.03));
  EXPECT_LE(f.inliers, static_cast<int>(pts.size()) - 6);
}

TEST(FitLine, BrokenWall)
{
  auto pts = linePoints(-0.15, -0.05, 0.09);
  const auto more = linePoints(0.05, 0.15, 0.09);
  pts.insert(pts.end(), more.begin(), more.end());
  const LineFit f = pode_follow_wall::fitLine(pts, FitParams{});
  ASSERT_TRUE(f.ok);
  EXPECT_NEAR(f.dist, 0.09, 1e-9);
  EXPECT_EQ(f.inliers, static_cast<int>(pts.size()));
}

TEST(FitLine, RightWall)
{
  const LineFit f = pode_follow_wall::fitLine(linePoints(-0.1, 0.1, -0.07), FitParams{});
  ASSERT_TRUE(f.ok);
  EXPECT_NEAR(f.dist, 0.07, 1e-9);
  EXPECT_NEAR(f.ny, -1.0, 1e-9);
}

TEST(FitLine, CornerPicksDominantWall)
{
  // 옆 벽 y = 0.08과 앞 벽 x = 0.06이 만나는 모서리
  auto pts = linePoints(-0.15, 0.06, 0.08);
  for (double y = 0.09; y <= 0.13; y += 0.01) {
    pts.push_back({0.06, y});
  }
  const LineFit f = pode_follow_wall::fitLine(pts, FitParams{});
  ASSERT_TRUE(f.ok);
  // 모서리 근처 앞 벽 점 하나가 인라이어로 들어와도 데드밴드(5 mm)보다 훨씬 작다
  EXPECT_NEAR(f.dist, 0.08, 0.001);
  EXPECT_NEAR(std::abs(f.ny), 1.0, 1e-3);
}

TEST(FitLine, TooFewOrScattered)
{
  EXPECT_FALSE(pode_follow_wall::fitLine(linePoints(0.0, 0.02, 0.08), FitParams{}).ok);
  // 흩어진 점: 잔차가 커서 믿지 않는다
  std::vector<Point2> pts;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-0.1, 0.1);
  for (int i = 0; i < 40; ++i) {
    pts.push_back({u(rng), 0.1 + u(rng)});
  }
  FitParams p;
  p.ransac_threshold = 0.05;
  EXPECT_FALSE(pode_follow_wall::fitLine(pts, p).ok);
}

// ---- 창 ----

TEST(Window, SidesAndRange)
{
  const std::vector<Point2> pts = {
    {0.0, 0.1},  // 왼쪽 90도
    {0.0, -0.1},  // 오른쪽
    {0.1, 0.0},  // 앞
    {0.0, 0.5},  // 멀다
    {0.1, 0.1},  // 45도, 창 밖
  };
  pode_follow_wall::WindowParams w;
  const auto left = pode_follow_wall::selectWindow(pts, 1, w);
  ASSERT_EQ(left.size(), 1u);
  EXPECT_DOUBLE_EQ(left[0].y, 0.1);
  const auto right = pode_follow_wall::selectWindow(pts, -1, w);
  ASSERT_EQ(right.size(), 1u);
  EXPECT_DOUBLE_EQ(right[0].y, -0.1);
}

// ---- 기대 거리 ----

TEST(Raycast, ExactDistanceToCellBoundary)
{
  auto g = emptyGrid();
  addRow(g, 100);  // 벽 면 y = 1.00
  // 지도 해상도가 float32라 1e-6까지만 본다
  EXPECT_NEAR(pode_follow_wall::raycast(g, 0.5, 0.92, M_PI / 2, 0.5, 65), 0.08, 1e-6);
  EXPECT_NEAR(pode_follow_wall::raycast(g, 0.5, 0.92, M_PI / 4, 0.5, 65), 0.08 * std::sqrt(2.0), 1e-6);
  // 짧은 거리 안에 없으면 음수
  EXPECT_LT(pode_follow_wall::raycast(g, 0.5, 0.92, M_PI / 2, 0.05, 65), 0.0);
  // 반대쪽에는 벽이 없다
  EXPECT_LT(pode_follow_wall::raycast(g, 0.5, 0.92, -M_PI / 2, 0.5, 65), 0.0);
}

TEST(Raycast, UnknownIsNotWall)
{
  auto g = emptyGrid();
  addRow(g, 100, 3, -1);
  EXPECT_LT(pode_follow_wall::raycast(g, 0.5, 0.92, M_PI / 2, 0.5, 65), 0.0);
}

TEST(ExpectedPoints, FollowsEstimatedYaw)
{
  auto g = emptyGrid();
  addRow(g, 100);
  // 몸이 0.1 rad 틀어져도 수직 거리는 같다
  const Pose2 pose{0.5, 0.92, 0.1};
  const auto pts = scanHorizontalWall(pose, 1.0, 0.3);
  const auto exp = pode_follow_wall::expectedPoints(g, pose, pts, 0.4, 65);
  ASSERT_EQ(exp.size(), pts.size());
  const LineFit f = pode_follow_wall::fitLine(exp, FitParams{});
  ASSERT_TRUE(f.ok);
  EXPECT_NEAR(f.dist, 0.08, 1e-6);
}

// ---- 오차 e ----

TEST(MeasureWall, FollowErrorGivesZero)
{
  // 추정 자세가 맞으면 로봇이 경로에서 어디에 있든 e = 0
  auto g = emptyGrid();
  addRow(g, 100);
  WallParams wp;
  for (double y : {0.90, 0.92, 0.935}) {
    const Pose2 pose{0.5, y, 0.05};
    const auto m = pode_follow_wall::measureWall(scanHorizontalWall(pose, 1.0), g, pose, wp);
    ASSERT_TRUE(m.valid) << y;
    EXPECT_EQ(m.side, 1);
    EXPECT_NEAR(m.error, 0.0, 1e-6) << y;
    EXPECT_NEAR(m.measured.dist, 1.0 - y, 1e-6);
  }
}

TEST(MeasureWall, LocalizationErrorSign)
{
  auto g = emptyGrid();
  addRow(g, 100);
  WallParams wp;
  const Pose2 estimate{0.5, 0.92, 0.0};
  // 실제로는 벽에서 2 cm 더 멀다: e = +0.02
  const Pose2 far{0.5, 0.90, 0.0};
  auto m = pode_follow_wall::measureWall(scanHorizontalWall(far, 1.0), g, estimate, wp);
  ASSERT_TRUE(m.valid);
  EXPECT_NEAR(m.error, 0.02, 1e-6);
  EXPECT_GT(m.measured.ny, 0.0);  // 벽 쪽 법선은 왼쪽
  // 실제로는 1.5 cm 더 가깝다: e = -0.015
  const Pose2 near{0.5, 0.935, 0.0};
  m = pode_follow_wall::measureWall(scanHorizontalWall(near, 1.0), g, estimate, wp);
  ASSERT_TRUE(m.valid);
  EXPECT_NEAR(m.error, -0.015, 1e-6);
}

TEST(MeasureWall, RightWallSign)
{
  // 몸 오른쪽 벽: 칸 줄 97~99, 위 면 y = 1.00, 동쪽을 본다
  auto g = emptyGrid();
  addRow(g, 97);
  WallParams wp;
  const Pose2 estimate{0.5, 1.05, 0.0};
  const Pose2 far{0.5, 1.07, 0.0};  // 실제로는 2 cm 멀다
  const auto m = pode_follow_wall::measureWall(scanHorizontalWall(far, 1.0), g, estimate, wp);
  ASSERT_TRUE(m.valid);
  EXPECT_EQ(m.side, -1);
  EXPECT_NEAR(m.error, 0.02, 1e-6);
  EXPECT_LT(m.measured.ny, 0.0);  // 벽 쪽 법선은 오른쪽
}

TEST(MeasureWall, ObstacleNotInMapIsIgnored)
{
  // 라이다에는 벽이 있지만 지도에는 없다(사람, 상자)
  const auto g = emptyGrid();
  const Pose2 pose{0.5, 0.92, 0.0};
  const auto m = pode_follow_wall::measureWall(scanHorizontalWall(pose, 1.0), g, pose, WallParams{});
  EXPECT_FALSE(m.valid);
}

TEST(MeasureWall, LargeErrorIsRejected)
{
  auto g = emptyGrid();
  addRow(g, 100);
  // 지도 벽보다 8 cm 앞에 물체가 있다(max_error 0.06 넘음)
  const Pose2 pose{0.5, 0.82, 0.0};
  const auto m = pode_follow_wall::measureWall(scanHorizontalWall(pose, 0.92), g, pose, WallParams{});
  EXPECT_FALSE(m.valid);
}

TEST(MeasureWall, AngleMismatchIsRejected)
{
  auto g = emptyGrid();
  addRow(g, 100);
  // 라이다 벽은 0.3 rad 기울었다
  const Pose2 pose{0.5, 0.92, 0.0};
  std::vector<Point2> pts;
  for (double a = 0.9; a < 2.3; a += kStep) {
    // 몸 기준 직선 x sin(0.3) - y cos(0.3) + 0.08 = 0 과 만나는 점
    const double denom = std::cos(a) * std::sin(0.3) - std::sin(a) * std::cos(0.3);
    const double t = -0.08 / denom;
    if (t > 0.0 && t < 0.3) {
      pts.push_back({t * std::cos(a), t * std::sin(a)});
    }
  }
  const auto m = pode_follow_wall::measureWall(pts, g, pose, WallParams{});
  EXPECT_FALSE(m.valid);
}

TEST(MeasureWall, NoWallInWindow)
{
  auto g = emptyGrid();
  addRow(g, 100);
  const Pose2 pose{0.5, 0.50, 0.0};  // 벽이 0.5 m 떨어져 창(0.3 m) 밖
  const auto m = pode_follow_wall::measureWall(scanHorizontalWall(pose, 1.0), g, pose, WallParams{});
  EXPECT_FALSE(m.valid);
  EXPECT_EQ(m.side, 0);
}

TEST(MeasureWall, PicksNearerWall)
{
  // 위 벽 면 y = 1.00, 아래 벽 면 y = 0.80(칸 77~79)
  auto g = emptyGrid();
  addRow(g, 100);
  addRow(g, 77);
  const Pose2 pose{0.5, 0.88, 0.0};  // 아래 벽까지 0.08, 위 벽까지 0.12
  auto pts = scanHorizontalWall(pose, 1.0);
  const auto low = scanHorizontalWall(pose, 0.80);
  pts.insert(pts.end(), low.begin(), low.end());
  const auto m = pode_follow_wall::measureWall(pts, g, pose, WallParams{});
  ASSERT_TRUE(m.valid);
  EXPECT_EQ(m.side, -1);
  EXPECT_NEAR(m.measured.dist, 0.08, 1e-6);
}

// ---- 보정 상한, 데드밴드, 부호 ----

TEST(Correction, DeadbandCapAndSign)
{
  EXPECT_DOUBLE_EQ(pode_follow_wall::shapeCorrection(0.004, 0.005, 0.02), 0.0);
  EXPECT_DOUBLE_EQ(pode_follow_wall::shapeCorrection(-0.004, 0.005, 0.02), 0.0);
  EXPECT_DOUBLE_EQ(pode_follow_wall::shapeCorrection(0.012, 0.005, 0.02), 0.012);
  EXPECT_DOUBLE_EQ(pode_follow_wall::shapeCorrection(-0.012, 0.005, 0.02), -0.012);
  EXPECT_DOUBLE_EQ(pode_follow_wall::shapeCorrection(0.05, 0.005, 0.02), 0.02);
  EXPECT_DOUBLE_EQ(pode_follow_wall::shapeCorrection(-0.05, 0.005, 0.02), -0.02);
}

// ---- 경로 ----

namespace
{

nav_msgs::msg::Path makePath(const std::vector<Point2> & corners, double spacing = 0.05)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (std::size_t k = 0; k + 1 < corners.size(); ++k) {
    const double len = std::hypot(corners[k + 1].x - corners[k].x, corners[k + 1].y - corners[k].y);
    const int n = static_cast<int>(std::round(len / spacing));
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = corners[k].x + (corners[k + 1].x - corners[k].x) * i / n;
      p.pose.position.y = corners[k].y + (corners[k + 1].y - corners[k].y) * i / n;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
  }
  geometry_msgs::msg::PoseStamped last;
  last.header.frame_id = "map";
  last.pose.position.x = corners.back().x;
  last.pose.position.y = corners.back().y;
  last.pose.orientation.w = 1.0;
  path.poses.push_back(last);
  return path;
}

double heading(const nav_msgs::msg::Path & p, std::size_t i)
{
  return std::atan2(
    p.poses[i + 1].pose.position.y - p.poses[i].pose.position.y,
    p.poses[i + 1].pose.position.x - p.poses[i].pose.position.x);
}

double angleDiff(double a, double b)
{
  return std::abs(std::remainder(a - b, 2.0 * M_PI));
}

}  // namespace

TEST(ShiftPath, StraightLineFullThenTaper)
{
  const auto path = makePath({{0.0, 0.0}, {2.0, 0.0}});  // 41점
  const auto out = pode_follow_wall::shiftPath(path, 8, 10, 0.0, 0.02, 0.30, 0.20);
  ASSERT_EQ(out.poses.size(), path.poses.size() - 8);
  for (std::size_t k = 0; k < out.poses.size(); ++k) {
    const std::size_t i = k + 8;
    const double dy = out.poses[k].pose.position.y - path.poses[i].pose.position.y;
    EXPECT_DOUBLE_EQ(out.poses[k].pose.position.x, path.poses[i].pose.position.x);
    const double s = std::max(0.0, (static_cast<double>(i) - 10.0) * 0.05);
    if (s <= 0.30 + 1e-9) {
      EXPECT_NEAR(dy, 0.02, 1e-9) << i;
    } else if (s >= 0.50 - 1e-9) {
      EXPECT_NEAR(dy, 0.0, 1e-9) << i;
    } else {
      EXPECT_GT(dy, 0.0);
      EXPECT_LT(dy, 0.02);
    }
  }
  // 줄어드는 동안 기울기는 상한/taper보다 크지 않다
  for (std::size_t k = 0; k + 1 < out.poses.size(); ++k) {
    EXPECT_LE(angleDiff(heading(out, k), 0.0), std::atan(0.02 / 0.20) + 1e-9);
  }
}

TEST(ShiftPath, EndOfLineKeepsShape)
{
  // 줄 끝 가까이: 남은 점이 모두 같이 옮겨져 모양과 간격이 그대로다
  const auto path = makePath({{0.0, 0.0}, {1.0, 0.0}});
  const std::size_t nearest = path.poses.size() - 4;
  const auto out = pode_follow_wall::shiftPath(path, nearest - 2, nearest, 0.0, -0.02, 0.30, 0.20);
  ASSERT_EQ(out.poses.size(), 6u);
  for (std::size_t k = 0; k < out.poses.size(); ++k) {
    const auto & a = out.poses[k].pose.position;
    const auto & b = path.poses[nearest - 2 + k].pose.position;
    EXPECT_NEAR(a.x, b.x, 1e-12);
    EXPECT_NEAR(a.y - b.y, -0.02, 1e-12);
  }
  // 끝 방향(자세)은 바꾸지 않는다
  EXPECT_DOUBLE_EQ(out.poses.back().pose.orientation.w, path.poses.back().pose.orientation.w);
}

TEST(ShiftPath, CornerStaysSmooth)
{
  // 동쪽 0.4 m 뒤 북쪽으로 꺾인다, 왼쪽(북쪽) 벽 쪽으로 2 cm
  const auto path = makePath({{0.0, 0.0}, {0.4, 0.0}, {0.4, 0.6}});
  const auto out = pode_follow_wall::shiftPath(path, 0, 2, 0.0, 0.02, 0.30, 0.20);
  ASSERT_EQ(out.poses.size(), path.poses.size());
  for (std::size_t k = 0; k < out.poses.size(); ++k) {
    const auto & a = out.poses[k].pose.position;
    const auto & b = path.poses[k].pose.position;
    EXPECT_LE(std::hypot(a.x - b.x, a.y - b.y), 0.02 + 1e-12);
  }
  for (std::size_t k = 0; k + 1 < out.poses.size(); ++k) {
    EXPECT_LE(angleDiff(heading(out, k), heading(path, k)), std::atan(0.02 / 0.05) + 1e-9) << k;
    // 점이 거꾸로 가지 않는다
    const double dx = out.poses[k + 1].pose.position.x - out.poses[k].pose.position.x;
    const double dy = out.poses[k + 1].pose.position.y - out.poses[k].pose.position.y;
    const double ox = path.poses[k + 1].pose.position.x - path.poses[k].pose.position.x;
    const double oy = path.poses[k + 1].pose.position.y - path.poses[k].pose.position.y;
    EXPECT_GT(dx * ox + dy * oy, 0.0) << k;
  }
}

TEST(NearestIndex, MonotonicAndBounded)
{
  // 갔다가 돌아오는 경로: 이미 지난 앞쪽 점으로 되돌아가지 않는다
  const auto path = makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.1}, {0.0, 0.1}});
  const std::size_t back = path.poses.size() - 3;  // 돌아오는 줄의 x = 0.1 근처
  EXPECT_EQ(pode_follow_wall::nearestIndex(path, 0.1, 0.1, back - 2, 1.0), back);
  // 처음부터 찾으면 가는 줄의 점을 고른다
  EXPECT_EQ(pode_follow_wall::nearestIndex(path, 0.1, 0.02, 0, 1.0), 2u);
  // 찾는 길이 밖은 보지 않는다
  EXPECT_EQ(pode_follow_wall::nearestIndex(path, 0.9, 0.0, 0, 0.3), 6u);
}

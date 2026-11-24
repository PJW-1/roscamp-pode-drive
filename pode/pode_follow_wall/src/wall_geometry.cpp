#include "pode_follow_wall/wall_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace pode_follow_wall
{

namespace
{

// 쿼터니언의 yaw
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 인라이어에 최소제곱(주성분) 직선을 맞춘다
LineFit leastSquares(const std::vector<Point2> & pts)
{
  LineFit fit;
  const double n = static_cast<double>(pts.size());
  for (const auto & p : pts) {
    fit.cx += p.x;
    fit.cy += p.y;
  }
  fit.cx /= n;
  fit.cy /= n;
  double sxx = 0.0, syy = 0.0, sxy = 0.0;
  for (const auto & p : pts) {
    const double ux = p.x - fit.cx;
    const double uy = p.y - fit.cy;
    sxx += ux * ux;
    syy += uy * uy;
    sxy += ux * uy;
  }
  const double angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  fit.dx = std::cos(angle);
  fit.dy = std::sin(angle);
  fit.nx = -fit.dy;
  fit.ny = fit.dx;
  double d = fit.nx * fit.cx + fit.ny * fit.cy;
  if (d < 0.0) {
    fit.nx = -fit.nx;
    fit.ny = -fit.ny;
    d = -d;
  }
  fit.dist = d;
  double sq = 0.0;
  double tmin = std::numeric_limits<double>::max();
  double tmax = std::numeric_limits<double>::lowest();
  for (const auto & p : pts) {
    const double r = fit.nx * p.x + fit.ny * p.y - d;
    sq += r * r;
    const double t = fit.dx * (p.x - fit.cx) + fit.dy * (p.y - fit.cy);
    tmin = std::min(tmin, t);
    tmax = std::max(tmax, t);
  }
  fit.rms = std::sqrt(sq / n);
  fit.inliers = static_cast<int>(pts.size());
  fit.start = {fit.cx + tmin * fit.dx, fit.cy + tmin * fit.dy};
  fit.end = {fit.cx + tmax * fit.dx, fit.cy + tmax * fit.dy};
  return fit;
}

}  // namespace

// ---- 직선 맞추기 ----

LineFit fitLine(const std::vector<Point2> & points, const FitParams & params)
{
  LineFit none;
  const int n = static_cast<int>(points.size());
  if (n < std::max(2, params.min_points)) {
    none.inliers = n;
    return none;
  }
  // RANSAC: 두 점 직선 가운데 인라이어가 가장 많은 것
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> pick(0, n - 1);
  std::vector<Point2> best;
  for (int it = 0; it < params.ransac_iterations; ++it) {
    const int i = pick(rng);
    int j = pick(rng);
    if (i == j) {
      j = (j + 1) % n;
    }
    const double ux = points[j].x - points[i].x;
    const double uy = points[j].y - points[i].y;
    const double len = std::hypot(ux, uy);
    if (len < 1e-6) {
      continue;
    }
    const double nx = -uy / len;
    const double ny = ux / len;
    std::vector<Point2> in;
    for (const auto & p : points) {
      if (std::abs(nx * (p.x - points[i].x) + ny * (p.y - points[i].y)) <= params.ransac_threshold) {
        in.push_back(p);
      }
    }
    if (in.size() > best.size()) {
      best.swap(in);
    }
  }
  if (static_cast<int>(best.size()) < std::max(2, params.min_points)) {
    none.inliers = static_cast<int>(best.size());
    return none;
  }
  LineFit fit = leastSquares(best);
  fit.ok = fit.rms <= params.max_rms;
  return fit;
}

// ---- 창 고르기 ----

std::vector<Point2> selectWindow(
  const std::vector<Point2> & points, int side, const WindowParams & params)
{
  std::vector<Point2> out;
  for (const auto & p : points) {
    const double r = std::hypot(p.x, p.y);
    if (!(r > 0.0) || r > params.range) {
      continue;
    }
    const double a = std::atan2(p.y, p.x) * static_cast<double>(side);
    if (a >= params.min_angle && a <= params.max_angle) {
      out.push_back(p);
    }
  }
  return out;
}

// ---- 지도 광선 ----

double raycast(
  const nav_msgs::msg::OccupancyGrid & grid, double x0, double y0, double theta,
  double max_range, int occupied_threshold)
{
  const double res = grid.info.resolution;
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (res <= 0.0 || w == 0 || h == 0 || grid.data.size() < static_cast<std::size_t>(w * h)) {
    return -1.0;
  }
  // 지도 칸 좌표로 바꾼다
  const double oyaw = yawOf(grid.info.origin.orientation);
  const double rx = x0 - grid.info.origin.position.x;
  const double ry = y0 - grid.info.origin.position.y;
  const double gx = (std::cos(oyaw) * rx + std::sin(oyaw) * ry) / res;
  const double gy = (-std::sin(oyaw) * rx + std::cos(oyaw) * ry) / res;
  const double dx = std::cos(theta - oyaw);
  const double dy = std::sin(theta - oyaw);
  int ix = static_cast<int>(std::floor(gx));
  int iy = static_cast<int>(std::floor(gy));
  auto occupied = [&](int cx, int cy) {
      return grid.data[static_cast<std::size_t>(cy) * w + cx] >= occupied_threshold;
    };
  if (ix < 0 || iy < 0 || ix >= w || iy >= h) {
    return -1.0;
  }
  if (occupied(ix, iy)) {
    return 0.0;
  }
  // 칸 경계를 하나씩 넘는다(DDA)
  const double inf = std::numeric_limits<double>::infinity();
  const int sx = dx > 0.0 ? 1 : -1;
  const int sy = dy > 0.0 ? 1 : -1;
  double tmax_x = std::abs(dx) < 1e-12 ? inf : ((ix + (dx > 0.0 ? 1 : 0)) - gx) / dx;
  double tmax_y = std::abs(dy) < 1e-12 ? inf : ((iy + (dy > 0.0 ? 1 : 0)) - gy) / dy;
  const double tdx = std::abs(dx) < 1e-12 ? inf : 1.0 / std::abs(dx);
  const double tdy = std::abs(dy) < 1e-12 ? inf : 1.0 / std::abs(dy);
  const double tlimit = max_range / res;
  while (true) {
    double t;
    if (tmax_x < tmax_y) {
      t = tmax_x;
      tmax_x += tdx;
      ix += sx;
    } else {
      t = tmax_y;
      tmax_y += tdy;
      iy += sy;
    }
    if (t > tlimit || ix < 0 || iy < 0 || ix >= w || iy >= h) {
      return -1.0;
    }
    if (occupied(ix, iy)) {
      return t * res;
    }
  }
}

std::vector<Point2> expectedPoints(
  const nav_msgs::msg::OccupancyGrid & grid, const Pose2 & pose,
  const std::vector<Point2> & directions, double max_range, int occupied_threshold)
{
  std::vector<Point2> out;
  for (const auto & p : directions) {
    const double a = std::atan2(p.y, p.x);
    const double d = raycast(grid, pose.x, pose.y, pose.yaw + a, max_range, occupied_threshold);
    if (d > 0.0) {
      out.push_back({d * std::cos(a), d * std::sin(a)});
    }
  }
  return out;
}

// ---- 벽 판정 ----

WallMeasurement measureSide(
  const std::vector<Point2> & scan_points, const nav_msgs::msg::OccupancyGrid & grid,
  const Pose2 & pose, int side, const WallParams & params)
{
  WallMeasurement m;
  m.side = side;
  const auto pts = selectWindow(scan_points, side, params.window);
  m.measured = fitLine(pts, params.fit);
  if (!m.measured.ok) {
    return m;
  }
  // 같은 추정 자세에서 같은 방향으로 지도를 본다
  const auto exp_pts =
    expectedPoints(grid, pose, pts, params.window.range + params.max_error, params.occupied_threshold);
  m.expected = fitLine(exp_pts, params.fit);
  if (!m.expected.ok) {
    return m;
  }
  const double dot = m.measured.dx * m.expected.dx + m.measured.dy * m.expected.dy;
  m.angle_diff = std::acos(std::min(1.0, std::abs(dot)));
  m.error = m.measured.dist - m.expected.dist;
  m.valid = m.angle_diff <= params.max_angle_diff && std::abs(m.error) <= params.max_error;
  return m;
}

WallMeasurement measureWall(
  const std::vector<Point2> & scan_points, const nav_msgs::msg::OccupancyGrid & grid,
  const Pose2 & pose, const WallParams & params)
{
  const auto left = measureSide(scan_points, grid, pose, 1, params);
  const auto right = measureSide(scan_points, grid, pose, -1, params);
  if (left.valid && right.valid) {
    return left.measured.dist <= right.measured.dist ? left : right;
  }
  if (left.valid) {
    return left;
  }
  if (right.valid) {
    return right;
  }
  WallMeasurement none;
  return none;
}

// ---- 보정량 ----

double shapeCorrection(double filtered_error, double deadband, double max_correction)
{
  if (std::abs(filtered_error) < deadband) {
    return 0.0;
  }
  return std::clamp(filtered_error, -max_correction, max_correction);
}

// ---- 경로 ----

std::size_t nearestIndex(
  const nav_msgs::msg::Path & path, double x, double y, std::size_t from, double search_length)
{
  const auto & ps = path.poses;
  if (ps.empty()) {
    return 0;
  }
  from = std::min(from, ps.size() - 1);
  std::size_t best = from;
  double best_d = std::numeric_limits<double>::max();
  double acc = 0.0;
  for (std::size_t i = from; i < ps.size(); ++i) {
    if (i > from) {
      acc += std::hypot(
        ps[i].pose.position.x - ps[i - 1].pose.position.x,
        ps[i].pose.position.y - ps[i - 1].pose.position.y);
      if (acc > search_length) {
        break;
      }
    }
    const double d = std::hypot(ps[i].pose.position.x - x, ps[i].pose.position.y - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

nav_msgs::msg::Path shiftPath(
  const nav_msgs::msg::Path & path, std::size_t start, std::size_t nearest,
  double offset_x, double offset_y, double full_length, double taper_length)
{
  nav_msgs::msg::Path out;
  out.header = path.header;
  const auto & ps = path.poses;
  if (ps.empty()) {
    return out;
  }
  start = std::min(start, ps.size() - 1);
  double s = 0.0;
  for (std::size_t i = start; i < ps.size(); ++i) {
    if (i > nearest) {
      s += std::hypot(
        ps[i].pose.position.x - ps[i - 1].pose.position.x,
        ps[i].pose.position.y - ps[i - 1].pose.position.y);
    }
    double wgt = 1.0;
    if (s > full_length) {
      wgt = taper_length > 0.0 ? std::max(0.0, 1.0 - (s - full_length) / taper_length) : 0.0;
    }
    auto p = ps[i];
    p.pose.position.x += wgt * offset_x;
    p.pose.position.y += wgt * offset_y;
    out.poses.push_back(p);
  }
  return out;
}

}  // namespace pode_follow_wall

#include "pode_rl_controller/closing_guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace pode_rl_controller
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr float kRayCap = 100.0f;

void require(bool ok, const std::string & what, double value)
{
  if (!ok) {
    throw std::runtime_error(
      "FollowPathRL: " + what + ", got " + std::to_string(value));
  }
}

// 돌아오지 않은 레이(nan, +-inf)는 먼 것으로 바꾸고 100 m로 자른다
std::vector<float> capped(const std::vector<float> & rays)
{
  std::vector<float> out(rays.size());
  for (size_t i = 0; i < rays.size(); ++i) {
    out[i] = std::isfinite(rays[i]) ? std::min(rays[i], kRayCap) : kRayCap;
  }
  return out;
}
}  // namespace

// ---- 인자 검사 ----
void check_closing_params(const ClosingParams & p)
{
  require(std::isfinite(p.stop_range), "closing_stop_range must be finite (0 or less = off)",
    p.stop_range);
  require(
    std::isfinite(p.half_width_deg) && p.half_width_deg >= 0.0 && p.half_width_deg <= 180.0,
    "closing_half_width_deg must be in [0, 180]", p.half_width_deg);
  require(std::isfinite(p.lookback_s) && p.lookback_s > 0.0, "closing_lookback_s must be > 0",
    p.lookback_s);
  require(
    std::isfinite(p.spread_deg) && p.spread_deg >= 0.0 && p.spread_deg <= 180.0,
    "closing_spread_deg must be in [0, 180]", p.spread_deg);
  require(std::isfinite(p.closing_mps) && p.closing_mps > 0.0, "closing_mps must be > 0",
    p.closing_mps);
  require(std::isfinite(p.hold_s) && p.hold_s >= 0.0, "closing_hold_s must be >= 0", p.hold_s);
  require(p.min_rays >= 1, "closing_min_rays must be >= 1", p.min_rays);
}

// ---- 옛 스캔을 지금 자세로 옮기기 ----
std::vector<double> static_ranges(
  const std::vector<float> & rays, double x, double y, double yaw, double x_now, double y_now,
  double yaw_now, int spread_bins)
{
  const long n = static_cast<long>(rays.size());
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> nearest(rays.size(), inf);
  for (long i = 0; i < n; ++i) {
    const double world = yaw + (-180.0 + i * 360.0 / n) * (kPi / 180.0);
    const double px = x + rays[i] * std::cos(world) - x_now;
    const double py = y + rays[i] * std::sin(world) - y_now;
    const double bearing = std::atan2(py, px) - yaw_now;
    long bin = static_cast<long>(std::nearbyint((bearing + kPi) * (n / kTwoPi))) % n;
    if (bin < 0) {
      bin += n;
    }
    nearest[bin] = std::min(nearest[bin], std::hypot(px, py));
  }
  if (spread_bins <= 0) {
    return nearest;
  }
  // 이웃 +-spread_bins 칸의 최솟값, 이웃은 한 바퀴를 넘어 이어진다
  std::vector<double> out(rays.size(), inf);
  for (long i = 0; i < n; ++i) {
    for (long d = -spread_bins; d <= spread_bins; ++d) {
      out[i] = std::min(out[i], nearest[((i + d) % n + n) % n]);
    }
  }
  return out;
}

// ---- 다가옴 막기 ----
ClosingGuard::ClosingGuard(const ClosingParams & p)
: p_(p)
{
  check_closing_params(p_);
}

bool ClosingGuard::enabled() const
{
  return p_.stop_range > 0.0;
}

const ClosingParams & ClosingGuard::params() const
{
  return p_;
}

bool ClosingGuard::add_scan(
  const std::vector<float> & rays, double x, double y, double yaw, double stamp, double now)
{
  if (!enabled()) {
    return false;
  }
  // 같은 스캔이 다시 왔으면 넣지 않고, 시계가 되돌아갔으면 이력과 유지를 비운다
  if (!history_.empty() && !(stamp > history_.back().stamp)) {
    if (stamp < history_.back().stamp) {
      reset();
    }
    return false;
  }
  Frame cur{capped(rays), x, y, yaw, stamp};
  // 기준은 lookback_s 이상 지난 스캔 가운데 가장 새 것이고, 그보다 오래된 스캔은 버린다
  size_t ref = history_.size();
  for (size_t k = history_.size(); k-- > 0; ) {
    if (stamp - history_[k].stamp >= p_.lookback_s - 1e-6) {
      ref = k;
      break;
    }
  }
  bool hit = false;
  if (ref < history_.size()) {
    history_.erase(history_.begin(), history_.begin() + static_cast<long>(ref));
    const double age = stamp - history_.front().stamp;
    // 기준이 너무 오래되었으면(스캔이 끊겼던 동안) 이번에는 판정하지 않는다
    const double max_age = p_.lookback_s + std::max(0.5 * p_.lookback_s, 0.15);
    hit = age <= max_age && judge(history_.front(), cur);
  }
  history_.push_back(std::move(cur));
  if (hit) {
    have_hit_ = true;
    last_hit_ = now;
  }
  return hit;
}

// 정면 부채꼴 안에서 stop_range보다 가깝고 closing_mps보다 빨리 다가온 레이가 나란히 min_rays개
bool ClosingGuard::judge(const Frame & ref, const Frame & cur) const
{
  const size_t n = cur.rays.size();
  if (n == 0 || ref.rays.size() != n) {
    return false;
  }
  const int spread_bins = static_cast<int>(std::nearbyint(p_.spread_deg * n / 360.0));
  const auto still = static_ranges(
    ref.rays, ref.x, ref.y, ref.yaw, cur.x, cur.y, cur.yaw, spread_bins);
  const double dt = cur.stamp - ref.stamp;
  // 거리 문턱은 Python(torch float32)과 같게 float로 견준다
  const float stop = static_cast<float>(p_.stop_range);
  int run = 0;
  for (size_t i = 0; i < n; ++i) {
    const double deg = -180.0 + i * 360.0 / n;
    const double closing = (still[i] - cur.rays[i]) / dt;
    const bool near = std::abs(deg) <= p_.half_width_deg + 1e-4 && cur.rays[i] < stop &&
      std::isfinite(closing) && closing > p_.closing_mps;
    run = near ? run + 1 : 0;
    if (run >= p_.min_rays) {
      return true;
    }
  }
  return false;
}

bool ClosingGuard::holding(double now) const
{
  return have_hit_ && now - last_hit_ <= p_.hold_s + 1e-6;
}

// 걸려 있는 동안 전진 목표 v만 0으로 하고 w와 후진 목표는 그대로 둔다
Twist2 ClosingGuard::apply(const Twist2 & target, double now) const
{
  if (target.v > 0.0 && holding(now)) {
    return {0.0, target.w};
  }
  return target;
}

void ClosingGuard::reset()
{
  history_.clear();
  have_hit_ = false;
  last_hit_ = 0.0;
}

size_t ClosingGuard::history_size() const
{
  return history_.size();
}

}  // namespace pode_rl_controller

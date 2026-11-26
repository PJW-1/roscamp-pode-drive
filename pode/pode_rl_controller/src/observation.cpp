#include "pode_rl_controller/observation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace pode_rl_controller
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
// 가림 경계에 놓인 빔은 반올림과 상관없이 가린다(task_spec.rays_within과 같은 허용 오차, 도)
constexpr double kMaskEdgeTolDeg = 1e-6;

// 레이 k가 고르는 빔(가장 가까운 빔). scan이 그 각을 덮지 않으면 -1
long ray_beam(const Scan & scan, double lidar_yaw, const LidarSpec & lid, int k)
{
  const long n = static_cast<long>(scan.ranges.size());
  const double theta = (lid.first_ray_deg + k * lid.ray_step_deg) * (kPi / 180.0);
  double rel = std::fmod(theta - lidar_yaw - scan.angle_min, kTwoPi);
  if (rel < 0.0) {
    rel += kTwoPi;
  }
  long i = static_cast<long>(std::floor(rel / scan.angle_increment + 0.5));
  if (i >= n) {
    i = (kTwoPi - rel <= 0.5 * scan.angle_increment) ? 0 : -1;
  }
  return i;
}

// base 기준 각(도)이 가림 범위 안인가
bool angle_masked(double any_deg, const std::vector<AngleRange> & mask)
{
  const double deg = std::remainder(any_deg, 360.0);
  // 바로 뒤는 -180과 180 둘 다로 본다
  const double alt = deg <= -180.0 ? 180.0 : (deg >= 180.0 ? -180.0 : deg);
  for (const auto & m : mask) {
    const double lo = m.lo_deg - kMaskEdgeTolDeg;
    const double hi = m.hi_deg + kMaskEdgeTolDeg;
    if ((deg >= lo && deg <= hi) || (alt >= lo && alt <= hi)) {
      return true;
    }
  }
  return false;
}

// 빔 i가 가림 범위 안인가
bool beam_masked(
  const Scan & scan, size_t i, double lidar_yaw, const std::vector<AngleRange> & mask)
{
  const double rad = lidar_yaw + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
  return angle_masked(rad * (180.0 / kPi), mask);
}
}  // namespace

// ---- 경로 따라가기 ----
size_t follow_progress(
  const std::vector<Point2> & path, size_t prog, const Point2 & robot, int window)
{
  const size_t n = path.size();
  prog = std::min(prog, n - 1);
  const size_t last = std::min(n - 1, prog + static_cast<size_t>(window));
  size_t near = prog;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = prog; i <= last; ++i) {
    const double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
    if (d < best) {
      best = d;
      near = i;
    }
  }
  return near;
}

size_t lookahead_index(const std::vector<Point2> & path, size_t prog, double lookahead)
{
  std::vector<double> cum(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cum[i] = cum[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  const double target = cum[prog] + lookahead;
  const auto it = std::lower_bound(cum.begin(), cum.end(), target);
  return std::min(static_cast<size_t>(it - cum.begin()), path.size() - 1);
}

GoalPolar goal_in_robot_frame(const Point2 & robot, double yaw, const Point2 & goal)
{
  const double dx = goal.x - robot.x;
  const double dy = goal.y - robot.y;
  double ang = std::atan2(dy, dx) - yaw;
  ang = std::atan2(std::sin(ang), std::cos(ang));
  return {std::hypot(dx, dy), ang};
}

GoalPolar path_goal(
  const std::vector<Point2> & path, size_t & prog, const Point2 & robot, double yaw,
  const GoalRule & rule)
{
  if (rule.rule != "lookahead") {
    return goal_in_robot_frame(robot, yaw, path.back());
  }
  prog = follow_progress(path, prog, robot, rule.follow_window);
  return goal_in_robot_frame(robot, yaw, path[lookahead_index(path, prog, rule.lookahead_m)]);
}

// ---- 스캔 -> 학습 광선 360개 ----
std::vector<float> scan_to_rays(const Scan & scan, double lidar_yaw, const PolicySpec & spec)
{
  const auto & lid = spec.lidar;
  std::vector<float> rays(lid.rays);
  for (int k = 0; k < lid.rays; ++k) {
    const long i = ray_beam(scan, lidar_yaw, lid, k);
    double r = i >= 0 ? scan.ranges[i] : std::numeric_limits<double>::infinity();
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      r = lid.obs_max;
    }
    rays[k] = static_cast<float>(std::min(r, lid.obs_max));
  }
  return rays;
}

// ---- 관측 벡터 ----
std::vector<float> build_observation(
  const std::vector<float> & rays, double goal_dist, double goal_angle, double v, double w,
  const PolicySpec & spec)
{
  const auto & lid = spec.lidar;
  const int per_sector = lid.rays / lid.sectors;
  std::vector<float> obs;
  obs.reserve(spec.obs_dim);
  for (int s = 0; s < lid.sectors; ++s) {
    float m = static_cast<float>(lid.obs_max);
    for (int j = 0; j < per_sector; ++j) {
      const float r = rays[s * per_sector + j];
      m = std::min(m, std::isnan(r) ? static_cast<float>(lid.obs_max) : r);
    }
    obs.push_back(m / static_cast<float>(lid.obs_max));
  }
  obs.push_back(static_cast<float>(std::min(goal_dist, spec.goal_obs_max) / spec.goal_obs_max));
  obs.push_back(static_cast<float>(std::sin(goal_angle)));
  obs.push_back(static_cast<float>(std::cos(goal_angle)));
  obs.push_back(static_cast<float>(v / spec.action.v_max));
  obs.push_back(static_cast<float>(w / spec.action.w_max));
  return obs;
}

// ---- 행동 -> (v, w), 가속 제한 ----
Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim)
{
  return action_to_target(a_v, a_w, lim, lim.v_max);
}

Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim, double v_top)
{
  const double av = std::clamp(static_cast<double>(a_v), -1.0, 1.0);
  const double aw = std::clamp(static_cast<double>(a_w), -1.0, 1.0);
  return {lim.v_min + (av + 1.0) * 0.5 * (v_top - lim.v_min), aw * lim.w_max};
}

Twist2 block_reverse(
  const Twist2 & target, const std::vector<float> & rays, double stop_range, double half_width_deg,
  const PolicySpec & spec)
{
  if (stop_range <= 0.0 || target.v >= 0.0) {
    return target;
  }
  for (size_t k = 0; k < rays.size(); ++k) {
    const double deg = spec.lidar.first_ray_deg + k * spec.lidar.ray_step_deg;
    const double off = std::abs(std::remainder(deg, 360.0));
    if (off >= 180.0 - half_width_deg - 1e-9 && rays[k] < stop_range) {
      return {0.0, target.w};
    }
  }
  return target;
}

Twist2 rate_limit(const Twist2 & cmd, const Twist2 & target, double dt, const ActionSpec & lim)
{
  const double sv = lim.acc_v * dt;
  const double sw = lim.acc_w * dt;
  return {cmd.v + std::clamp(target.v - cmd.v, -sv, sv),
    cmd.w + std::clamp(target.w - cmd.w, -sw, sw)};
}

// ---- format 3 관측 블록과 에피소드 기억 ----
ObsMemory::ObsMemory(const PolicySpec & spec)
: spec_(spec)
{
  if (spec_.has_block("lidar_history")) {
    depth_ = static_cast<size_t>(
      (spec_.lidar_history.frames - 1) * spec_.lidar_history.stride_steps);
  }
}

void ObsMemory::reset()
{
  prev_ = {0.0, 0.0};
  frames_.clear();
}

std::vector<float> ObsMemory::blocks(const std::vector<float> & sectors, double speed_cap) const
{
  std::vector<float> out;
  for (const auto & block : spec_.obs_blocks) {
    if (block == "prev_action") {
      out.push_back(static_cast<float>(prev_.v / spec_.prev_action.v_scale));
      out.push_back(static_cast<float>(prev_.w / spec_.prev_action.w_scale));
    } else if (block == "lidar_history") {
      const size_t stride = static_cast<size_t>(spec_.lidar_history.stride_steps);
      for (int f = 1; f < spec_.lidar_history.frames; ++f) {
        const size_t k = static_cast<size_t>(f) * stride - 1;
        const auto & old = frames_.empty() ? sectors :
          (k < frames_.size() ? frames_[k] : frames_.back());
        out.insert(out.end(), old.begin(), old.end());
      }
    } else if (block == "speed_cap") {
      out.push_back(static_cast<float>(speed_cap / spec_.speed_cap.obs_scale));
    }
  }
  return out;
}

void ObsMemory::step(const std::vector<float> & sectors, const Twist2 & policy_target)
{
  prev_ = policy_target;
  if (depth_ == 0) {
    return;
  }
  frames_.push_front(sectors);
  while (frames_.size() > depth_) {
    frames_.pop_back();
  }
}

bool memory_stale(bool policy_ran_before, double since_last_policy_s, double control_dt)
{
  return !policy_ran_before || since_last_policy_s > 2.0 * control_dt;
}

bool new_episode(bool had_plan, const Point2 & old_end, const Point2 & new_end)
{
  return !had_plan || std::hypot(new_end.x - old_end.x, new_end.y - old_end.y) > 1e-3;
}

// ---- 출력 평활 ----
double check_ema_alpha(double alpha, const std::string & name)
{
  if (!(alpha > 0.0 && alpha <= 1.0)) {
    throw std::runtime_error(
      "FollowPathRL: " + name + " must be in (0, 1] (1 = off), got " + std::to_string(alpha));
  }
  return alpha;
}

TargetSmoother::TargetSmoother(double alpha_v, double alpha_w)
: alpha_v_(check_ema_alpha(alpha_v, "ema_alpha_v")),
  alpha_w_(check_ema_alpha(alpha_w, "ema_alpha_w"))
{
}

void TargetSmoother::reset()
{
  primed_ = false;
  out_ = {0.0, 0.0};
}

// 리셋 뒤 첫 주기는 start(지금 내는 명령)에서 출발한다. alpha 1인 성분은 목표를 그대로 낸다
Twist2 TargetSmoother::step(const Twist2 & target, const Twist2 & start)
{
  if (!primed_) {
    out_ = start;
    primed_ = true;
  }
  out_.v = alpha_v_ == 1.0 ? target.v : alpha_v_ * target.v + (1.0 - alpha_v_) * out_.v;
  out_.w = alpha_w_ == 1.0 ? target.w : alpha_w_ * target.w + (1.0 - alpha_w_) * out_.w;
  return out_;
}

// ---- 스캔 가리기 ----
std::vector<AngleRange> parse_scan_mask(const std::vector<double> & deg)
{
  if (deg.size() % 2 != 0) {
    throw std::runtime_error(
      "FollowPathRL: scan_mask_deg must list pairs [lo, hi, lo, hi, ...] in degrees, got " +
      std::to_string(deg.size()) + " numbers");
  }
  std::vector<AngleRange> out;
  for (size_t i = 0; i < deg.size(); i += 2) {
    const double lo = deg[i];
    const double hi = deg[i + 1];
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo < -180.0 || hi > 180.0 || lo >= hi) {
      throw std::runtime_error(
        "FollowPathRL: scan_mask_deg range " + std::to_string(i / 2) + " [" + std::to_string(lo) +
        ", " + std::to_string(hi) + "] must satisfy -180 <= lo < hi <= 180 (split a range "
        "across 180 into two)");
    }
    out.push_back({lo, hi});
  }
  return out;
}

void mask_scan(Scan & scan, double lidar_yaw, const std::vector<AngleRange> & mask)
{
  if (mask.empty()) {
    return;
  }
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (beam_masked(scan, i, lidar_yaw, mask)) {
      scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
  }
}

// ---- 가린 섹터 채우기 ----
void check_scan_mask_fill(bool fill, const std::vector<AngleRange> & mask)
{
  if (fill && mask.empty()) {
    throw std::runtime_error(
      "FollowPathRL: scan_mask_fill needs scan_mask_deg (with no masked beams there is no sector "
      "to fill)");
  }
}

// 채우기 없이 쓰면 정책이 반환 없음으로 읽게 되는 섹터를 configure에서 알리는 데 쓴다
std::vector<int> fully_masked_sectors(
  const std::vector<AngleRange> & mask, const PolicySpec & spec)
{
  const auto & lid = spec.lidar;
  const int per_sector = lid.rays / lid.sectors;
  std::vector<int> out;
  for (int s = 0; s < lid.sectors && !mask.empty(); ++s) {
    bool all = true;
    for (int k = s * per_sector; k < (s + 1) * per_sector && all; ++k) {
      all = angle_masked(lid.first_ray_deg + k * lid.ray_step_deg, mask);
    }
    if (all) {
      out.push_back(s);
    }
  }
  return out;
}

// 가린 레이는 고른 빔이 가려진 레이, 쓸 수 있는 레이는 고른 빔이 가리지 않았고 유효한 반환인 레이다.
// scan은 mask_scan 전이든 뒤든 같은 답을 낸다
MaskedSectors masked_sectors(
  const Scan & scan, double lidar_yaw, const std::vector<AngleRange> & mask,
  const PolicySpec & spec)
{
  const auto & lid = spec.lidar;
  const int per_sector = lid.rays / lid.sectors;
  MaskedSectors out{std::vector<bool>(lid.sectors, false), std::vector<bool>(lid.sectors, false)};
  for (int k = 0; k < lid.sectors * per_sector; ++k) {
    const long i = ray_beam(scan, lidar_yaw, lid, k);
    if (i < 0) {
      continue;
    }
    const int s = k / per_sector;
    if (beam_masked(scan, static_cast<size_t>(i), lidar_yaw, mask)) {
      out.touched[s] = true;
      continue;
    }
    const double r = scan.ranges[i];
    if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
      out.usable[s] = true;
    }
  }
  return out;
}

// 가림에 걸렸고 쓸 수 있는 레이가 없는 섹터(empty)를 양쪽으로 kFillReach 섹터 안의 가장 가까운
// empty 아닌 섹터 값 가운데 작은 쪽으로 바꾼다. 양쪽 다 없으면 limit이다. 섹터 번호는 한 바퀴를
// 넘어 이어지고, 후보는 채우기 전의 값에서 읽는다. values의 앞 섹터 수만큼의 칸만 본다
void fill_masked_sectors(std::vector<float> & values, const MaskedSectors & flags, float limit)
{
  const int n = static_cast<int>(flags.touched.size());
  auto empty = [&](int s) {return flags.touched[s] && !flags.usable[s];};
  const std::vector<float> before(values.begin(), values.begin() + n);
  for (int s = 0; s < n; ++s) {
    if (!empty(s)) {
      continue;
    }
    float best = limit;
    for (const int step : {-1, 1}) {
      for (int d = 1; d <= kFillReach; ++d) {
        const int j = ((s + step * d) % n + n) % n;
        if (!empty(j)) {
          best = std::min(best, before[j]);
          break;
        }
      }
    }
    values[s] = best;
  }
}

}  // namespace pode_rl_controller

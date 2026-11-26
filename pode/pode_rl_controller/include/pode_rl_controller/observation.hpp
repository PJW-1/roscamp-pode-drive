#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "pode_rl_controller/policy_spec.hpp"

namespace pode_rl_controller
{

// ---- 관측과 행동(task_spec.py와 같은 정의) ----
struct Scan
{
  double angle_min;
  double angle_increment;
  double range_min;
  double range_max;
  std::vector<float> ranges;
};

struct Twist2
{
  double v;
  double w;
};

struct Point2
{
  double x;
  double y;
};

struct GoalPolar
{
  double dist;
  double angle;
};

// ---- 경로 따라가기(route_spec.follow와 같은 정의) ----
size_t follow_progress(
  const std::vector<Point2> & path, size_t prog, const Point2 & robot, int window);
size_t lookahead_index(const std::vector<Point2> & path, size_t prog, double lookahead);
GoalPolar goal_in_robot_frame(const Point2 & robot, double yaw, const Point2 & goal);
GoalPolar path_goal(
  const std::vector<Point2> & path, size_t & prog, const Point2 & robot, double yaw,
  const GoalRule & rule);

std::vector<float> scan_to_rays(const Scan & scan, double lidar_yaw, const PolicySpec & spec);
std::vector<float> build_observation(
  const std::vector<float> & rays, double goal_dist, double goal_angle, double v, double w,
  const PolicySpec & spec);
Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim);
Twist2 action_to_target(float a_v, float a_w, const ActionSpec & lim, double v_top);
Twist2 block_reverse(
  const Twist2 & target, const std::vector<float> & rays, double stop_range, double half_width_deg,
  const PolicySpec & spec);
Twist2 rate_limit(const Twist2 & cmd, const Twist2 & target, double dt, const ActionSpec & lim);

// ---- format 3 관측 블록(prev_action, lidar_history, speed_cap)과 에피소드 기억 ----
class ObsMemory
{
public:
  explicit ObsMemory(const PolicySpec & spec = PolicySpec{});
  void reset();
  std::vector<float> blocks(const std::vector<float> & sectors, double speed_cap) const;
  void step(const std::vector<float> & sectors, const Twist2 & policy_target);

private:
  PolicySpec spec_;
  size_t depth_{0};
  Twist2 prev_{0.0, 0.0};
  std::deque<std::vector<float>> frames_;
};

bool memory_stale(bool policy_ran_before, double since_last_policy_s, double control_dt);
bool new_episode(bool had_plan, const Point2 & old_end, const Point2 & new_end);

// ---- 출력 평활: 정책 목표 (v, w)의 지수 이동 평균(alpha 1 = 끔) ----
double check_ema_alpha(double alpha, const std::string & name);

class TargetSmoother
{
public:
  explicit TargetSmoother(double alpha_v = 1.0, double alpha_w = 1.0);
  void reset();
  Twist2 step(const Twist2 & target, const Twist2 & start);

private:
  double alpha_v_;
  double alpha_w_;
  bool primed_{false};
  Twist2 out_{0.0, 0.0};
};

// ---- 스캔 가리기: base 기준 각도 범위(도)의 빔을 "돌아온 것 없음"으로 ----
struct AngleRange
{
  double lo_deg;
  double hi_deg;
};

std::vector<AngleRange> parse_scan_mask(const std::vector<double> & deg);
void mask_scan(Scan & scan, double lidar_yaw, const std::vector<AngleRange> & mask);

// ---- 가린 섹터 채우기(scan_mask_fill, task_spec.lidar_mask_fill과 같은 규칙) ----
// 채울 값을 찾아 한쪽으로 보는 섹터 수(task_spec.FILL_REACH)
constexpr int kFillReach = 4;

// 섹터마다: 고른 빔이 가려진 레이가 있는지(touched), 쓸 수 있는 레이가 있는지(usable)
struct MaskedSectors
{
  std::vector<bool> touched;
  std::vector<bool> usable;
};

void check_scan_mask_fill(bool fill, const std::vector<AngleRange> & mask);
// 가림이 레이를 모두 덮는 섹터(레이 각으로 본다. 빔 격자에 따라 경계의 레이 하나는 다를 수 있다)
std::vector<int> fully_masked_sectors(
  const std::vector<AngleRange> & mask, const PolicySpec & spec);
MaskedSectors masked_sectors(
  const Scan & scan, double lidar_yaw, const std::vector<AngleRange> & mask,
  const PolicySpec & spec);
void fill_masked_sectors(std::vector<float> & values, const MaskedSectors & flags, float limit);

}  // namespace pode_rl_controller

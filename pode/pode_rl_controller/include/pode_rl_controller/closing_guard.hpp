#pragma once

#include <cstddef>
#include <deque>
#include <vector>

#include "pode_rl_controller/observation.hpp"

namespace pode_rl_controller
{

// ---- 다가옴 막기(task_spec.ClosingGuard와 같은 판정, 기준 스캔과 유지는 시각으로) ----
// 기본값은 긴 창이다: 걸리는 여유(closing_mps * lookback_s)가 8 cm라 스캔 잡음 0.02 m에 걸리지 않는다
struct ClosingParams
{
  double stop_range{0.0};
  double half_width_deg{60.0};
  double lookback_s{1.0};
  double spread_deg{4.0};
  double closing_mps{0.08};
  double hold_s{0.3};
  int min_rays{2};
};

void check_closing_params(const ClosingParams & p);

// 옛 스캔의 점들이 서 있었다면 지금 레이마다 보일 가장 가까운 거리(task_spec.static_ranges)
std::vector<double> static_ranges(
  const std::vector<float> & rays, double x, double y, double yaw, double x_now, double y_now,
  double yaw_now, int spread_bins);

class ClosingGuard
{
public:
  explicit ClosingGuard(const ClosingParams & p = ClosingParams{});
  bool enabled() const;
  const ClosingParams & params() const;
  // 스캔 하나를 넣고 이번에 걸렸는지를 돌려준다. x, y는 라이다 원점, yaw는 base의 yaw
  bool add_scan(
    const std::vector<float> & rays, double x, double y, double yaw, double stamp, double now);
  bool holding(double now) const;
  Twist2 apply(const Twist2 & target, double now) const;
  void reset();
  size_t history_size() const;

private:
  struct Frame
  {
    std::vector<float> rays;
    double x;
    double y;
    double yaw;
    double stamp;
  };
  bool judge(const Frame & ref, const Frame & cur) const;

  ClosingParams p_;
  std::deque<Frame> history_;
  bool have_hit_{false};
  double last_hit_{0.0};
};

}  // namespace pode_rl_controller

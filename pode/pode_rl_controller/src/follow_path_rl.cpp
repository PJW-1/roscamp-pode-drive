#include "pode_rl_controller/follow_path_rl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pode_rl_controller
{

namespace
{
double us_since(const std::chrono::steady_clock::time_point & t0)
{
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

// ---- 설정 ----
void FollowPathRL::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("FollowPathRL: parent node is gone");
  }
  name_ = name;
  tf_ = tf;
  base_frame_ = costmap_ros->getBaseFrameID();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  using nav2_util::declare_parameter_if_not_declared;
  using rclcpp::ParameterValue;
  declare_parameter_if_not_declared(node, name_ + ".policy", ParameterValue("store_arena_4b"));
  declare_parameter_if_not_declared(node, name_ + ".goal_rule", ParameterValue(""));
  declare_parameter_if_not_declared(node, name_ + ".lookahead_dist", ParameterValue(-1.0));
  declare_parameter_if_not_declared(node, name_ + ".scan_topic", ParameterValue("scan"));
  declare_parameter_if_not_declared(node, name_ + ".scan_timeout", ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".transform_tolerance", ParameterValue(0.2));
  declare_parameter_if_not_declared(node, name_ + ".intra_op_threads", ParameterValue(1));
  declare_parameter_if_not_declared(node, name_ + ".rotate_gain", ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name_ + ".log_period", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".rear_stop_range", ParameterValue(0.0));
  declare_parameter_if_not_declared(node, name_ + ".rear_half_width_deg", ParameterValue(30.0));
  declare_parameter_if_not_declared(node, name_ + ".max_linear_vel", ParameterValue(0.0));
  declare_parameter_if_not_declared(node, name_ + ".ema_alpha_v", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".ema_alpha_w", ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".scan_mask_deg", ParameterValue(std::vector<double>{}));
  const ClosingParams closing_default{};
  declare_parameter_if_not_declared(node, name_ + ".closing_stop_range", ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".closing_half_width_deg", ParameterValue(closing_default.half_width_deg));
  declare_parameter_if_not_declared(
    node, name_ + ".closing_lookback_s", ParameterValue(closing_default.lookback_s));
  declare_parameter_if_not_declared(
    node, name_ + ".closing_spread_deg", ParameterValue(closing_default.spread_deg));
  declare_parameter_if_not_declared(
    node, name_ + ".closing_mps", ParameterValue(closing_default.closing_mps));
  declare_parameter_if_not_declared(
    node, name_ + ".closing_hold_s", ParameterValue(closing_default.hold_s));
  declare_parameter_if_not_declared(
    node, name_ + ".closing_min_rays", ParameterValue(closing_default.min_rays));

  const std::string policy = node->get_parameter(name_ + ".policy").as_string();
  const std::string rule_param = node->get_parameter(name_ + ".goal_rule").as_string();
  const double lookahead_param = node->get_parameter(name_ + ".lookahead_dist").as_double();
  scan_topic_ = node->get_parameter(name_ + ".scan_topic").as_string();
  scan_timeout_ = node->get_parameter(name_ + ".scan_timeout").as_double();
  transform_tolerance_ = node->get_parameter(name_ + ".transform_tolerance").as_double();
  const int threads = node->get_parameter(name_ + ".intra_op_threads").as_int();
  rotate_gain_ = node->get_parameter(name_ + ".rotate_gain").as_double();
  log_period_ = node->get_parameter(name_ + ".log_period").as_double();
  rear_stop_range_ = node->get_parameter(name_ + ".rear_stop_range").as_double();
  rear_half_width_deg_ = node->get_parameter(name_ + ".rear_half_width_deg").as_double();
  const double max_v_param = node->get_parameter(name_ + ".max_linear_vel").as_double();
  const double alpha_v = node->get_parameter(name_ + ".ema_alpha_v").as_double();
  const double alpha_w = node->get_parameter(name_ + ".ema_alpha_w").as_double();
  smoother_ = TargetSmoother(alpha_v, alpha_w);
  scan_mask_ = parse_scan_mask(node->get_parameter(name_ + ".scan_mask_deg").as_double_array());
  ClosingParams cp;
  cp.stop_range = node->get_parameter(name_ + ".closing_stop_range").as_double();
  cp.half_width_deg = node->get_parameter(name_ + ".closing_half_width_deg").as_double();
  cp.lookback_s = node->get_parameter(name_ + ".closing_lookback_s").as_double();
  cp.spread_deg = node->get_parameter(name_ + ".closing_spread_deg").as_double();
  cp.closing_mps = node->get_parameter(name_ + ".closing_mps").as_double();
  cp.hold_s = node->get_parameter(name_ + ".closing_hold_s").as_double();
  cp.min_rays = static_cast<int>(node->get_parameter(name_ + ".closing_min_rays").as_int());
  closing_ = ClosingGuard(cp);
  have_closing_stamp_ = false;
  const std::string dir = policy.rfind('/', 0) == 0 ? policy :
    ament_index_cpp::get_package_share_directory("pode_rl_controller") + "/policies/" + policy;
  spec_ = load_policy_spec(dir + "/policy_spec.json");
  policy_ = std::make_unique<OnnxPolicy>(
    dir + "/policy.onnx", spec_.obs_dim, spec_.act_dim, threads);
  const double nav_speed = load_nav_speed(
    ament_index_cpp::get_package_share_directory("pode_nav_config") + "/params/nav_speed.yaml");
  memory_ = ObsMemory(spec_);
  // 다가옴 막기는 레이 i의 방위를 -180도부터 고른 간격으로 읽는다
  if (closing_.enabled() && (spec_.lidar.first_ray_deg != -180.0 ||
    std::abs(spec_.lidar.rays * spec_.lidar.ray_step_deg - 360.0) > 1e-6))
  {
    throw std::runtime_error(
      "FollowPathRL: closing_stop_range needs the policy's rays to start at -180 deg and cover 360 "
      "deg evenly");
  }
  if (spec_.has_block("speed_cap")) {
    max_v_ = max_v_param > 0.0 ? std::min(max_v_param, nav_speed) : nav_speed;
    const SpeedCapValue cap = speed_cap_value(max_v_, spec_.speed_cap);
    speed_cap_ = cap.value;
    v_top_ = cap.value;
    speed_mismatch_ = cap.error;
    if (!speed_mismatch_.empty()) {
      RCLCPP_ERROR(
        logger_, "FollowPathRL %s: %s. The command stays capped at %.3f m/s and this error "
        "repeats while it drives", name_.c_str(), speed_mismatch_.c_str(), max_v_);
    }
  } else {
    speed_mismatch_ = speed_mismatch(spec_.action.v_max, nav_speed);
    if (!speed_mismatch_.empty()) {
      RCLCPP_ERROR(
        logger_, "FollowPathRL %s: %s. Retrain or export the policy at %.3f m/s; until then the "
        "command is capped at %.3f m/s and this error repeats while it drives", name_.c_str(),
        speed_mismatch_.c_str(), nav_speed, nav_speed);
    }
    const double top = std::min(spec_.action.v_max, nav_speed);
    max_v_ = max_v_param > 0.0 ? std::min(max_v_param, top) : top;
    v_top_ = spec_.action.v_max;
  }
  speed_limit_ = max_v_;

  rule_ = spec_.goal_rule;
  if (rule_.follow_window <= 0) {
    rule_.follow_window = 20;
  }
  if (!rule_param.empty() && rule_param != rule_.rule) {
    if (rule_param != "path_end" && rule_param != "lookahead") {
      throw std::runtime_error("FollowPathRL: goal_rule must be path_end or lookahead");
    }
    RCLCPP_WARN(
      logger_, "FollowPathRL %s: goal_rule %s OVERRIDES the policy spec (%s): the policy was not "
      "trained with this goal", name_.c_str(), rule_param.c_str(), rule_.rule.c_str());
    rule_.rule = rule_param;
  }
  if (lookahead_param >= 0.0 && lookahead_param != rule_.lookahead_m) {
    RCLCPP_WARN(
      logger_, "FollowPathRL %s: lookahead_dist %.2f m OVERRIDES the policy spec (%.2f m): the "
      "policy was not trained with this goal", name_.c_str(), lookahead_param, rule_.lookahead_m);
    rule_.lookahead_m = lookahead_param;
  }
  if (rule_.rule == "lookahead" && rule_.lookahead_m <= 0.0) {
    throw std::runtime_error("FollowPathRL: goal_rule lookahead needs lookahead_dist > 0");
  }

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan_ = msg;
    });
  std::string blocks;
  for (const auto & b : spec_.obs_blocks) {
    blocks += (blocks.empty() ? "" : ", ") + b;
  }
  // 다가옴 막기가 켜져 있을 때만 요약 줄 끝에 그 값을 덧붙인다
  std::string closing_note;
  if (closing_.enabled()) {
    char buf[256];
    std::snprintf(
      buf, sizeof(buf), ", closing stop %.3f m within %.1f deg (lookback %.3f s, spread %.1f deg, "
      "closing %.3f m/s, hold %.3f s, min rays %d, tf in %s)", cp.stop_range, cp.half_width_deg,
      cp.lookback_s, cp.spread_deg, cp.closing_mps, cp.hold_s, cp.min_rays, global_frame_.c_str());
    closing_note = buf;
  }
  RCLCPP_INFO(
    logger_, "FollowPathRL %s: policy %s (obs %d, dt %.3f s, v %.2f..%.2f m/s, w %.2f rad/s), "
    "goal_rule %s (lookahead %.2f m, window %d points), scan %s, ORT CPU threads %d, "
    "max linear vel %.2f m/s, spec format %d, obs blocks [%s], speed_cap %.3f m/s, "
    "ema alpha v %.3f w %.3f, scan mask %zu ranges%s",
    name_.c_str(), dir.c_str(), spec_.obs_dim, spec_.control_dt, spec_.action.v_min,
    spec_.action.v_max, spec_.action.w_max, rule_.rule.c_str(), rule_.lookahead_m,
    rule_.follow_window, scan_topic_.c_str(), threads, max_v_, spec_.format, blocks.c_str(),
    speed_cap_, alpha_v, alpha_w, scan_mask_.size(), closing_note.c_str());
  for (const auto & m : scan_mask_) {
    RCLCPP_WARN(
      logger_, "FollowPathRL %s: scan beams at %.1f..%.1f deg (base frame) are masked as no "
      "return", name_.c_str(), m.lo_deg, m.hi_deg);
  }
}

void FollowPathRL::cleanup()
{
  scan_sub_.reset();
  policy_.reset();
  closing_.reset();
  have_closing_stamp_ = false;
}

void FollowPathRL::activate()
{
  reset();
}

void FollowPathRL::deactivate()
{
}

void FollowPathRL::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
  path_.resize(path.poses.size());
  for (size_t i = 0; i < path_.size(); ++i) {
    path_[i] = {path.poses[i].pose.position.x, path.poses[i].pose.position.y};
  }
  prog_ = 0;
  if (!path_.empty()) {
    if (new_episode(have_plan_, plan_end_, path_.back())) {
      memory_.reset();
      smoother_.reset();
      policy_ran_ = false;
    }
    plan_end_ = path_.back();
    have_plan_ = true;
  }
}

void FollowPathRL::reset()
{
  cmd_ = {0.0, 0.0};
  have_last_call_ = false;
  prog_ = 0;
  memory_.reset();
  smoother_.reset();
  policy_ran_ = false;
  have_plan_ = false;
}

void FollowPathRL::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    speed_limit_ = max_v_;
  } else if (percentage) {
    speed_limit_ = max_v_ * std::min(speed_limit, 100.0) / 100.0;
  } else {
    speed_limit_ = std::min(speed_limit, max_v_);
  }
}

// ---- 라이다 방향과 목표점 ----
double FollowPathRL::lidar_yaw(const std::string & frame)
{
  if (frame == lidar_frame_) {
    return lidar_yaw_;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    throw nav2_core::ControllerTFError(std::string("FollowPathRL: lidar frame: ") + e.what());
  }
  lidar_frame_ = frame;
  lidar_yaw_ = tf2::getYaw(t.transform.rotation);
  RCLCPP_INFO(
    logger_, "FollowPathRL: lidar %s in %s at (%.3f, %.3f) m, yaw %.3f rad", frame.c_str(),
    base_frame_.c_str(), t.transform.translation.x, t.transform.translation.y, lidar_yaw_);
  return lidar_yaw_;
}

void FollowPathRL::goals(GoalPolar & goal, GoalPolar & end, double & yaw_end)
{
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(
      plan_.header.frame_id, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & e) {
    throw nav2_core::ControllerTFError(std::string("FollowPathRL: plan frame: ") + e.what());
  }
  const Point2 robot{t.transform.translation.x, t.transform.translation.y};
  const double yaw = tf2::getYaw(t.transform.rotation);
  end = goal_in_robot_frame(robot, yaw, path_.back());
  const double e = tf2::getYaw(plan_.poses.back().pose.orientation) - yaw;
  yaw_end = std::atan2(std::sin(e), std::cos(e));
  goal = path_goal(path_, prog_, robot, yaw, rule_);
}

// ---- 다가옴 막기: 스캔 시각의 라이다 자세로 이력에 넣기 ----
void FollowPathRL::add_closing_scan(
  const sensor_msgs::msg::LaserScan & scan, const std::vector<float> & rays, double yaw_l,
  double now)
{
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(
      global_frame_, scan.header.frame_id, tf2_ros::fromMsg(scan.header.stamp),
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException &) {
    // 자세를 모르면 그 스캔만 건너뛴다(주행은 멈추지 않는다)
    ++closing_tf_misses_;
    return;
  }
  const double yaw = tf2::getYaw(t.transform.rotation) - yaw_l;
  closing_.add_scan(
    rays, t.transform.translation.x, t.transform.translation.y, yaw,
    rclcpp::Time(scan.header.stamp).seconds(), now);
}

// ---- 한 주기: 관측 -> 정책 -> (v, w) ----
geometry_msgs::msg::TwistStamped FollowPathRL::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto t_call = std::chrono::steady_clock::now();
  const rclcpp::Time now = clock_->now();
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = scan_;
  }
  if (!speed_mismatch_.empty()) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 10000, "FollowPathRL %s: %s", name_.c_str(), speed_mismatch_.c_str());
  }
  if (!scan) {
    throw nav2_core::ControllerException("FollowPathRL: no scan on " + scan_topic_);
  }
  const double scan_age = (now - rclcpp::Time(scan->header.stamp, now.get_clock_type())).seconds();
  if (scan_age > scan_timeout_) {
    throw nav2_core::ControllerException(
      "FollowPathRL: scan is " + std::to_string(scan_age) + " s old");
  }
  if (plan_.poses.empty()) {
    throw nav2_core::InvalidPath("FollowPathRL: empty plan");
  }
  GoalPolar goal{0.0, 0.0};
  GoalPolar end{0.0, 0.0};
  double yaw_end = 0.0;
  goals(goal, end, yaw_end);

  if (!have_last_call_ || (now - last_call_).seconds() > 2.0 * spec_.control_dt) {
    cmd_ = {velocity.linear.x, velocity.angular.z};
  }
  last_call_ = now;
  have_last_call_ = true;

  const auto & lim = spec_.action;
  Twist2 target{0.0, 0.0};
  std::vector<float> act{0.0f, 0.0f};
  double infer_us = 0.0;
  double turn_radius = spec_.success_radius;
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (goal_checker && goal_checker->getTolerances(pose_tol, vel_tol)) {
    turn_radius = pose_tol.position.x;
  }
  const bool rotating = end.dist < turn_radius;
  if (rotating) {
    target.w = std::clamp(rotate_gain_ * yaw_end, -lim.w_max, lim.w_max);
  } else {
    Scan s{scan->angle_min, scan->angle_increment, scan->range_min, scan->range_max,
      scan->ranges};
    const double yaw_l = lidar_yaw(scan->header.frame_id);
    mask_scan(s, yaw_l, scan_mask_);
    const auto rays = scan_to_rays(s, yaw_l, spec_);
    auto obs = build_observation(
      rays, goal.dist, goal.angle, velocity.linear.x, velocity.angular.z, spec_);
    const std::vector<float> sectors(obs.begin(), obs.begin() + spec_.lidar.sectors);
    // 직전 주기에 정책이 돌지 않았으면 기억과 평활 상태를 에피소드 시작으로 되돌린다
    if (memory_stale(policy_ran_, (now - last_policy_).seconds(), spec_.control_dt)) {
      memory_.reset();
      smoother_.reset();
    }
    if (!spec_.obs_blocks.empty()) {
      const auto extra = memory_.blocks(sectors, speed_cap_);
      obs.insert(obs.end(), extra.begin(), extra.end());
    }
    const auto t_infer = std::chrono::steady_clock::now();
    act = policy_->run(obs);
    infer_us = us_since(t_infer);
    if (!std::isfinite(act[0]) || !std::isfinite(act[1])) {
      throw nav2_core::NoValidControl("FollowPathRL: policy returned a non-finite action");
    }
    target = action_to_target(act[0], act[1], lim, v_top_);
    // prev_action 기억은 평활 전 정책 목표를 담고, 평활은 후진 막기·속도 상한·가속 제한 앞에 둔다
    memory_.step(sectors, target);
    last_policy_ = now;
    policy_ran_ = true;
    target = smoother_.step(target, cmd_);
    const double policy_v = target.v;
    target = block_reverse(target, rays, rear_stop_range_, rear_half_width_deg_, spec_);
    rear_blocks_ += target.v != policy_v ? 1 : 0;
    // 다가옴 막기: 새 스캔만 이력에 넣고, 걸려 있으면 전진 목표만 0으로
    if (closing_.enabled()) {
      if (!have_closing_stamp_ || scan->header.stamp != closing_stamp_) {
        closing_stamp_ = scan->header.stamp;
        have_closing_stamp_ = true;
        add_closing_scan(*scan, rays, yaw_l, now.seconds());
      }
      const double guarded_v = target.v;
      target = closing_.apply(target, now.seconds());
      closing_blocks_ += target.v != guarded_v ? 1 : 0;
    }
  }
  target.v = std::min(target.v, speed_limit_);
  cmd_ = rate_limit(cmd_, target, spec_.control_dt, lim);

  const double call_us = us_since(t_call);
  ++calls_;
  infer_sum_us_ += infer_us;
  infer_max_us_ = std::max(infer_max_us_, infer_us);
  call_sum_us_ += call_us;
  call_max_us_ = std::max(call_max_us_, call_us);
  RCLCPP_DEBUG(
    logger_, "FollowPathRL: goal %.3f m at %.3f rad, prog %zu, act (%.3f, %.3f) cmd (%.3f, %.3f) "
    "infer %.0f us", goal.dist, goal.angle, prog_, act[0], act[1], cmd_.v, cmd_.w, infer_us);
  if ((now - last_log_).seconds() >= log_period_) {
    std::string closing_note;
    if (closing_.enabled()) {
      char buf[64];
      std::snprintf(
        buf, sizeof(buf), " | closing blocks %d, tf misses %d", closing_blocks_,
        closing_tf_misses_);
      closing_note = buf;
    }
    RCLCPP_INFO(
      logger_, "FollowPathRL: %d calls, infer mean %.0f max %.0f us, call mean %.0f max %.0f us | "
      "%s goal %.3f m at %.2f rad, end %.3f m | act (%.2f, %.2f) cmd v %.3f w %.3f | rear blocks %d | scan age %.2f s%s",
      calls_, infer_sum_us_ / calls_, infer_max_us_, call_sum_us_ / calls_, call_max_us_,
      rotating ? "rotate" : "policy", goal.dist, goal.angle, end.dist, act[0],
      act[1], cmd_.v, cmd_.w, rear_blocks_, scan_age, closing_note.c_str());
    last_log_ = now;
    calls_ = 0;
    rear_blocks_ = 0;
    closing_blocks_ = 0;
    closing_tf_misses_ = 0;
    infer_sum_us_ = infer_max_us_ = call_sum_us_ = call_max_us_ = 0.0;
  }

  geometry_msgs::msg::TwistStamped out;
  out.header.frame_id = pose.header.frame_id;
  out.header.stamp = pose.header.stamp;
  out.twist.linear.x = cmd_.v;
  out.twist.angular.z = cmd_.w;
  return out;
}

}  // namespace pode_rl_controller

PLUGINLIB_EXPORT_CLASS(pode_rl_controller::FollowPathRL, nav2_core::Controller)

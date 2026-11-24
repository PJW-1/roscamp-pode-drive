#include "pode_follow_wall/follow_wall.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace pode_follow_wall
{

using nav2_util::declare_parameter_if_not_declared;
using rclcpp::ParameterValue;

FollowWall::FollowWall()
: loader_("nav2_core", "nav2_core::Controller")
{
}

// ---- 수명 주기 ----

void FollowWall::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("FollowWall: unable to lock node");
  }
  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 파라미터 선언과 읽기
  auto declare = [&](const std::string & key, const ParameterValue & value) {
      declare_parameter_if_not_declared(node, name + "." + key, value);
    };
  declare("primary_controller",
    ParameterValue(std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")));
  declare("primary_namespace", ParameterValue(std::string("rpp")));
  declare("scan_topic", ParameterValue(std::string("scan")));
  declare("map_topic", ParameterValue(std::string("map_fine")));
  declare("target_gap", ParameterValue(0.03));
  declare("max_correction", ParameterValue(0.02));
  declare("deadband", ParameterValue(0.005));
  declare("filter_alpha", ParameterValue(0.1));
  declare("window_min_angle", ParameterValue(1.0472));
  declare("window_max_angle", ParameterValue(2.0944));
  declare("window_range", ParameterValue(0.30));
  declare("ransac_iterations", ParameterValue(40));
  declare("ransac_threshold", ParameterValue(0.01));
  declare("min_points", ParameterValue(8));
  declare("max_fit_rms", ParameterValue(0.008));
  declare("max_angle_diff", ParameterValue(0.15));
  declare("max_error", ParameterValue(0.06));
  declare("occupied_threshold", ParameterValue(65));
  declare("shift_full_length", ParameterValue(0.30));
  declare("shift_taper_length", ParameterValue(0.20));
  declare("scan_timeout", ParameterValue(0.3));

  auto get_d = [&](const std::string & key) {return node->get_parameter(name + "." + key).as_double();};
  auto get_i = [&](const std::string & key) {
      return static_cast<int>(node->get_parameter(name + "." + key).as_int());
    };
  auto get_s = [&](const std::string & key) {return node->get_parameter(name + "." + key).as_string();};

  target_gap_ = get_d("target_gap");
  max_correction_ = get_d("max_correction");
  deadband_ = get_d("deadband");
  filter_alpha_ = get_d("filter_alpha");
  wall_params_.window.min_angle = get_d("window_min_angle");
  wall_params_.window.max_angle = get_d("window_max_angle");
  wall_params_.window.range = get_d("window_range");
  wall_params_.fit.ransac_iterations = get_i("ransac_iterations");
  wall_params_.fit.ransac_threshold = get_d("ransac_threshold");
  wall_params_.fit.min_points = get_i("min_points");
  wall_params_.fit.max_rms = get_d("max_fit_rms");
  wall_params_.max_angle_diff = get_d("max_angle_diff");
  wall_params_.max_error = get_d("max_error");
  wall_params_.occupied_threshold = get_i("occupied_threshold");
  shift_full_length_ = get_d("shift_full_length");
  shift_taper_length_ = get_d("shift_taper_length");
  scan_timeout_ = get_d("scan_timeout");

  // 보정 상한은 목표 틈보다 1 cm 이상 작게
  const double cap = std::max(0.0, target_gap_ - 0.01);
  if (max_correction_ > cap + 1e-9) {
    RCLCPP_WARN(logger_, "%s: max_correction %.3f > target_gap - 0.01, using %.3f",
      name.c_str(), max_correction_, cap);
    max_correction_ = cap;
  }

  // 품은 controller를 띄운다(파라미터는 name.primary_namespace 아래)
  const std::string primary_type = get_s("primary_controller");
  const std::string primary_name = name + "." + get_s("primary_namespace");
  primary_ = loader_.createSharedInstance(primary_type);
  primary_->configure(parent, primary_name, tf, costmap_ros);
  // 가까운 점 찾기 범위와 자세 변환 허용 시간은 품은 RPP와 같게(RPP처럼 음수 범위면 끝까지)
  robot_search_dist_ = std::numeric_limits<double>::max();
  if (node->has_parameter(primary_name + ".max_robot_pose_search_dist")) {
    const double d = node->get_parameter(primary_name + ".max_robot_pose_search_dist").as_double();
    if (d >= 0.0) {
      robot_search_dist_ = d;
    }
  }
  if (node->has_parameter(primary_name + ".transform_tolerance")) {
    transform_tolerance_ = node->get_parameter(primary_name + ".transform_tolerance").as_double();
  }

  // 입력과 진단 토픽
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    get_s("scan_topic"), rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {scanCallback(msg);});
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    get_s("map_topic"), rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {mapCallback(msg);});
  state_pub_ = node->create_publisher<msg::FollowWallState>(name + "/state", 10);

  RCLCPP_INFO(logger_, "%s: wrapping %s as %s (gap %.3f, max correction %.3f)",
    name.c_str(), primary_type.c_str(), primary_name.c_str(), target_gap_, max_correction_);
}

void FollowWall::cleanup()
{
  if (primary_) {
    primary_->cleanup();
  }
  primary_.reset();
  scan_sub_.reset();
  map_sub_.reset();
  state_pub_.reset();
}

void FollowWall::activate()
{
  primary_->activate();
  state_pub_->on_activate();
}

void FollowWall::deactivate()
{
  primary_->deactivate();
  state_pub_->on_deactivate();
}

// ---- 입력 ----

void FollowWall::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  scan_ = msg;
}

void FollowWall::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  map_ = msg;
}

// ---- 경로와 명령 ----

void FollowWall::setPlan(const nav_msgs::msg::Path & path)
{
  // 원래 경로를 그대로 넘기고, 옮긴 경로는 주기마다 다시 만든다
  plan_ = path;
  rpp_start_ = 0;
  shifted_ = false;
  primary_->setPlan(path);
}

double FollowWall::updateCorrection(
  const geometry_msgs::msg::PoseStamped & pose, msg::FollowWallState & state,
  double & normal_x, double & normal_y, std::size_t & start, std::size_t & nearest)
{
  // 보정하지 않을 때는 거른 값을 비운다
  auto none = [&]() {
      filtered_error_ = 0.0;
      filter_side_ = 0;
      return 0.0;
    };
  start = rpp_start_;
  nearest = rpp_start_;
  if (plan_.poses.empty()) {
    return none();
  }

  // 추정 자세(AMCL, 경로 틀): RPP PathHandler::transformPose와 같은 방법으로 변환한다
  geometry_msgs::msg::PoseStamped map_pose = pose;
  if (pose.header.frame_id != plan_.header.frame_id) {
    try {
      tf_->transform(pose, map_pose, plan_.header.frame_id,
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      // 품은 RPP를 부르지 않고 RPP와 같은 예외를 낸다(RPP 경로 상태가 앞서 나가지 않게)
      none();
      throw nav2_core::ControllerTFError(
              std::string("Unable to transform robot pose into global plan's frame: ") + ex.what());
    }
  }
  // 따로 띄운 RPP라면 이번 주기에 고를 점과 잘라 낸 뒤의 시작(옮기지 않은 경로 기준)
  nearest = rppClosestIndex(
    plan_, rpp_start_, map_pose.pose.position.x, map_pose.pose.position.y, robot_search_dist_);
  rpp_start_ = nearest;

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    scan = scan_;
    map = map_;
  }
  if (!scan || !map) {
    return none();
  }
  if (map->header.frame_id != plan_.header.frame_id) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: map frame %s != plan frame %s",
      plugin_name_.c_str(), map->header.frame_id.c_str(), plan_.header.frame_id.c_str());
    return none();
  }
  const double age =
    (clock_->now() - rclcpp::Time(scan->header.stamp, clock_->get_clock_type())).seconds();
  if (age > scan_timeout_) {
    return none();
  }

  // scan을 몸 기준 점으로 바꾼다(라이다 장착 각도를 가정하지 않는다)
  geometry_msgs::msg::TransformStamped tf_base_scan;
  try {
    tf_base_scan = tf_->lookupTransform(
      costmap_ros_->getBaseFrameID(), scan->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: %s", plugin_name_.c_str(), ex.what());
    return none();
  }
  const double tx = tf_base_scan.transform.translation.x;
  const double ty = tf_base_scan.transform.translation.y;
  const double tyaw = tf2::getYaw(tf_base_scan.transform.rotation);
  std::vector<Point2> points;
  points.reserve(scan->ranges.size());
  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const double r = scan->ranges[i];
    if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) {
      continue;
    }
    const double a = scan->angle_min + static_cast<double>(i) * scan->angle_increment + tyaw;
    points.push_back({tx + r * std::cos(a), ty + r * std::sin(a)});
  }

  // 벽 오차
  const double yaw = tf2::getYaw(map_pose.pose.orientation);
  const Pose2 p{map_pose.pose.position.x, map_pose.pose.position.y, yaw};
  const WallMeasurement m = measureWall(points, *map, p, wall_params_);
  state.wall_seen = m.valid;
  state.side = static_cast<int8_t>(m.side);
  state.measured_dist = m.measured.dist;
  state.expected_dist = m.expected.dist;
  state.error = m.error;
  state.line_start.x = m.measured.start.x;
  state.line_start.y = m.measured.start.y;
  state.line_end.x = m.measured.end.x;
  state.line_end.y = m.measured.end.y;
  state.fit_points = m.measured.inliers;
  state.fit_rms = m.measured.rms;
  state.angle_diff = m.angle_diff;
  if (!m.valid) {
    return none();
  }

  // 거르기, 데드밴드, 상한
  if (m.side != filter_side_) {
    filtered_error_ = 0.0;
    filter_side_ = m.side;
  }
  filtered_error_ += filter_alpha_ * (m.error - filtered_error_);
  state.filtered_error = filtered_error_;
  normal_x = std::cos(yaw) * m.measured.nx - std::sin(yaw) * m.measured.ny;
  normal_y = std::sin(yaw) * m.measured.nx + std::cos(yaw) * m.measured.ny;
  return shapeCorrection(filtered_error_, deadband_, max_correction_);
}

geometry_msgs::msg::TwistStamped FollowWall::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  msg::FollowWallState state;
  state.header.stamp = clock_->now();
  state.header.frame_id = costmap_ros_->getBaseFrameID();
  double nx = 0.0, ny = 0.0;
  std::size_t start = 0, nearest = 0;
  const double shift = updateCorrection(pose, state, nx, ny, start, nearest);

  // 보정이 있으면 옮긴 경로를, 보정이 끝나면 원래 경로를 RPP가 쥐고 있었을 시작부터 넘긴다
  if (shift != 0.0) {
    primary_->setPlan(shiftPath(
        plan_, start, nearest, shift * nx, shift * ny, shift_full_length_, shift_taper_length_));
    shifted_ = true;
  } else if (shifted_) {
    primary_->setPlan(shiftPath(plan_, start, nearest, 0.0, 0.0, 0.0, 0.0));
    shifted_ = false;
  }
  state.applied_shift = shift;
  if (state_pub_ && state_pub_->is_activated()) {
    state_pub_->publish(state);
  }
  return primary_->computeVelocityCommands(pose, velocity, goal_checker);
}

bool FollowWall::cancel()
{
  return primary_->cancel();
}

void FollowWall::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_->setSpeedLimit(speed_limit, percentage);
}

void FollowWall::reset()
{
  filtered_error_ = 0.0;
  filter_side_ = 0;
  primary_->reset();
}

}  // namespace pode_follow_wall

PLUGINLIB_EXPORT_CLASS(pode_follow_wall::FollowWall, nav2_core::Controller)

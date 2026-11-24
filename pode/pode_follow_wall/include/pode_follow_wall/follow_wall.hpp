// FollowWall: RPP를 품고, 라이다 벽 거리 오차만큼 RPP에 주는 경로를 옆으로 옮긴다
#ifndef PODE_FOLLOW_WALL__FOLLOW_WALL_HPP_
#define PODE_FOLLOW_WALL__FOLLOW_WALL_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "pode_follow_wall/msg/follow_wall_state.hpp"
#include "pode_follow_wall/wall_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace pode_follow_wall
{

class FollowWall : public nav2_core::Controller
{
public:
  FollowWall();
  ~FollowWall() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

protected:
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  // 이번 주기의 보정량을 구하고 진단 값을 채운다
  double updateCorrection(
    const geometry_msgs::msg::PoseStamped & pose, msg::FollowWallState & state,
    double & normal_x, double & normal_y, std::size_t & start, std::size_t & nearest);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("FollowWall")};
  rclcpp::Clock::SharedPtr clock_;

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr primary_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<msg::FollowWallState>::SharedPtr state_pub_;

  std::mutex data_mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;

  // 파라미터
  WallParams wall_params_;
  double target_gap_{0.03};
  double max_correction_{0.02};
  double deadband_{0.005};
  double filter_alpha_{0.1};
  double shift_full_length_{0.30};
  double shift_taper_length_{0.20};
  double scan_timeout_{0.3};
  double transform_tolerance_{0.2};

  // 주기 사이 상태
  nav_msgs::msg::Path plan_;
  double robot_search_dist_{1.0};
  std::size_t rpp_start_{0};  // 따로 띄운 RPP가 잘라 낸 뒤 경로의 시작
  int filter_side_{0};
  double filtered_error_{0.0};
  bool shifted_{false};
};

}  // namespace pode_follow_wall

#endif  // PODE_FOLLOW_WALL__FOLLOW_WALL_HPP_

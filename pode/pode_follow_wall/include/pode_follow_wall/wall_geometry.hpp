// FollowWall의 계산부: ROS 노드 없이 시험할 수 있는 순수 함수들
#ifndef PODE_FOLLOW_WALL__WALL_GEOMETRY_HPP_
#define PODE_FOLLOW_WALL__WALL_GEOMETRY_HPP_

#include <cstddef>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace pode_follow_wall
{

struct Point2
{
  double x;
  double y;
};

// 직선 맞추기 설정
struct FitParams
{
  int ransac_iterations{40};
  double ransac_threshold{0.01};
  int min_points{8};
  double max_rms{0.008};
};

// 맞춘 직선: 몸 기준, 법선은 원점에서 벽 쪽을 향한다
struct LineFit
{
  bool ok{false};
  double cx{0.0}, cy{0.0};  // 인라이어 중심
  double dx{1.0}, dy{0.0};  // 방향 단위 벡터
  double nx{0.0}, ny{1.0};  // 원점에서 벽 쪽 법선
  double dist{0.0};  // 원점에서 직선까지 수직 거리
  double rms{0.0};
  int inliers{0};
  Point2 start{0.0, 0.0};  // 인라이어를 직선에 투영한 양 끝
  Point2 end{0.0, 0.0};
};

// 벽 창 설정
struct WindowParams
{
  double min_angle{1.0472};  // 60도
  double max_angle{2.0944};  // 120도
  double range{0.30};
};

// 벽 판정 설정
struct WallParams
{
  WindowParams window;
  FitParams fit;
  double max_angle_diff{0.15};  // 잰 벽과 지도 벽의 방향 차 상한
  double max_error{0.06};  // 이보다 큰 e는 벽이 아닌 물체로 본다
  int occupied_threshold{65};
};

// 한쪽 벽 판정 결과
struct WallMeasurement
{
  bool valid{false};
  int side{0};  // 1 왼쪽, -1 오른쪽, 0 없음
  LineFit measured;
  LineFit expected;
  double angle_diff{0.0};
  double error{0.0};  // 잰 거리 - 기대 거리
};

// 2D 자세
struct Pose2
{
  double x;
  double y;
  double yaw;
};

// 점들에 직선을 맞춘다(RANSAC 뒤 인라이어 최소제곱)
LineFit fitLine(const std::vector<Point2> & points, const FitParams & params);

// 한쪽 옆 창 안의 점만 고른다(side 1 왼쪽, -1 오른쪽)
std::vector<Point2> selectWindow(
  const std::vector<Point2> & points, int side, const WindowParams & params);

// 지도에서 광선이 처음 만나는 점유 칸까지 거리, 못 만나면 음수
double raycast(
  const nav_msgs::msg::OccupancyGrid & grid, double x0, double y0, double theta,
  double max_range, int occupied_threshold);

// 같은 자세에서 지도로 만든 가상 점(몸 기준), 잰 점들과 같은 방향으로 쏜다
std::vector<Point2> expectedPoints(
  const nav_msgs::msg::OccupancyGrid & grid, const Pose2 & pose,
  const std::vector<Point2> & directions, double max_range, int occupied_threshold);

// 한쪽 벽의 오차 e를 구한다
WallMeasurement measureSide(
  const std::vector<Point2> & scan_points, const nav_msgs::msg::OccupancyGrid & grid,
  const Pose2 & pose, int side, const WallParams & params);

// 양쪽을 보고 믿을 만한 쪽 가운데 가까운 벽을 고른다
WallMeasurement measureWall(
  const std::vector<Point2> & scan_points, const nav_msgs::msg::OccupancyGrid & grid,
  const Pose2 & pose, const WallParams & params);

// 데드밴드와 상한을 건 보정량
double shapeCorrection(double filtered_error, double deadband, double max_correction);

// 품은 RPP(Jazzy 1.3 PathHandler)와 같은 규칙으로 고른 가까운 점, RPP는 이 점 앞을 잘라 낸다
std::size_t rppClosestIndex(
  const nav_msgs::msg::Path & path, std::size_t start, double x, double y, double search_dist);

// start부터의 경로를 옮긴다: nearest 뒤 full_length까지 모두, taper_length 동안 줄여 0
nav_msgs::msg::Path shiftPath(
  const nav_msgs::msg::Path & path, std::size_t start, std::size_t nearest,
  double offset_x, double offset_y, double full_length, double taper_length);

}  // namespace pode_follow_wall

#endif  // PODE_FOLLOW_WALL__WALL_GEOMETRY_HPP_

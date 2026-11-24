# FollowWall 한 주기의 진단 값(controller 주기마다 낸다)
std_msgs/Header header

# 믿을 만한 벽을 봤는지
bool wall_seen
# 고른 벽: 1 왼쪽, -1 오른쪽, 0 없음
int8 side

# 라이다로 잰 벽 거리와 지도 기대 거리(몸 중심에서 수직, m)
float64 measured_dist
float64 expected_dist
# e = 잰 거리 - 기대 거리, 거른 값
float64 error
float64 filtered_error
# 실제로 옮긴 양(+ 벽 쪽, m)
float64 applied_shift

# 맞춘 직선(몸 기준 양 끝), 점 수, 잔차, 지도 벽과 방향 차(rad)
geometry_msgs/Point line_start
geometry_msgs/Point line_end
int32 fit_points
float64 fit_rms
float64 angle_diff

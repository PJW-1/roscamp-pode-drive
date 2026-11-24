# FollowWall 플러그인 (청소 로봇 D3, 2026-11-24)

브랜치 `feat/follow-wall-2026-11-24`(main `aff00d7`에서). 계획: 청소로봇_도입_계획_2026-11-24 4판 6-1, 9절 D3.
새 패키지 `pode/pode_follow_wall`만 더했다. `nav2_params.yaml`, `make_params.py`, launch 파일, pode_fsm은 고치지 않았다.

## 무엇을 하는가

`pode_follow_wall::FollowWall`(`nav2_core::Controller`, pluginlib)은 RPP를 품고, 라이다로 잰 벽 거리와
지도 기대 거리의 차 e만큼 RPP에 주는 경로를 옆으로 옮긴다. 벽이 안 보이거나 믿을 만하지 않으면
경로를 옮기지 않으므로 출력이 RPP와 같다.

매 controller 주기(`computeVelocityCommands`):

1. 받은 pose를 경로 틀(map)로 바꾼다. 이것이 추정 자세(AMCL)다. 경로에서 가장 가까운 점은 품은 RPP(Jazzy 1.3 `PathHandler::transformGlobalPlan`)와 같은 규칙으로 고른다: RPP가 잘라 낸 뒤의 시작부터 `FollowWall.rpp.max_robot_pose_search_dist`(적분 거리) 안, 같은 거리면 뒤의 점, 찾는 끝 바로 앞이면 한 칸 앞. 그 점을 다음 주기의 시작으로 기억해, 따로 띄운 RPP가 쥐고 있을 경로 상태를 그대로 따라간다.
2. 최신 scan을 TF(`base_footprint` ← scan 틀)로 몸 기준 점으로 바꾼다. 라이다가 180° 돌아 있어도 각도를 가정하지 않는다.
3. 왼쪽, 오른쪽 옆 창(60°~120°, 0.3 m 안)의 점에 직선을 맞춘다: RANSAC(고정 시드) 뒤 인라이어 최소제곱(주성분).
4. 같은 추정 자세에서, 창 안의 각 점과 같은 방향으로 지도(`map_fine`)에 광선을 쏴(DDA, 칸 경계까지 정확, 지도 밖이나 미지 칸(−1)을 만나면 기대 거리 없음) 가상 점을 만들고 같은 방법으로 직선을 맞춘다. 기대 거리 = 그 직선까지 수직 거리.
5. 믿을 만하면(두 직선 모두 점 수 ≥ 8, 잔차 ≤ 8 mm, 방향 차 ≤ 0.15 rad, |e| ≤ 0.06 m) e = 잰 거리 − 기대 거리. 두 쪽이 다 되면 가까운 벽을 고른다.
6. 거르기: `filtered += alpha * (e - filtered)`. 벽을 잃거나 쪽이 바뀌면 0부터 다시 한다.
7. 보정량 = 데드밴드(|filtered| < 5 mm면 0) 뒤 ±상한(2 cm)으로 자른다. 옮기는 방향은 벽 쪽 법선(map 틀)이고, e가 양수(실제 벽이 더 멀다)면 벽 쪽, 음수면 반대쪽이다.
8. 보정이 있으면 RPP가 쥐고 있었을 시작부터의 경로를 옮겨 RPP에 준다: 로봇 앞 0.3 m까지는 모두, 다음 0.2 m 동안 줄여 0. 자세(방향)는 바꾸지 않는다. 보정이 끝나면 원래 경로를 같은 시작부터 한 번 다시 준다. 보정이 없는 동안은 RPP의 `setPlan`을 부르지 않는다(RPP 안의 경로 상태가 따로 띄운 RPP와 똑같다).
9. 진단 메시지를 내고 품은 RPP의 `computeVelocityCommands` 결과를 그대로 돌려준다.

잰 거리와 기대 거리가 같은 추정 자세에서 나오므로 경로 추종 오차(로봇이 경로 옆에 있음)는 차에서 빠지고(시험 `FollowErrorGivesZero`, `FollowErrorOnlySameAsRpp`), 위치 추정의 벽 수직 오차와 지도 오차만 남는다.

`setSpeedLimit`, `cancel`, `reset`, 수명 주기는 품은 RPP에 그대로 넘긴다.

## 파라미터 (이름공간 `FollowWall.`)

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `primary_controller` | RPP | 품은 controller 종류 |
| `primary_namespace` | `rpp` | 품은 controller의 파라미터는 `FollowWall.rpp.*` |
| `scan_topic`, `map_topic` | `scan`, `map_fine` | 상대 이름(로봇 namespace 안). 지도는 AMCL과 같은 위치 추정 지도 |
| `target_gap` | 0.03 | 목표 틈 g |
| `max_correction` | 0.02 | 보정 상한. g − 0.01보다 크면 경고하고 g − 0.01로 쓴다 |
| `deadband` | 0.005 | 이보다 작은 거른 e는 0 |
| `filter_alpha` | 0.1 | 주기(20 Hz)마다 따라가는 비율(약 0.5 s) |
| `window_min_angle`, `window_max_angle`, `window_range` | 1.0472, 2.0944, 0.30 | 옆 창 |
| `ransac_iterations`, `ransac_threshold`, `min_points`, `max_fit_rms` | 40, 0.01, 8, 0.008 | 직선 맞추기 |
| `max_angle_diff` | 0.15 | 잰 벽과 지도 벽 방향 차 상한(rad) |
| `max_error` | 0.06 | 이보다 큰 e는 벽 앞 물체로 보고 버린다 |
| `occupied_threshold` | 65 | 지도 점유 문턱 |
| `shift_full_length`, `shift_taper_length` | 0.30, 0.20 | 경로 옮기는 길이 |
| `scan_timeout` | 0.3 | scan이 이보다 오래되면 보정하지 않는다 |

예시 파일: `pode/pode_follow_wall/params/cleaner_follow_wall.yaml`(controller_server 블록, `FollowWall.rpp`는 지금 `FollowPath`와 키·값이 모두 같음을 확인).

## 진단 토픽

`<namespace>/FollowWall/state` (`pode_follow_wall/msg/FollowWallState`, controller 주기마다):
`wall_seen`, `side`(1 왼쪽, −1 오른쪽, 0 없음), `measured_dist`, `expected_dist`, `error`, `filtered_error`,
`applied_shift`(+ 벽 쪽), 맞춘 직선의 양 끝(`line_start`, `line_end`, 몸 기준), `fit_points`, `fit_rms`, `angle_diff`.

## 시험

```
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=94
colcon build --packages-select pode_follow_wall --build-base build_fw --install-base install_fw
colcon test --packages-select pode_follow_wall --build-base build_fw --install-base install_fw
colcon test-result --test-result-base build_fw/pode_follow_wall --verbose
# Summary: 40 tests, 0 errors, 0 failures, 0 skipped
```

`test_wall_geometry`(24개, 노드 없음):
- 직선 맞추기: 깨끗한 벽, 잡음 2 mm + 벽 안쪽 물체 점, 끊긴 벽, 오른쪽 벽, 모서리(옆 벽을 고르고 오차 1 mm 안), 점 부족·흩어진 점은 믿지 않음
- 창 고르기, 지도 광선(칸 경계까지 정확, 미지 칸 뒤의 벽은 기대 거리로 쓰지 않음), 기대 점이 추정 yaw를 따름
- e: 추정 자세가 맞으면 로봇이 어디 있든 0, 실제가 2 cm 멀면 +0.02, 1.5 cm 가까우면 −0.015, 오른쪽 벽 부호, 지도에 없는 벽·큰 e·방향이 다른 벽·창 밖 벽은 버림, 양쪽이면 가까운 벽
- 보정: 데드밴드, 상한, 부호
- 가까운 점: RPP와 같은 규칙(되돌아가지 않음, 같은 거리면 뒤의 점, 찾는 끝 바로 앞이면 한 칸 앞, 찾는 길이 안의 이웃 줄로 넘어감)
- 경로: 곧은 줄(앞 0.3 m 모두, 줄이는 구간 기울기 ≤ atan(0.02/0.2)), 줄 끝(남은 점이 같이 옮겨져 모양·간격·끝 방향 그대로), 모서리(어느 점도 2 cm 넘게 안 옮겨짐, 점이 거꾸로 가지 않음)

`test_follow_wall`(16개, RPP와 견주기, 시간에 기대는 것은 진단 토픽 수신 대기뿐):
- 같은 노드에 RPP를 하나 더 띄우고(`RPP.*`), FollowWall 안의 RPP(`FollowWall.rpp.*`)와 같은 파라미터를 준다. 같은 경로, 같은 costmap, 같은 TF, 같은 pose와 속도로 매 주기 둘을 부르고 `linear.x`, `angular.z`가 **정확히 같은지**(`ASSERT_DOUBLE_EQ`) 본다. 줄을 따라 260주기 가까이 돌며 yaw를 흔들고 로봇을 경로 옆에 둔다. 견줄 값이 0만이 아닌지(움직이는 주기 절반 넘게, 도는 주기 1/4 넘게)도 본다.
- 라이다 틀은 몸 뒤 2 cm, 180° 돌아 있다. scan은 참 자세에서 지도 벽까지 광선을 계산해 만든다.
- RPP와 같아야 하는 판: scan·지도 없음, 벽 없음, 지도에 없는 벽(물체), 창 밖 벽, 오래된 scan, 추정 자세는 맞고 로봇만 경로에서 1.5 cm 벗어남(e = 0), 벽을 잃은 뒤(원래 경로로 돌아감), 지도 벽 앞 미지 칸 띠(위치 추정 오차가 있어도 보정 0), U자 경로(이웃 줄 0.10 m)에서 서쪽 줄로 흔들려 RPP가 동쪽 줄을 잘라 낸 뒤 U턴 앞에서 벽을 잃음, 속도 제한 넘김
- 달라야 하는 판: 실제 벽이 3 cm 멀면 진단 e ≈ 0.03, 보정은 0~0.02로 상한에서 멈추고 RPP보다 왼쪽(벽 쪽)으로 돈다. 1.5 cm 가까우면 오른쪽으로 돈다
- pluginlib가 `pode_follow_wall::FollowWall` 이름으로 띄운다

## 계획 6-1과 다르거나 정한 것

- 기대 거리: 계획의 "추정 자세에서 지도로 같은 벽까지의 기대 거리"를, 잰 점과 같은 방향으로 지도 광선을 쏴 같은 직선 맞추기를 하는 방법으로 구현했다. 지도 벽의 방향도 같이 나와서 방향 차 판정에 쓴다.
- 품은 RPP의 파라미터 이름공간을 `FollowWall.rpp`로 따로 두었다(RotationShim은 같은 이름공간을 같이 쓴다). FollowWall 자신의 키와 섞이지 않게 하려는 것이다. D4에서 `make_params.py`의 대상은 `FollowWall.rpp.desired_linear_vel`, `FollowWall.rpp.use_collision_detection`이다.
- 계획에 값이 없던 것을 정했다: 데드밴드 5 mm, 거르기 0.1/주기, |e| 상한 0.06 m(벽 앞 물체 거르기), 두 벽이 다 보이면 가까운 벽, 경로 옮기는 길이 0.3 m + 줄이기 0.2 m.
- 벽을 잃으면 거른 값을 바로 0으로 하고 원래 경로로 돌아간다(계획의 "벽이 안 보이면 RPP와 똑같다"를 그대로 따름). 그래서 벽이 끊겼다 이어지면 보정이 0부터 다시 쌓인다.
- 위치 추정 지도는 `map_fine`(지금 nav2_params의 AMCL `map_topic`)을 기본으로 했다.
- 진단 토픽의 bag 녹화(`bag_record.sh` 토픽 목록 추가)는 이번 범위(새 패키지만) 밖이라 D4에서 한다.
- 새로 쓴 시험 두 곳의 허용치를 첫 통과 전에 고쳤다: 지도 광선 거리 1e-9 → 1e-6(OccupancyGrid 해상도가 float32라 0.01이 정확히 안 맞음), 모서리 직선 거리 1e-6 → 1 mm(모서리 근처 앞 벽 점 하나가 RANSAC 문턱 1 cm 안에 들어와 0.47 mm 치우침, 데드밴드 5 mm보다 훨씬 작음).

## Codex 검수 뒤 고친 것 (fix 커밋)

- 벽을 잃은 뒤 경로 복원: 전에는 FollowWall이 따로 찾은 가까운 점(1.0 m 안, 같은 거리면 앞의 점) − 2부터 원래 경로를 넘겨, RPP가 이미 잘라 낸 점을 되살릴 수 있었다. U자 시험에서 고치기 전 FollowWall 0.12 m/s, RPP 0 m/s로 갈렸다. 지금은 RPP와 같은 규칙으로 RPP의 잘라 낸 시작을 따라가고 그 시작부터 넘긴다. RPP 코드를 부르지 않고 규칙을 옮겨 쓴 것은 RPP의 `PathHandler`가 RPP 객체 안에 숨어 있어 꺼내 볼 수 없기 때문이다. Nav2를 올리면 규칙이 바뀌었는지 다시 본다(시험 `WallLostNearUTurnSameAsRpp`가 잡는다). `max_robot_pose_search_dist`와 `transform_tolerance`는 configure 때 한 번 읽는다(실행 중 바꾸면 따라가지 않는다).
- 자세 변환(재검수 두 번): FollowWall이 자기 `transform_tolerance`(0.2)와 `nav2_util::transformPoseInTargetFrame`(늦으면 마지막 TF로 대신함)으로 변환해, 품은 RPP와 변환 성공 여부가 갈릴 수 있었다. FollowWall만 실패하면 RPP는 경로를 자르는데 따라가는 시작은 머물렀다. 지금은
  - FollowWall 자기 파라미터를 없애고 `FollowWall.rpp.transform_tolerance`를 읽으며, 변환도 RPP `PathHandler::transformPose`와 같은 방법(같은 틀이면 그대로, 아니면 `tf_->transform`에 같은 허용 시간, 대신 쓰기 없음)으로 한다. 품은 RPP 값이 0.2보다 크면 FollowWall이 예전보다 그만큼 오래 기다린다.
  - 변환에 실패하면 품은 RPP를 부르지 않고 바로 `nav2_core::ControllerTFError`를 낸다. 단독 RPP도 같은 허용 시간 안에 변환하지 못하면 같은 예외를 내므로 결과가 같고, RPP 경로 상태가 앞서 나가지 않는다. 그래서 FollowWall이 실패한 뒤 RPP가 기다리는 동안 TF가 들어와 갈리는 경우도 없어졌다.
  - 시험 `FollowWallOdomTest.TfFailureThrowsWithoutCallingRppThenSameAsRpp`(시간에 기대지 않음): 보정 중 map->odom이 없는 먼 미래 시각의 자세로 부르면 FollowWall과 RPP 대조군이 모두 `ControllerTFError`를 내고, 품은 RPP의 `computeVelocityCommands`와 `setPlan`이 불리지 않는다(감싼 controller로 횟수를 셈). TF를 되살린 뒤 U자 경로에서 흔들림, 보정, 벽 잃음을 거쳐 매 주기 RPP와 출력이 같다. 고치기 전 코드에서는 실패 주기에 품은 RPP가 불려 횟수 시험이 실패했다. 이 시험이 sleep과 스레드로 TF를 늦게 넣던 `FollowWallLateTfTest`를 대신한다. 옛 시험은 실행 순서에 따라 고치기 전 코드도 통과하거나 고친 코드도 실패할 수 있었다.
- 미지 칸: 광선이 미지 칸을 만나면 기대 거리 없음으로 멈춘다. 그러면 지도 직선을 못 맞춰 보정하지 않는다.

고치지 않은 지적:
- 줄 끝에서 끝점도 같이 옮겨짐: follow_path의 `general_goal_checker`가 xy 0.05 m이고 옮기는 양은 최대 2 cm라 도착 판정이 깨지지 않는다. 남은 경로를 같은 벡터로 옮기는 것은 위치 오차를 일관되게 고치는 방법이다.
- 모서리 뒤도 같은 벡터로 옮김: 같은 이유로 평행 이동이라 모양이 유지된다.
- 오래된 scan과 지금 자세를 섞음(추측 지적): 0.12 m/s에서 영향은 데드밴드보다 작다. Gazebo에서 확인한다(남은 일).

## 남은 일

- D4 연결: 청소 로봇 Nav2 파라미터에 이 블록을 합치기(`make_params.py` 대상 목록에 `FollowWall.rpp` 더하기, 시뮬 `sim_nav_params` 확인), 청소 로봇 pode_fsm에 `controller_id=FollowWall`과 빈 `through_controller_id`(`real_pode.launch.py`, `launch_parts.py`), `ros2 param get` launch 시험, `pode_nav_config`의 `exec_depend`.
- `bag_record.sh`와 시뮬 녹화에 `FollowWall/state` 더하기.
- Gazebo 비교(D4): 같은 가장자리 경로를 RPP만 / FollowWall로, 추종 오차 판(경로 옆 2 cm 출발, e ≈ 0)과 위치 추정 오차 판(AMCL 초기 자세 옆 2~3 cm, e가 따라감), 참값으로 틈 평균·표준편차·최솟값, BLOCKED 0. scan 지연(최대 `scan_timeout` 0.3 s)과 지금 자세를 섞어 생기는 e가 데드밴드보다 작은지도 본다.
- 실물(D5, D7): `scan_info.py`로 range_min과 각도 간격 확인(벽까지 약 0.07 m가 range_min보다 커야 함), Pi 4 CPU.
- 실제 controller_server 안에서 띄워 본 적은 아직 없다(단위 시험은 같은 노드에서 직접 configure, pluginlib 이름 로드만 확인).

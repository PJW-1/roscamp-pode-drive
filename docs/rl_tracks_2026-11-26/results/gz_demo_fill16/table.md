| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHistFill | 통로 | 7/10 | 18.2 ± 1.8 | 1.187 ± 0.347 | 0.050 ± 0.000 | 0.011 ± 0.014 | 12 ± 6 (31.3/분, 9.69/m) | 8 (2.2/분, 0.67/m) | 4 (1.1/분, 0.34/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 5/7 | 18.2 ± 1.6 | 1.195 ± 0.259 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 3 (32.4/분, 11.12/m) | 13 (4.5/분, 1.55/m) | 4 (1.4/분, 0.48/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 밖 | 1/3 | 25.0 ± 0.0 | 1.698 ± 0.086 | 0.050 ± 0.000 | 0.000 ± 0.000 | 15 ± 1 (27.4/분, 8.83/m) | 13 (7.9/분, 2.55/m) | 2 (1.2/분, 0.39/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHistFill | 통로 | 0.0 ± 0.0 | 4.0 ± 0.8 | 0.9 ± 0.5 | 13.3 ± 0.9 | 0.087 ± 0.006 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 4.0 ± 0.8 | 0.8 ± 0.3 | 13.4 ± 0.9 | 0.093 ± 0.004 |
| g_demo | FollowPathRLShimDemoHist | 밖 | 0.0 ± 0.0 | 3.4 ± 0.0 | 0.3 ± 0.0 | 21.2 ± 0.0 | 0.060 ± 0.000 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHistFill | 10/10 | 10/10 | 10/10 | 0.002 ± 0.007, 0.024 | 3.7 | 5 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHist | 7/10 | 9/10 | 8/10 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 시간 s | 0.03 ± 1.01 | t 0.0 | 5/7 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 달린 길 m | 0.01 ± 0.15 | t 0.1 | 7/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 몸 최소 거리 m | -0.01 ± 0.00 | t -2.4 | 7/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 도리도리 /분 | 4.25 ± 8.76 | t 0.5 | 7/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 주행 vx m/s | 0.01 ± 0.01 | t 0.4 | 7/10 |

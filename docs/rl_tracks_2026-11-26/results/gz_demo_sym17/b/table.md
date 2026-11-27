| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHistSym | 통로 | 10/10 | 12.6 ± 0.4 | 1.148 ± 0.016 | 0.050 ± 0.000 | 0.028 ± 0.009 | 11 ± 1 (51.1/분, 9.32/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 3/7 | 19.8 ± 0.9 | 0.933 ± 0.488 | 0.050 ± 0.000 | 0.002 ± 0.005 | 10 ± 4 (21.1/분, 10.72/m) | 7 (2.1/분, 1.07/m) | 8 (2.4/분, 1.22/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 밖 | 0/3 | - | 2.086 ± 0.254 | 0.050 ± 0.000 | 0.000 ± 0.000 | 22 ± 7 (22.5/분, 10.39/m) | 7 (2.4/분, 1.12/m) | 4 (1.4/분, 0.64/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHistSym | 통로 | 0.0 ± 0.0 | 2.0 ± 0.6 | 0.4 ± 0.2 | 10.2 ± 0.5 | 0.110 ± 0.007 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 4.1 ± 0.9 | 0.9 ± 0.4 | 14.8 ± 0.4 | 0.079 ± 0.005 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHistSym | 10/10 | 10/10 | 10/10 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHist | 7/10 | 8/10 | 7/10 | 0.003 ± 0.009, 0.027 | 0.8 | 6 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 시간 s | 7.22 ± 0.52 | t 14.0 | 3/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 달린 길 m | -0.21 ± 0.18 | t -1.2 | 7/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 몸 최소 거리 m | -0.03 ± 0.00 | t -7.9 | 7/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 도리도리 /분 | -25.98 ± 6.31 | t -4.1 | 7/10 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 주행 vx m/s | -0.06 ± 0.01 | t -5.8 | 7/10 |

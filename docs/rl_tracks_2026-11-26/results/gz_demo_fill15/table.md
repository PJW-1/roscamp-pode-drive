| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistFill | 통로 | 5/5 | 13.0 ± 0.4 | 1.169 ± 0.016 | 0.050 ± 0.001 | 0.046 ± 0.003 | 10 ± 2 (47.9/분, 8.90/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimDemoHistMask | 통로 | 5/5 | 13.3 ± 1.2 | 1.169 ± 0.005 | 0.050 ± 0.000 | 0.018 ± 0.007 | 8 ± 1 (36.1/분, 6.84/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 5/5 | 13.0 ± 0.3 | 1.161 ± 0.008 | 0.052 ± 0.005 | 0.049 ± 0.004 | 11 ± 2 (48.9/분, 9.13/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHistFill | 통로 | 2/4 | 19.9 ± 3.1 | 0.966 ± 0.491 | 0.050 ± 0.000 | 0.000 ± 0.000 | 11 ± 5 (23.8/분, 11.39/m) | 9 (4.9/분, 2.33/m) | 4 (2.2/분, 1.04/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHistFill | 밖 | 0/1 | - | 1.497 ± 0.000 | 0.050 ± 0.000 | 0.000 ± 0.000 | 15 ± 0 (15.6/분, 10.02/m) | 1 (1.0/분, 0.67/m) | 3 (3.1/분, 2.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHistMask | 통로 | 2/3 | 14.2 ± 3.2 | 1.124 ± 0.111 | 0.050 ± 0.000 | 0.000 ± 0.000 | 18 ± 6 (78.2/분, 16.31/m) | 4 (5.7/분, 1.19/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHistMask | 밖 | 0/2 | - | 1.141 ± 0.181 | 0.050 ± 0.000 | 0.000 ± 0.000 | 14 ± 2 (56.6/분, 11.83/m) | 2 (4.2/분, 0.88/m) | 0 (0.0/분, 0.00/m) | 11 ± 1 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 5/5 | 18.7 ± 1.2 | 1.404 ± 0.076 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 2 (42.9/분, 9.54/m) | 5 (3.2/분, 0.71/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistFill | 통로 | 0.0 ± 0.0 | 2.6 ± 0.2 | 0.5 ± 0.3 | 9.9 ± 0.1 | 0.118 ± 0.000 |
| f_cart | FollowPathRLShimDemoHistMask | 통로 | 0.0 ± 0.0 | 2.9 ± 0.8 | 0.6 ± 0.4 | 9.8 ± 0.0 | 0.120 ± 0.000 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.6 ± 0.2 | 0.6 ± 0.1 | 9.8 ± 0.1 | 0.119 ± 0.001 |
| g_demo | FollowPathRLShimDemoHistFill | 통로 | 0.0 ± 0.0 | 4.0 ± 0.6 | 0.8 ± 0.0 | 15.1 ± 2.5 | 0.081 ± 0.011 |
| g_demo | FollowPathRLShimDemoHistMask | 통로 | 0.0 ± 0.0 | 3.2 ± 1.4 | 0.5 ± 0.4 | 10.5 ± 1.4 | 0.117 ± 0.002 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 3.7 ± 0.8 | 0.8 ± 0.1 | 14.2 ± 1.3 | 0.084 ± 0.010 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistFill | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| f_cart | FollowPathRLShimDemoHistMask | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| f_cart | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHistFill | 4/5 | 5/5 | 4/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHistMask | 3/5 | 5/5 | 3/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 시간 s | -0.02 ± 0.23 | t -0.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 달린 길 m | -0.01 ± 0.01 | t -1.0 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 몸 최소 거리 m | 0.00 ± 0.00 | t 1.4 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 도리도리 /분 | 1.25 ± 5.45 | t 0.2 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 주행 vx m/s | 0.00 ± 0.00 | t 0.6 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 시간 s | -0.31 ± 0.56 | t -0.5 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 달린 길 m | -0.01 ± 0.00 | t -1.9 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 몸 최소 거리 m | 0.03 ± 0.00 | t 8.5 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 도리도리 /분 | 13.10 ± 3.88 | t 3.4 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 주행 vx m/s | -0.00 ± 0.00 | t -2.7 | 5/5 |
| f_cart | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 시간 s | 0.29 ± 0.57 | t 0.5 | 5/5 |
| f_cart | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 달린 길 m | 0.00 ± 0.01 | t 0.0 | 5/5 |
| f_cart | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 몸 최소 거리 m | -0.03 ± 0.00 | t -8.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 도리도리 /분 | -11.85 ± 4.10 | t -2.9 | 5/5 |
| f_cart | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 주행 vx m/s | 0.00 ± 0.00 | t 8.6 | 5/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 시간 s | -1.18 ± 2.27 | t -0.5 | 5/2 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 달린 길 m | 0.44 ± 0.25 | t 1.8 | 5/4 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/4 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 도리도리 /분 | 13.78 ± 10.69 | t 1.3 | 5/4 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistFill | 주행 vx m/s | 0.03 ± 0.02 | t 1.8 | 5/4 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 시간 s | 4.56 ± 2.30 | t 2.0 | 5/2 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 달린 길 m | 0.28 ± 0.07 | t 3.9 | 5/3 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/3 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 도리도리 /분 | -34.46 ± 9.28 | t -3.7 | 5/3 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistMask | 주행 vx m/s | -0.03 ± 0.00 | t -6.7 | 5/3 |
| g_demo | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 시간 s | -5.74 ± 3.15 | t -1.8 | 2/2 |
| g_demo | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 달린 길 m | 0.16 ± 0.25 | t 0.6 | 3/4 |
| g_demo | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 몸 최소 거리 m | 0.00 ± 0.00 | - | 3/4 |
| g_demo | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 도리도리 /분 | 48.24 ± 13.76 | t 3.5 | 3/4 |
| g_demo | FollowPathRLShimDemoHistMask - FollowPathRLShimDemoHistFill | 주행 vx m/s | 0.06 ± 0.02 | t 4.0 | 3/4 |

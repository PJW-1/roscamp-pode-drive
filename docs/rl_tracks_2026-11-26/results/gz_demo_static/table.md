| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| b_static | FollowPathGoal | 통로 | 0/5 | - | 0.451 ± 0.009 | 0.062 ± 0.011 | 0.078 ± 0.008 | 4 ± 1 (6.8/분, 9.76/m) | 0 (0.0/분, 0.00/m) | 15 (4.6/분, 6.65/m) | 10 ± 0 | 52.0 ± 0.0 |
| b_static | FollowPathRLShimACD3W50 | 통로 | 3/5 | 13.6 ± 1.1 | 1.002 ± 0.221 | 0.050 ± 0.000 | 0.009 ± 0.009 | 9 ± 3 (22.1/분, 8.58/m) | 3 (1.5/분, 0.60/m) | 4 (2.1/분, 0.80/m) | 10 ± 0 | 60.0 ± 0.0 |
| b_static | FollowPathRLShimACD3 | 통로 | 4/5 | 13.8 ± 1.2 | 1.087 ± 0.189 | 0.050 ± 0.000 | 0.013 ± 0.008 | 9 ± 3 (31.0/분, 8.65/m) | 1 (0.7/분, 0.18/m) | 2 (1.3/분, 0.37/m) | 10 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathGoal | 통로 | 0/5 | - | 0.228 ± 0.026 | 0.056 ± 0.007 | 0.051 ± 0.018 | 1 ± 1 (2.3/분, 6.14/m) | 0 (0.0/분, 0.00/m) | 15 (4.9/분, 13.15/m) | 10 ± 0 | 52.0 ± 0.0 |
| f_cart | FollowPathRLShimACD3W50 | 통로 | 5/5 | 13.7 ± 0.4 | 1.152 ± 0.007 | 0.051 ± 0.002 | 0.046 ± 0.004 | 10 ± 2 (42.9/분, 8.51/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimACD3 | 통로 | 5/5 | 13.4 ± 0.8 | 1.159 ± 0.006 | 0.050 ± 0.001 | 0.048 ± 0.002 | 11 ± 1 (47.4/분, 9.14/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathGoal | 통로 | 0/5 | - | 0.070 ± 0.017 | 0.050 ± 0.000 | 0.050 ± 0.006 | 1 ± 1 (1.4/분, 11.35/m) | 0 (0.0/분, 0.00/m) | 15 (5.1/분, 42.55/m) | 10 ± 0 | 52.0 ± 0.0 |
| g_demo | FollowPathRLShimACD3W50 | 통로 | 4/4 | 13.7 ± 2.3 | 1.175 ± 0.010 | 0.050 ± 0.000 | 0.001 ± 0.002 | 12 ± 2 (53.7/분, 10.43/m) | 6 (6.6/분, 1.28/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimACD3W50 | 밖 | 0/1 | - | 1.320 ± 0.000 | 0.050 ± 0.000 | 0.003 ± 0.000 | 22 ± 0 (18.8/분, 16.67/m) | 1 (0.9/분, 0.76/m) | 3 (2.6/분, 2.27/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimACD3 | 통로 | 4/5 | 13.6 ± 1.3 | 1.025 ± 0.325 | 0.050 ± 0.000 | 0.000 ± 0.001 | 13 ± 5 (43.9/분, 12.68/m) | 6 (4.1/분, 1.17/m) | 2 (1.4/분, 0.39/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| b_static | FollowPathRLShimACD3W50 | 통로 | 0.0 ± 0.0 | 2.7 ± 0.6 | 0.5 ± 0.1 | 10.3 ± 0.5 | 0.113 ± 0.004 |
| b_static | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 2.6 ± 0.5 | 0.5 ± 0.2 | 10.6 ± 0.6 | 0.110 ± 0.006 |
| f_cart | FollowPathRLShimACD3W50 | 통로 | 0.0 ± 0.0 | 2.8 ± 0.3 | 0.6 ± 0.2 | 10.4 ± 0.2 | 0.111 ± 0.002 |
| f_cart | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 2.6 ± 0.4 | 0.5 ± 0.2 | 10.3 ± 0.2 | 0.112 ± 0.002 |
| g_demo | FollowPathRLShimACD3W50 | 통로 | 0.0 ± 0.0 | 1.7 ± 1.1 | 0.5 ± 0.4 | 11.5 ± 0.8 | 0.097 ± 0.002 |
| g_demo | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 1.8 ± 0.9 | 0.5 ± 0.2 | 11.4 ± 0.4 | 0.100 ± 0.004 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| b_static | FollowPathGoal | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 1 | 0 | 0 |
| b_static | FollowPathRLShimACD3W50 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| b_static | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| f_cart | FollowPathGoal | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 1 | 0 | 0 |
| f_cart | FollowPathRLShimACD3W50 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| f_cart | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| g_demo | FollowPathGoal | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 1 | 0 | 0 |
| g_demo | FollowPathRLShimACD3W50 | 4/5 | 4/5 | 4/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| g_demo | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| b_static | FollowPathRLShimACD3 - RPP | 달린 길 m | 0.64 ± 0.08 | t 7.5 | 5/5 |
| b_static | FollowPathRLShimACD3 - RPP | 몸 최소 거리 m | -0.06 ± 0.00 | t -13.1 | 5/5 |
| b_static | FollowPathRLShimACD3 - RPP | 도리도리 /분 | 30.02 ± 6.99 | t 4.3 | 5/5 |
| b_static | FollowPathRLShimACD3 - RPP | 주행 vx m/s | -0.02 ± 0.01 | t -1.8 | 5/5 |
| b_static | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 시간 s | 0.22 ± 0.88 | t 0.3 | 4/3 |
| b_static | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 달린 길 m | 0.08 ± 0.13 | t 0.6 | 5/5 |
| b_static | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 몸 최소 거리 m | 0.00 ± 0.01 | t 0.8 | 5/5 |
| b_static | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 도리도리 /분 | 4.34 ± 12.29 | t 0.4 | 5/5 |
| b_static | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 주행 vx m/s | 0.01 ± 0.02 | t 0.5 | 5/5 |
| b_static | FollowPathRLShimACD3W50 - RPP | 달린 길 m | 0.55 ± 0.10 | t 5.6 | 5/5 |
| b_static | FollowPathRLShimACD3W50 - RPP | 몸 최소 거리 m | -0.07 ± 0.01 | t -12.9 | 5/5 |
| b_static | FollowPathRLShimACD3W50 - RPP | 도리도리 /분 | 25.68 ± 10.15 | t 2.5 | 5/5 |
| b_static | FollowPathRLShimACD3W50 - RPP | 주행 vx m/s | -0.03 ± 0.01 | t -2.1 | 5/5 |
| f_cart | FollowPathRLShimACD3 - RPP | 달린 길 m | 0.93 ± 0.01 | t 77.4 | 5/5 |
| f_cart | FollowPathRLShimACD3 - RPP | 몸 최소 거리 m | -0.00 ± 0.01 | t -0.4 | 5/5 |
| f_cart | FollowPathRLShimACD3 - RPP | 도리도리 /분 | 45.15 ± 1.58 | t 28.6 | 5/5 |
| f_cart | FollowPathRLShimACD3 - RPP | 주행 vx m/s | -0.01 ± 0.00 | t -5.4 | 5/5 |
| f_cart | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 시간 s | -0.30 ± 0.41 | t -0.7 | 5/5 |
| f_cart | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 달린 길 m | 0.01 ± 0.00 | t 1.9 | 5/5 |
| f_cart | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 몸 최소 거리 m | 0.00 ± 0.00 | t 0.7 | 5/5 |
| f_cart | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 도리도리 /분 | 4.70 ± 3.33 | t 1.4 | 5/5 |
| f_cart | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 주행 vx m/s | 0.00 ± 0.00 | t 0.7 | 5/5 |
| f_cart | FollowPathRLShimACD3W50 - RPP | 달린 길 m | 0.92 ± 0.01 | t 76.2 | 5/5 |
| f_cart | FollowPathRLShimACD3W50 - RPP | 몸 최소 거리 m | -0.00 ± 0.01 | t -0.5 | 5/5 |
| f_cart | FollowPathRLShimACD3W50 - RPP | 도리도리 /분 | 40.45 ± 3.06 | t 13.2 | 5/5 |
| f_cart | FollowPathRLShimACD3W50 - RPP | 주행 vx m/s | -0.01 ± 0.00 | t -7.0 | 5/5 |
| g_demo | FollowPathRLShimACD3 - RPP | 달린 길 m | 0.95 ± 0.15 | t 6.6 | 5/5 |
| g_demo | FollowPathRLShimACD3 - RPP | 몸 최소 거리 m | -0.05 ± 0.00 | t -18.1 | 5/5 |
| g_demo | FollowPathRLShimACD3 - RPP | 도리도리 /분 | 53.06 ± 12.67 | t 4.2 | 5/5 |
| g_demo | FollowPathRLShimACD3 - RPP | 주행 vx m/s | -0.03 ± 0.01 | t -2.5 | 5/5 |
| g_demo | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 시간 s | -0.07 ± 1.32 | t -0.1 | 4/4 |
| g_demo | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 달린 길 m | -0.15 ± 0.15 | t -1.0 | 5/4 |
| g_demo | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 몸 최소 거리 m | -0.00 ± 0.00 | t -0.5 | 5/4 |
| g_demo | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 도리도리 /분 | 0.68 ± 12.96 | t 0.1 | 5/4 |
| g_demo | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 주행 vx m/s | -0.01 ± 0.01 | t -0.8 | 5/4 |
| g_demo | FollowPathRLShimACD3W50 - RPP | 달린 길 m | 1.10 ± 0.01 | t 121.1 | 4/5 |
| g_demo | FollowPathRLShimACD3W50 - RPP | 몸 최소 거리 m | -0.05 ± 0.00 | t -17.4 | 4/5 |
| g_demo | FollowPathRLShimACD3W50 - RPP | 도리도리 /분 | 52.38 ± 2.87 | t 18.2 | 4/5 |
| g_demo | FollowPathRLShimACD3W50 - RPP | 주행 vx m/s | -0.02 ± 0.00 | t -22.3 | 4/5 |

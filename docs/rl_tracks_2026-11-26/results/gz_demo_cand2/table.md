| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimACD3 | 통로 | 5/5 | 13.3 ± 0.3 | 1.161 ± 0.005 | 0.050 ± 0.001 | 0.049 ± 0.002 | 11 ± 3 (49.7/분, 9.47/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimDemoHist2 | 통로 | 5/5 | 12.8 ± 0.4 | 1.147 ± 0.013 | 0.054 ± 0.005 | 0.049 ± 0.005 | 11 ± 1 (51.4/분, 9.59/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 5/5 | 13.0 ± 0.1 | 1.163 ± 0.011 | 0.052 ± 0.003 | 0.049 ± 0.005 | 10 ± 1 (47.0/분, 8.77/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimStd | 통로 | 5/5 | 16.9 ± 6.3 | 1.310 ± 0.290 | 0.050 ± 0.000 | 0.040 ± 0.012 | 11 ± 4 (39.1/분, 8.40/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimACD3 | 통로 | 2/5 | 17.2 ± 6.3 | 0.749 ± 0.454 | 0.050 ± 0.000 | 0.000 ± 0.000 | 8 ± 7 (18.0/분, 10.94/m) | 7 (3.1/분, 1.87/m) | 6 (2.6/분, 1.60/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist2 | 통로 | 5/5 | 14.6 ± 3.2 | 1.278 ± 0.149 | 0.050 ± 0.000 | 0.006 ± 0.010 | 11 ± 3 (45.1/분, 8.61/m) | 4 (3.3/분, 0.63/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 58.4 ± 3.6 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 1/4 | 21.0 ± 0.0 | 1.087 ± 0.378 | 0.050 ± 0.000 | 0.005 ± 0.009 | 10 ± 5 (16.9/분, 8.97/m) | 11 (4.8/분, 2.53/m) | 6 (2.6/분, 1.38/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 밖 | 0/1 | - | 1.811 ± 0.000 | 0.050 ± 0.000 | 0.000 ± 0.000 | 15 ± 0 (15.6/분, 8.28/m) | 2 (2.1/분, 1.10/m) | 2 (2.1/분, 1.10/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimStd | 통로 | 2/3 | 20.1 ± 0.2 | 1.074 ± 0.561 | 0.050 ± 0.000 | 0.000 ± 0.000 | 12 ± 8 (28.7/분, 10.86/m) | 3 (2.5/분, 0.93/m) | 2 (1.6/분, 0.62/m) | 10 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimStd | 밖 | 0/2 | - | 1.374 ± 0.334 | 0.050 ± 0.000 | 0.000 ± 0.000 | 12 ± 2 (20.4/분, 9.10/m) | 9 (7.3/분, 3.27/m) | 3 (2.4/분, 1.09/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimACD3 | 통로 | 5/5 | 18.6 ± 3.7 | 1.247 ± 0.066 | 0.050 ± 0.000 | 0.000 ± 0.000 | 14 ± 3 (45.8/분, 11.39/m) | 11 (7.1/분, 1.76/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHist2 | 통로 | 5/5 | 14.1 ± 2.3 | 1.167 ± 0.043 | 0.050 ± 0.000 | 0.000 ± 0.000 | 15 ± 4 (62.3/분, 12.51/m) | 5 (4.3/분, 0.86/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHist | 통로 | 3/5 | 13.8 ± 1.8 | 1.102 ± 0.133 | 0.050 ± 0.000 | 0.000 ± 0.000 | 11 ± 2 (28.4/분, 10.35/m) | 8 (4.0/분, 1.45/m) | 4 (2.0/분, 0.73/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimStd | 통로 | 3/5 | 19.4 ± 5.1 | 1.366 ± 0.284 | 0.050 ± 0.000 | 0.000 ± 0.000 | 15 ± 3 (35.5/분, 10.69/m) | 12 (5.8/분, 1.76/m) | 3 (1.5/분, 0.44/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimACD3 | 통로 | 3/4 | 14.6 ± 0.1 | 1.189 ± 0.017 | 0.050 ± 0.000 | 0.000 ± 0.000 | 14 ± 2 (57.7/분, 11.78/m) | 8 (8.2/분, 1.68/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimACD3 | 밖 | 0/1 | - | 1.147 ± 0.000 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 0 (18.8/분, 11.34/m) | 9 (13.0/분, 7.85/m) | 2 (2.9/분, 1.74/m) | 9 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHist2 | 통로 | 3/5 | 21.1 ± 5.8 | 1.300 ± 0.411 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 3 (28.1/분, 10.15/m) | 24 (10.2/분, 3.69/m) | 4 (1.7/분, 0.62/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHist | 통로 | 2/5 | 15.2 ± 0.5 | 1.051 ± 0.208 | 0.050 ± 0.000 | 0.000 ± 0.000 | 9 ± 2 (18.4/분, 8.56/m) | 23 (9.4/분, 4.38/m) | 6 (2.5/분, 1.14/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimStd | 통로 | 5/5 | 20.6 ± 6.9 | 1.415 ± 0.231 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 5 (38.5/분, 9.33/m) | 18 (10.5/분, 2.54/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 2.4 ± 0.3 | 0.5 ± 0.2 | 10.3 ± 0.1 | 0.112 ± 0.001 |
| f_cart | FollowPathRLShimDemoHist2 | 통로 | 0.0 ± 0.0 | 2.6 ± 0.3 | 0.7 ± 0.1 | 9.6 ± 0.1 | 0.120 ± 0.000 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.6 ± 0.1 | 0.6 ± 0.1 | 9.8 ± 0.1 | 0.119 ± 0.000 |
| f_cart | FollowPathRLShimStd | 통로 | 0.0 ± 0.0 | 3.7 ± 1.2 | 0.8 ± 0.4 | 12.3 ± 4.7 | 0.104 ± 0.025 |
| g_demo | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 3.2 ± 2.2 | 0.6 ± 0.4 | 13.3 ± 3.7 | 0.089 ± 0.018 |
| g_demo | FollowPathRLShimDemoHist2 | 통로 | 0.0 ± 0.0 | 2.2 ± 1.2 | 0.4 ± 0.3 | 12.1 ± 2.6 | 0.097 ± 0.018 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 4.5 ± 0.0 | 1.4 ± 0.0 | 15.0 ± 0.0 | 0.074 ± 0.000 |
| g_demo | FollowPathRLShimStd | 통로 | 0.0 ± 0.0 | 4.1 ± 0.1 | 1.0 ± 0.1 | 14.9 ± 0.0 | 0.080 ± 0.004 |
| j_cross | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 4.1 ± 1.1 | 1.0 ± 0.8 | 13.5 ± 2.2 | 0.094 ± 0.014 |
| j_cross | FollowPathRLShimDemoHist2 | 통로 | 0.0 ± 0.0 | 2.6 ± 0.7 | 0.7 ± 0.7 | 10.8 ± 1.0 | 0.116 ± 0.007 |
| j_cross | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.3 ± 0.6 | 0.4 ± 0.2 | 11.1 ± 1.0 | 0.116 ± 0.003 |
| j_cross | FollowPathRLShimStd | 통로 | 0.0 ± 0.0 | 3.3 ± 0.7 | 0.8 ± 0.5 | 15.3 ± 4.0 | 0.091 ± 0.016 |
| k_cross | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 2.1 ± 0.2 | 0.3 ± 0.1 | 12.1 ± 0.1 | 0.102 ± 0.001 |
| k_cross | FollowPathRLShimDemoHist2 | 통로 | 0.0 ± 0.0 | 4.6 ± 3.3 | 1.0 ± 0.8 | 15.5 ± 2.0 | 0.089 ± 0.001 |
| k_cross | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.3 ± 0.1 | 0.2 ± 0.0 | 12.6 ± 0.3 | 0.102 ± 0.001 |
| k_cross | FollowPathRLShimStd | 통로 | 0.0 ± 0.0 | 3.3 ± 1.1 | 0.9 ± 0.7 | 16.3 ± 5.4 | 0.079 ± 0.023 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| f_cart | FollowPathRLShimDemoHist2 | 5/5 | 5/5 | 5/5 | 0.006 ± 0.013, 0.028 | 2.2 | 4 | 0 | 0 |
| f_cart | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| f_cart | FollowPathRLShimStd | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| g_demo | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHist2 | 5/5 | 5/5 | 5/5 | 0.006 ± 0.013, 0.028 | 0.6 | 1 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHist | 4/5 | 4/5 | 4/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| g_demo | FollowPathRLShimStd | 3/5 | 4/5 | 3/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| j_cross | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| j_cross | FollowPathRLShimDemoHist2 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| j_cross | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| j_cross | FollowPathRLShimStd | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| k_cross | FollowPathRLShimACD3 | 4/5 | 4/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| k_cross | FollowPathRLShimDemoHist2 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| k_cross | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| k_cross | FollowPathRLShimStd | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| f_cart | FollowPathRLShimStd - FollowPathRLShimACD3 | 시간 s | 3.60 ± 2.82 | t 1.3 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimACD3 | 달린 길 m | 0.15 ± 0.13 | t 1.1 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimACD3 | 몸 최소 거리 m | -0.01 ± 0.01 | t -1.7 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimACD3 | 도리도리 /분 | -10.93 ± 6.34 | t -1.7 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimACD3 | 주행 vx m/s | -0.01 ± 0.01 | t -0.7 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 시간 s | 4.03 ± 2.83 | t 1.4 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 달린 길 m | 0.16 ± 0.13 | t 1.3 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | -0.01 ± 0.01 | t -1.6 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 도리도리 /분 | -12.65 ± 3.34 | t -3.8 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.02 ± 0.01 | t -1.4 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist | 시간 s | 3.86 ± 2.82 | t 1.4 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist | 달린 길 m | 0.15 ± 0.13 | t 1.1 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist | 몸 최소 거리 m | -0.01 ± 0.01 | t -1.6 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist | 도리도리 /분 | -8.18 ± 2.22 | t -3.7 | 5/5 |
| f_cart | FollowPathRLShimStd - FollowPathRLShimDemoHist | 주행 vx m/s | -0.01 ± 0.01 | t -1.3 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 시간 s | -0.26 ± 0.17 | t -1.5 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 달린 길 m | 0.00 ± 0.01 | t 0.3 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 몸 최소 거리 m | -0.00 ± 0.00 | t -0.0 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 도리도리 /분 | -2.75 ± 6.68 | t -0.4 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 주행 vx m/s | 0.01 ± 0.00 | t 18.6 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 시간 s | 0.17 ± 0.19 | t 0.9 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 달린 길 m | 0.02 ± 0.01 | t 2.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | -0.00 ± 0.00 | t -0.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 도리도리 /분 | -4.47 ± 3.96 | t -1.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.00 ± 0.00 | t -11.7 | 5/5 |
| f_cart | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 시간 s | -0.43 ± 0.24 | t -1.8 | 5/5 |
| f_cart | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 달린 길 m | -0.01 ± 0.01 | t -2.3 | 5/5 |
| f_cart | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | t 0.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 도리도리 /분 | 1.73 ± 7.14 | t 0.2 | 5/5 |
| f_cart | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 주행 vx m/s | 0.01 ± 0.00 | t 20.4 | 5/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimACD3 | 시간 s | 2.91 ± 4.46 | t 0.7 | 2/2 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimACD3 | 달린 길 m | 0.32 ± 0.38 | t 0.8 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimACD3 | 도리도리 /분 | 8.31 ± 18.57 | t 0.4 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimACD3 | 주행 vx m/s | 0.01 ± 0.02 | t 0.3 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 시간 s | 5.45 ± 1.45 | t 3.8 | 2/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 달린 길 m | -0.20 ± 0.33 | t -0.6 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | -0.01 ± 0.00 | t -1.3 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 도리도리 /분 | -12.01 ± 14.74 | t -0.8 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.03 ± 0.02 | t -1.9 | 3/5 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist | 달린 길 m | -0.01 ± 0.37 | t -0.0 | 3/4 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist | 몸 최소 거리 m | -0.00 ± 0.00 | t -1.0 | 3/4 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist | 도리도리 /분 | 14.35 ± 15.98 | t 0.9 | 3/4 |
| g_demo | FollowPathRLShimStd - FollowPathRLShimDemoHist | 주행 vx m/s | 0.01 ± 0.02 | t 0.5 | 3/4 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 달린 길 m | 0.34 ± 0.28 | t 1.2 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | t 1.0 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 도리도리 /분 | -6.04 ± 14.08 | t -0.4 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 주행 vx m/s | -0.00 ± 0.02 | t -0.2 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 달린 길 m | -0.19 ± 0.20 | t -1.0 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | -0.00 ± 0.01 | t -0.2 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 도리도리 /분 | -26.36 ± 8.41 | t -3.1 | 4/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.04 ± 0.01 | t -2.8 | 4/5 |
| g_demo | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 시간 s | -2.55 ± 4.69 | t -0.5 | 5/2 |
| g_demo | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 달린 길 m | 0.53 ± 0.21 | t 2.5 | 5/5 |
| g_demo | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.01 ± 0.00 | t 1.3 | 5/5 |
| g_demo | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 도리도리 /분 | 20.32 ± 12.65 | t 1.6 | 5/5 |
| g_demo | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 주행 vx m/s | 0.04 ± 0.02 | t 2.4 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 시간 s | 0.75 ± 3.37 | t 0.2 | 3/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 달린 길 m | 0.12 ± 0.13 | t 0.9 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 도리도리 /분 | -7.42 ± 4.83 | t -1.5 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 주행 vx m/s | -0.01 ± 0.01 | t -1.1 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 시간 s | 5.31 ± 3.12 | t 1.7 | 3/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 달린 길 m | 0.20 ± 0.13 | t 1.5 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 도리도리 /분 | -23.20 ± 5.62 | t -4.1 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.03 ± 0.01 | t -3.8 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 시간 s | 5.55 ± 3.12 | t 1.8 | 3/3 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 달린 길 m | 0.26 ± 0.14 | t 1.9 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 도리도리 /분 | 0.20 ± 10.43 | t 0.0 | 5/5 |
| j_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 주행 vx m/s | -0.03 ± 0.01 | t -2.7 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 시간 s | -4.80 ± 1.93 | t -2.5 | 3/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 달린 길 m | -0.14 ± 0.07 | t -2.2 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 도리도리 /분 | -7.62 ± 9.97 | t -0.8 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 주행 vx m/s | 0.01 ± 0.01 | t 1.7 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 시간 s | -0.25 ± 1.46 | t -0.2 | 3/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 달린 길 m | -0.06 ± 0.06 | t -1.0 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 도리도리 /분 | -23.40 ± 10.38 | t -2.3 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.01 ± 0.01 | t -1.3 | 5/5 |
| j_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 시간 s | -4.55 ± 1.94 | t -2.3 | 5/5 |
| j_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 달린 길 m | -0.08 ± 0.04 | t -2.3 | 5/5 |
| j_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 도리도리 /분 | 15.78 ± 4.71 | t 3.4 | 5/5 |
| j_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 주행 vx m/s | 0.02 ± 0.01 | t 3.1 | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 시간 s | 6.00 ± 3.07 | t 2.0 | 5/3 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 달린 길 m | 0.23 ± 0.10 | t 2.2 | 5/4 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/4 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 도리도리 /분 | -19.12 ± 4.95 | t -3.9 | 5/4 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimACD3 | 주행 vx m/s | -0.02 ± 0.01 | t -2.2 | 5/4 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 시간 s | -0.49 ± 4.55 | t -0.1 | 5/3 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 달린 길 m | 0.11 ± 0.21 | t 0.5 | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 도리도리 /분 | 4.76 ± 9.20 | t 0.5 | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist2 | 주행 vx m/s | -0.01 ± 0.01 | t -0.7 | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 시간 s | 5.38 ± 3.09 | t 1.7 | 5/2 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 달린 길 m | 0.36 ± 0.14 | t 2.6 | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 도리도리 /분 | 15.18 ± 7.05 | t 2.2 | 5/5 |
| k_cross | FollowPathRLShimStd - FollowPathRLShimDemoHist | 주행 vx m/s | -0.01 ± 0.01 | t -0.8 | 5/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 시간 s | 0.62 ± 0.36 | t 1.7 | 2/3 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 달린 길 m | -0.14 ± 0.09 | t -1.5 | 5/4 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/4 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 도리도리 /분 | -34.31 ± 7.84 | t -4.4 | 5/4 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimACD3 | 주행 vx m/s | -0.01 ± 0.01 | t -1.5 | 5/4 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 시간 s | -5.87 ± 3.38 | t -1.7 | 2/3 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 달린 길 m | -0.25 ± 0.21 | t -1.2 | 5/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 도리도리 /분 | -10.42 ± 11.02 | t -0.9 | 5/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHist2 | 주행 vx m/s | 0.00 ± 0.01 | t 0.4 | 5/5 |
| k_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 시간 s | 6.49 ± 3.36 | t 1.9 | 3/3 |
| k_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 달린 길 m | 0.11 ± 0.18 | t 0.6 | 5/4 |
| k_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/4 |
| k_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 도리도리 /분 | -23.88 ± 9.82 | t -2.4 | 5/4 |
| k_cross | FollowPathRLShimDemoHist2 - FollowPathRLShimACD3 | 주행 vx m/s | -0.02 ± 0.00 | t -6.7 | 5/4 |

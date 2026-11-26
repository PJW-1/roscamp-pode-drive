| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistStop | 통로 | 5/5 | 12.9 ± 0.3 | 1.150 ± 0.006 | 0.053 ± 0.004 | 0.048 ± 0.002 | 11 ± 2 (49.4/분, 9.22/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 5/5 | 12.9 ± 0.2 | 1.163 ± 0.013 | 0.050 ± 0.000 | 0.046 ± 0.004 | 10 ± 2 (48.4/분, 8.95/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHistStop | 통로 | 2/3 | 35.7 ± 9.2 | 1.327 ± 0.446 | 0.050 ± 0.000 | 0.000 ± 0.000 | 28 ± 7 (42.4/분, 20.85/m) | 6 (3.1/분, 1.51/m) | 3 (1.5/분, 0.75/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHistStop | 밖 | 1/2 | 36.7 ± 0.0 | 1.527 ± 0.025 | 0.050 ± 0.000 | 0.000 ± 0.000 | 38 ± 11 (39.2/분, 24.88/m) | 12 (6.2/분, 3.93/m) | 6 (3.1/분, 1.96/m) | 11 ± 0 | 60.0 ± 0.0 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 4/5 | 20.3 ± 1.8 | 1.310 ± 0.322 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 3 (31.8/분, 9.77/m) | 8 (4.0/분, 1.22/m) | 3 (1.5/분, 0.46/m) | 11 ± 1 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHistStop | 통로 | 1/1 | 34.2 ± 0.0 | 1.492 ± 0.000 | 0.050 ± 0.000 | 0.000 ± 0.000 | 24 ± 0 (42.1/분, 16.09/m) | 6 (10.5/분, 4.02/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHistStop | 밖 | 4/4 | 41.8 ± 10.0 | 1.678 ± 0.200 | 0.050 ± 0.000 | 0.000 ± 0.000 | 36 ± 12 (52.0/분, 21.60/m) | 15 (5.4/분, 2.23/m) | 4 (1.4/분, 0.60/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHist | 통로 | 5/5 | 16.0 ± 2.0 | 1.274 ± 0.074 | 0.050 ± 0.000 | 0.000 ± 0.000 | 14 ± 3 (52.4/분, 10.99/m) | 6 (4.5/분, 0.94/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHistStop | 밖 | 3/5 | 59.1 ± 22.9 | 1.638 ± 0.168 | 0.050 ± 0.000 | 0.000 ± 0.001 | 60 ± 21 (50.2/분, 36.76/m) | 12 (2.0/분, 1.47/m) | 20 (3.3/분, 2.44/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHist | 통로 | 3/5 | 16.4 ± 3.5 | 1.318 ± 0.270 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 2 (28.6/분, 9.87/m) | 19 (8.4/분, 2.88/m) | 4 (1.8/분, 0.61/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistStop | 통로 | 0.0 ± 0.0 | 2.6 ± 0.1 | 0.6 ± 0.2 | 9.7 ± 0.1 | 0.118 ± 0.001 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.5 ± 0.2 | 0.5 ± 0.2 | 9.8 ± 0.2 | 0.118 ± 0.002 |
| g_demo | FollowPathRLShimDemoHistStop | 통로 | 0.0 ± 0.0 | 13.1 ± 5.1 | 4.8 ± 0.6 | 17.7 ± 3.5 | 0.064 ± 0.015 |
| g_demo | FollowPathRLShimDemoHistStop | 밖 | 0.0 ± 0.0 | 14.3 ± 0.0 | 5.0 ± 0.0 | 17.4 ± 0.0 | 0.062 ± 0.000 |
| g_demo | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 4.5 ± 0.5 | 1.1 ± 0.3 | 14.7 ± 1.5 | 0.081 ± 0.008 |
| j_cross | FollowPathRLShimDemoHistStop | 통로 | 0.0 ± 0.0 | 12.0 ± 0.0 | 6.4 ± 0.0 | 15.7 ± 0.0 | 0.073 ± 0.000 |
| j_cross | FollowPathRLShimDemoHistStop | 밖 | 0.0 ± 0.0 | 15.5 ± 5.0 | 7.0 ± 2.8 | 19.3 ± 3.3 | 0.055 ± 0.013 |
| j_cross | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 3.2 ± 0.9 | 0.5 ± 0.4 | 12.3 ± 0.9 | 0.110 ± 0.007 |
| k_cross | FollowPathRLShimDemoHistStop | 밖 | 0.0 ± 0.0 | 25.5 ± 10.1 | 13.4 ± 8.9 | 20.2 ± 4.7 | 0.061 ± 0.012 |
| k_cross | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 3.0 ± 0.8 | 0.3 ± 0.1 | 13.2 ± 2.7 | 0.098 ± 0.010 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistStop | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| f_cart | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHistStop | 3/5 | 3/5 | 5/5 | 0.001 ± 0.003, 0.007 | 3.3 | 3 | 0 | 0 |
| g_demo | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| j_cross | FollowPathRLShimDemoHistStop | 1/5 | 1/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| j_cross | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| k_cross | FollowPathRLShimDemoHistStop | 0/5 | 0/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 3 | 0 | 0 |
| k_cross | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 시간 s | 0.01 ± 0.14 | t 0.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 달린 길 m | 0.01 ± 0.01 | t 2.0 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 몸 최소 거리 m | -0.00 ± 0.00 | t -1.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 도리도리 /분 | -0.97 ± 4.86 | t -0.2 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 주행 vx m/s | -0.00 ± 0.00 | t -0.6 | 5/5 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 시간 s | -15.42 ± 6.58 | t -2.3 | 4/2 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 달린 길 m | -0.02 ± 0.30 | t -0.1 | 5/3 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/3 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 도리도리 /분 | -9.68 ± 12.93 | t -0.7 | 5/3 |
| g_demo | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 주행 vx m/s | 0.03 ± 0.02 | t 1.7 | 5/3 |

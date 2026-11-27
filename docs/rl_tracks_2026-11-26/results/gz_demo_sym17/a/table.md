| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistSym | 통로 | 5/5 | 13.0 ± 0.6 | 1.159 ± 0.006 | 0.052 ± 0.003 | 0.048 ± 0.001 | 10 ± 1 (47.2/분, 8.80/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 5/5 | 12.7 ± 0.5 | 1.144 ± 0.003 | 0.051 ± 0.001 | 0.045 ± 0.003 | 12 ± 1 (56.5/분, 10.49/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHistSym | 통로 | 2/5 | 13.4 ± 0.3 | 1.062 ± 0.142 | 0.050 ± 0.000 | 0.000 ± 0.000 | 10 ± 2 (20.6/분, 9.42/m) | 17 (7.0/분, 3.20/m) | 6 (2.5/분, 1.13/m) | 10 ± 0 | 60.0 ± 0.0 |
| j_cross | FollowPathRLShimDemoHist | 통로 | 4/5 | 16.0 ± 0.9 | 1.228 ± 0.100 | 0.050 ± 0.000 | 0.000 ± 0.000 | 14 ± 2 (55.6/분, 11.57/m) | 7 (5.5/분, 1.14/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHistSym | 통로 | 1/5 | 16.5 ± 0.0 | 0.860 ± 0.232 | 0.050 ± 0.000 | 0.000 ± 0.000 | 7 ± 2 (13.1/분, 7.91/m) | 29 (11.2/분, 6.75/m) | 8 (3.1/분, 1.86/m) | 11 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHist | 통로 | 4/4 | 15.5 ± 2.3 | 1.239 ± 0.052 | 0.050 ± 0.000 | 0.000 ± 0.000 | 14 ± 2 (52.2/분, 10.89/m) | 11 (10.6/분, 2.22/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| k_cross | FollowPathRLShimDemoHist | 밖 | 0/1 | - | 1.018 ± 0.000 | 0.050 ± 0.000 | 0.000 ± 0.000 | 13 ± 0 (46.5/분, 12.78/m) | 8 (28.6/분, 7.86/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistSym | 통로 | 0.0 ± 0.0 | 2.7 ± 0.3 | 0.5 ± 0.2 | 9.7 ± 0.1 | 0.119 ± 0.000 |
| f_cart | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.5 ± 0.3 | 0.6 ± 0.2 | 9.6 ± 0.0 | 0.119 ± 0.000 |
| j_cross | FollowPathRLShimDemoHistSym | 통로 | 0.0 ± 0.0 | 2.9 ± 0.5 | 0.4 ± 0.2 | 10.1 ± 0.4 | 0.113 ± 0.005 |
| j_cross | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 3.3 ± 0.5 | 0.7 ± 0.6 | 11.9 ± 0.4 | 0.109 ± 0.002 |
| k_cross | FollowPathRLShimDemoHistSym | 통로 | 0.0 ± 0.0 | 2.2 ± 0.0 | 1.2 ± 0.0 | 13.1 ± 0.0 | 0.100 ± 0.000 |
| k_cross | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.8 ± 1.1 | 0.5 ± 0.3 | 12.2 ± 1.0 | 0.103 ± 0.002 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHistSym | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| f_cart | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| j_cross | FollowPathRLShimDemoHistSym | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| j_cross | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 2 | 0 | 0 |
| k_cross | FollowPathRLShimDemoHistSym | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| k_cross | FollowPathRLShimDemoHist | 4/5 | 5/5 | 4/5 | 0.000 ± 0.000, 0.000 | 0.0 | 1 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 시간 s | -0.21 ± 0.34 | t -0.6 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 달린 길 m | -0.02 ± 0.00 | t -5.1 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 몸 최소 거리 m | -0.00 ± 0.00 | t -2.0 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 도리도리 /분 | 9.50 ± 3.45 | t 2.8 | 5/5 |
| f_cart | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 주행 vx m/s | -0.00 ± 0.00 | t -0.1 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 시간 s | 2.61 ± 0.49 | t 5.3 | 4/2 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 달린 길 m | 0.17 ± 0.08 | t 2.1 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 몸 최소 거리 m | 0.00 ± 0.00 | - | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 도리도리 /분 | 27.46 ± 9.17 | t 3.0 | 5/5 |
| j_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 주행 vx m/s | 0.03 ± 0.01 | t 2.1 | 5/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 달린 길 m | 0.38 ± 0.11 | t 3.5 | 4/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 몸 최소 거리 m | 0.00 ± 0.00 | - | 4/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 도리도리 /분 | 37.21 ± 7.38 | t 5.0 | 4/5 |
| k_cross | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistSym | 주행 vx m/s | 0.01 ± 0.00 | t 2.4 | 4/5 |

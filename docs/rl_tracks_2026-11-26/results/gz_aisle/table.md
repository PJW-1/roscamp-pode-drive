| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| b_static | FollowPathGoal | 밖 | 5/5 | 24.6 ± 5.0 | 1.576 ± 0.375 | 0.063 ± 0.012 | 0.052 ± 0.018 | 16 ± 7 (38.5/분, 10.03/m) | 0 (0.0/분, 0.00/m) | 1 (0.5/분, 0.13/m) | 11 ± 0 | 52.0 ± 0.0 |
| b_static | FollowPathRLShimACD3W50 | 밖 | 5/5 | 21.4 ± 0.2 | 1.772 ± 0.013 | 0.061 ± 0.008 | 0.055 ± 0.003 | 11 ± 3 (30.3/분, 6.10/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| b_static | FollowPathRLShimACD3 | 통로 | 2/2 | 12.7 ± 0.2 | 1.155 ± 0.001 | 0.050 ± 0.000 | 0.022 ± 0.001 | 8 ± 2 (40.3/분, 7.36/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| b_static | FollowPathRLShimACD3 | 밖 | 2/3 | 27.7 ± 6.0 | 1.444 ± 0.591 | 0.057 ± 0.008 | 0.035 ± 0.030 | 17 ± 9 (33.1/분, 12.00/m) | 1 (0.6/분, 0.23/m) | 2 (1.3/분, 0.46/m) | 10 ± 0 | 60.0 ± 0.0 |
| e_corridor | FollowPathGoal | 통로 | 4/4 | 13.1 ± 0.4 | 1.126 ± 0.007 | 0.074 ± 0.010 | 0.078 ± 0.005 | 12 ± 2 (56.1/분, 10.88/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 52.0 ± 0.0 |
| e_corridor | FollowPathGoal | 밖 | 1/1 | 15.4 ± 0.0 | 1.127 ± 0.000 | 0.082 ± 0.000 | 0.083 ± 0.000 | 18 ± 0 (69.9/분, 15.98/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 52.0 ± 0.0 |
| e_corridor | FollowPathRLShimACD3W50 | 통로 | 5/5 | 12.4 ± 0.2 | 1.148 ± 0.007 | 0.073 ± 0.005 | 0.072 ± 0.003 | 9 ± 4 (41.6/분, 7.49/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |
| e_corridor | FollowPathRLShimACD3 | 통로 | 5/5 | 12.3 ± 0.3 | 1.140 ± 0.005 | 0.073 ± 0.008 | 0.072 ± 0.001 | 14 ± 2 (67.1/분, 12.10/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 10 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| b_static | FollowPathGoal | 밖 | 0.1 ± 0.1 | 7.0 ± 2.6 | 3.2 ± 4.8 | 14.2 ± 3.3 | 0.111 ± 0.001 |
| b_static | FollowPathRLShimACD3W50 | 밖 | 0.0 ± 0.0 | 5.8 ± 0.1 | 0.6 ± 0.1 | 14.9 ± 0.2 | 0.119 ± 0.001 |
| b_static | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 2.1 ± 0.0 | 0.4 ± 0.3 | 10.1 ± 0.0 | 0.114 ± 0.000 |
| b_static | FollowPathRLShimACD3 | 밖 | 0.0 ± 0.0 | 9.6 ± 2.8 | 2.6 ± 2.0 | 15.5 ± 1.2 | 0.114 ± 0.007 |
| e_corridor | FollowPathGoal | 통로 | 0.0 ± 0.0 | 2.4 ± 0.1 | 0.4 ± 0.2 | 10.3 ± 0.1 | 0.110 ± 0.001 |
| e_corridor | FollowPathGoal | 밖 | 0.0 ± 0.0 | 4.4 ± 0.0 | 0.7 ± 0.0 | 10.3 ± 0.0 | 0.109 ± 0.000 |
| e_corridor | FollowPathRLShimACD3W50 | 통로 | 0.0 ± 0.0 | 2.2 ± 0.1 | 0.4 ± 0.2 | 9.8 ± 0.1 | 0.117 ± 0.001 |
| e_corridor | FollowPathRLShimACD3 | 통로 | 0.0 ± 0.0 | 2.2 ± 0.1 | 0.4 ± 0.2 | 9.7 ± 0.1 | 0.118 ± 0.001 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| b_static | FollowPathGoal | 0/5 | 0/5 | 2/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| b_static | FollowPathRLShimACD3W50 | 0/5 | 0/5 | 0/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| b_static | FollowPathRLShimACD3 | 2/5 | 2/5 | 3/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| e_corridor | FollowPathGoal | 4/5 | 4/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| e_corridor | FollowPathRLShimACD3W50 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| e_corridor | FollowPathRLShimACD3 | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| e_corridor | FollowPathRLShimACD3 - RPP | 시간 s | -0.75 ± 0.23 | t -3.2 | 5/4 |
| e_corridor | FollowPathRLShimACD3 - RPP | 달린 길 m | 0.01 ± 0.00 | t 3.6 | 5/4 |
| e_corridor | FollowPathRLShimACD3 - RPP | 몸 최소 거리 m | -0.01 ± 0.00 | t -2.4 | 5/4 |
| e_corridor | FollowPathRLShimACD3 - RPP | 도리도리 /분 | 10.94 ± 6.47 | t 1.7 | 5/4 |
| e_corridor | FollowPathRLShimACD3 - RPP | 주행 vx m/s | 0.01 ± 0.00 | t 15.6 | 5/4 |
| e_corridor | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 시간 s | -0.05 ± 0.17 | t -0.3 | 5/5 |
| e_corridor | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 달린 길 m | -0.01 ± 0.00 | t -1.9 | 5/5 |
| e_corridor | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 몸 최소 거리 m | 0.00 ± 0.00 | t 0.0 | 5/5 |
| e_corridor | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 도리도리 /분 | 25.39 ± 8.94 | t 2.8 | 5/5 |
| e_corridor | FollowPathRLShimACD3 - FollowPathRLShimACD3W50 | 주행 vx m/s | 0.00 ± 0.00 | t 0.6 | 5/5 |
| e_corridor | FollowPathRLShimACD3W50 - RPP | 시간 s | -0.70 ± 0.20 | t -3.5 | 5/4 |
| e_corridor | FollowPathRLShimACD3W50 - RPP | 달린 길 m | 0.02 ± 0.00 | t 4.6 | 5/4 |
| e_corridor | FollowPathRLShimACD3W50 - RPP | 몸 최소 거리 m | -0.01 ± 0.00 | t -2.2 | 5/4 |
| e_corridor | FollowPathRLShimACD3W50 - RPP | 도리도리 /분 | -14.45 ± 9.41 | t -1.5 | 5/4 |
| e_corridor | FollowPathRLShimACD3W50 - RPP | 주행 vx m/s | 0.01 ± 0.00 | t 17.3 | 5/4 |

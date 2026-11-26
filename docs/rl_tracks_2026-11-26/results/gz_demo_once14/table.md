| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU % | cmd 간격 p95 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| l_once | FollowPathRLShimDemoHistStop | 통로 | 5/5 | 16.4 ± 0.3 | 1.169 ± 0.030 | 0.087 ± 0.006 | 0.088 ± 0.003 | 16 ± 2 (58.6/분, 13.68/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| l_once | FollowPathRLShimDemoHist | 통로 | 5/5 | 14.5 ± 0.7 | 1.212 ± 0.016 | 0.050 ± 0.000 | 0.022 ± 0.003 | 11 ± 1 (47.2/분, 9.41/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| m_once | FollowPathRLShimDemoHistStop | 통로 | 5/5 | 17.9 ± 0.8 | 1.214 ± 0.024 | 0.064 ± 0.005 | 0.061 ± 0.004 | 15 ± 2 (49.0/분, 12.02/m) | 0 (0.0/분, 0.00/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| m_once | FollowPathRLShimDemoHist | 통로 | 4/4 | 14.2 ± 0.7 | 1.226 ± 0.003 | 0.050 ± 0.000 | 0.000 ± 0.000 | 11 ± 2 (45.4/분, 8.77/m) | 4 (4.2/분, 0.82/m) | 0 (0.0/분, 0.00/m) | 11 ± 0 | 60.0 ± 0.0 |
| m_once | FollowPathRLShimDemoHist | 밖 | 0/1 | - | 1.086 ± 0.000 | 0.050 ± 0.000 | 0.000 ± 0.000 | 7 ± 0 (10.2/분, 6.45/m) | 4 (5.8/분, 3.68/m) | 2 (2.9/분, 1.84/m) | 11 ± 0 | 60.0 ± 0.0 |

| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s | 주행 중 평균 vx m/s |
|---|---|---|---|---|---|---|---|
| l_once | FollowPathRLShimDemoHistStop | 통로 | 0.0 ± 0.0 | 4.1 ± 0.5 | 2.1 ± 0.7 | 10.2 ± 0.5 | 0.112 ± 0.005 |
| l_once | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.9 ± 0.5 | 0.7 ± 0.3 | 10.9 ± 0.2 | 0.104 ± 0.001 |
| m_once | FollowPathRLShimDemoHistStop | 통로 | 0.0 ± 0.0 | 5.2 ± 0.6 | 1.9 ± 0.1 | 10.7 ± 0.3 | 0.110 ± 0.002 |
| m_once | FollowPathRLShimDemoHist | 통로 | 0.0 ± 0.0 | 2.4 ± 0.5 | 0.4 ± 0.3 | 11.3 ± 0.1 | 0.103 ± 0.001 |

| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안 | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대) | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |
|---|---|---|---|---|---|---|---|---|---|
| l_once | FollowPathRLShimDemoHistStop | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 5 | 0 | 0 |
| l_once | FollowPathRLShimDemoHist | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| m_once | FollowPathRLShimDemoHistStop | 5/5 | 5/5 | 5/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |
| m_once | FollowPathRLShimDemoHist | 4/5 | 5/5 | 4/5 | 0.000 ± 0.000, 0.000 | 0.0 | 4 | 0 | 0 |

| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |
|---|---|---|---|---|---|
| l_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 시간 s | -1.91 ± 0.32 | t -5.9 | 5/5 |
| l_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 달린 길 m | 0.04 ± 0.02 | t 2.8 | 5/5 |
| l_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 몸 최소 거리 m | -0.07 ± 0.00 | t -32.5 | 5/5 |
| l_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 도리도리 /분 | -11.17 ± 4.07 | t -2.7 | 5/5 |
| l_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 주행 vx m/s | -0.01 ± 0.00 | t -3.4 | 5/5 |
| m_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 시간 s | -3.69 ± 0.50 | t -7.4 | 4/5 |
| m_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 달린 길 m | 0.01 ± 0.01 | t 1.0 | 4/5 |
| m_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 몸 최소 거리 m | -0.06 ± 0.00 | t -34.1 | 4/5 |
| m_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 도리도리 /분 | -3.84 ± 3.56 | t -1.1 | 4/5 |
| m_once | FollowPathRLShimDemoHist - FollowPathRLShimDemoHistStop | 주행 vx m/s | -0.01 ± 0.00 | t -7.5 | 4/5 |

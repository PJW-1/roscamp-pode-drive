### DESKTOP-6OULORO 2026-11-26 (N=600)

| 경우 | 플러그인 | batch | steps | dt (s) | 주기 (Hz) | n | 오류 | 넘침 | p50 (ms) | p99 (ms) | max (ms) | 평균 (ms) | p99 / 50 ms | 프로세스 CPU (%) | 측정 스레드 CPU (%) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rpp | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |  |  |  | 20 | 600 | 0 | 0 | 0.241 | 0.847 | 5.781 | 0.290 | 0.017 | 0.8 | 0.6 |
| mppi_l0 | `nav2_mppi_controller::MPPIController` | 1000 | 83 | 0.05 | 20 | 600 | 0 | 0 | 2.974 | 4.720 | 25.561 | 3.209 | 0.094 | 6.6 | 6.5 |
| mppi_l1 | `nav2_mppi_controller::MPPIController` | 500 | 56 | 0.05 | 20 | 600 | 0 | 0 | 0.819 | 1.978 | 8.395 | 0.940 | 0.040 | 2.1 | 1.9 |
| mppi_l2 | `nav2_mppi_controller::MPPIController` | 300 | 40 | 0.05 | 20 | 600 | 0 | 0 | 0.332 | 2.270 | 4.971 | 0.415 | 0.045 | 1.1 | 0.9 |
| mppi_l3 | `nav2_mppi_controller::MPPIController` | 200 | 20 | 0.1 | 10 | 600 | 0 | 0 | 0.165 | 0.547 | 3.675 | 0.197 | 0.011 | 0.4 | 0.2 |
| mppi_l4 | `nav2_mppi_controller::MPPIController` | 100 | 15 | 0.1 | 10 | 600 | 0 | 0 | 0.088 | 0.510 | 1.450 | 0.106 | 0.010 | 0.3 | 0.1 |
| rl | `pode_rl_controller::FollowPathRL` |  |  |  | 20 | 600 | 0 | 0 | 0.152 | 1.063 | 3.735 | 0.180 | 0.021 | 0.6 | 0.4 |
| onnx_only | `OnnxPolicy store_arena_4b` |  |  |  | 20 | 600 | 0 | 0 | 0.057 | 0.342 | 1.208 | 0.074 | 0.007 | 0.4 | 0.2 |

넘침: 그 경우의 주기(1/주기 Hz)보다 오래 걸린 호출 수. 프로세스 CPU: 측정 구간의 (user+system) / 벽시계, 한 코어 = 100 %.

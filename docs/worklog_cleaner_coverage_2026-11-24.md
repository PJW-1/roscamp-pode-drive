# 작업 기록: 청소 로봇 단계 1, 청소 격자와 커버리지 경로 (2026-11-24)

## 결론

- 청소 로봇 도입 계획(Drive `청소로봇_도입_계획_2026-11-24.md` 4판)의 단계 1(4-2)을 끝냈다. 사용자 결정(Q3)대로 Fields2Cover를 쓰지 않고 numpy로 직접 구현했다.
- v7: 7셀, 16줄, 한 바퀴 104.8 s(0.12 m/s, 제자리 회전 제외). d2: 3셀, 9줄, 126.4 s.
- 청소 격자 빈칸 기준 커버율은 v7과 d2 모두 100 %다. 키우기 전 HALL 바닥 기준 커버율은 v7 63.4 %, d2 82.5 %다.
- 둥글게 바꾼 모든 경로가 벽과 막힌 것에서 0.093 m(반지름 0.083 + 여유 0.01) 이상 떨어져 있다. 이것은 정확한 선분 거리로 시험한다.
- 브랜치는 `feat/cleaner-coverage-2026-11-24`(pode-drive main `aff00d7`에서 시작)이다. main에는 넣지 않았다.

## 바꾼 것

| 파일 | 내용 |
|---|---|
| `pc/fleet_core/fleet_core/coverage.py` | 청소 격자(`CleanGrid`), 셀 분해(`decompose`), 줄 고르기(`cell_lines`), 셀 순서와 이동(`make_plan`, `astar`, `transit`), pode_fsm과 같은 `rounded`, 정확한 거리 검사(`path_clear`, `segment_clearance`), 커버율, data 파일과 명령줄 |
| `pc/fleet_core/fleet_core/data/coverage_v7.yaml`, `coverage_d2.yaml` | 생성 결과(파라미터, 집 자리, 요약, 셀, 줄, 이동 구간). `python3 -m fleet_core.coverage --layout v7 --write`로 다시 만든다 |
| `pc/fleet_core/test/test_coverage.py` | 39개 시험 |
| `tools/coverage_plot.py` | 그림(`docs/img/coverage_v7.png`, `coverage_d2.png`) |
| `pc/fleet_core/package.xml` | `python3-numpy` 실행 의존성 |
| `docs/청소_로봇.md` | 방법, 파라미터, 결과, 다시 만들기, Fields2Cover를 쓰지 않은 까닭, 남은 일 |

설계와 숫자의 자세한 내용은 `docs/청소_로봇.md`에 있다.

## 결과 숫자 (기본 파라미터: 격자 0.025, 반지름 0.083, 여유 0.01, 줄 간격 0.10, 청소 폭 0.112, 모서리 반지름 0.10)

| | v7 | d2 |
|---|---|---|
| 셀 / 줄 | 7 / 16 | 3 / 9 |
| 줄 길이 합 | 8.775 m | 12.950 m |
| 이동 길이 합(둥글게 바꾼 뒤) | 3.803 m | 2.222 m |
| 한 바퀴 시간 | 104.8 s | 126.4 s |
| 커버율(HALL 바닥 / 청소 격자) | 63.4 % / 100 % | 82.5 % / 100 % |
| P-1을 가까운 빈칸으로 옮김 | 예: (0.12, 0.10) → (0.113, 0.263) | 아니오 |
| 가장 가까운 거리(둥근 경로, 2 mm 표본으로 따로 잼) | 0.0936 m | 0.0933 m |

- 줄 간격 0.068(격자 위에서 실제 0.05 m): v7 24줄 153.4 s, d2 14줄 210.0 s. 검사는 둘 다 통과한다.
- 재는 방법: `cd pc/fleet_core && python3 -m fleet_core.coverage --layout v7`(요약 출력). 가장 가까운 거리는 일회성 스크립트로 쟀다. 그 스크립트는 둥글게 바꾼 경로를 2 mm 간격으로 나누고, 각 점에서 벽과 막힌 사각형까지의 거리를 계산한다.

## 시험

- `cd pc/fleet_core && ROS_DOMAIN_ID=90 python3 -m pytest -q test/test_coverage.py -p no:cacheprovider` → `39 passed`.
- 전체 `pc/fleet_core` 시험 결과는 이 기록 아래 "전체 시험"에 적는다.

## 검수와 고친 것

Codex 검수(범위 `aff00d7..f508d4f`)의 지적을 확인하고, 실패하는 시험부터 쓴 뒤 고쳤다.

- **칸 중심 판정:** 칸 중심이 0.093 m 이상이면 칸 전체를 빈칸으로 보아서, 칸 가장자리 점은 0.091 m까지 가까워질 수 있었다. 경로 검사를 정확한 선분 거리 검사(`path_clear`)로 바꿨다. 시험: `test_path_clear_sees_points_off_cell_centre`.
- **표본 사이 건너뛰기:** 선분을 6.25 mm 간격 표본으로만 봤다. 선분과 사각형 사이 거리를 해석적으로 계산하도록 바꿨다. 시험 `test_path_clear_sees_segment_between_samples`는 가운데만 가까운 선분을 새 검사가 잡는지 본다. 이 입력은 고치기 전 코드도 잡으므로, 고치기 전 실패를 보여 주는 시험은 아니다.
- **격자보다 작은 줄 간격:** 받아들이고 한 칸으로 늘렸다. 이제 `ValueError`로 거절한다. 시험: `test_spacing_below_grid_is_rejected`.
- **검사 실패에도 `--write`가 파일을 씀:** 이제 검사가 실패하면 파일을 쓰지 않고 1을 돌려준다. 시험: `test_write_refuses_when_paths_fail`.
- **커버율 시험 기준:** 90 %에서 99 %로 올렸다. 한 칸 폭 구간을 줄로 쓰지 않아서 생길 수 있는 빈틈을 이 시험이 잡는다.
- 경로 단축도 같은 정확한 검사를 쓰게 되어 이동 구간이 조금 짧아졌다(v7 3.879 → 3.803 m). data 파일을 다시 만들었다.
- **두 번째 검수(범위 `f508d4f..076818d`) 반영:**
  - 좌표를 mm로 반올림해 저장해서, 저장된 경로가 0.093 m보다 최대 0.4 mm 가까웠다(v7 0.0926, d2 0.0930). 칸 중심이 0.5 mm 단위라 좌표를 0.1 mm로 저장하게 바꿨다. 시험 `test_saved_paths_pass_check`는 저장된 좌표로 다시 둥글게 바꿔 검사하며, 고치기 전에는 실패했다.
  - HALL 안인지 2 mm 표본으로 봐서 표본 사이의 짧은 이탈을 놓칠 수 있었다. HALL 사각형마다 선분이 드는 구간을 모아 [0, 1]을 덮는지 보도록 바꿨다. 시험 `test_path_clear_sees_short_exit_from_hall`은 고치기 전에는 실패했다.
  - 30 µm보다 짧은 선분을 길이 0으로 보는 근사(오차 1 nm 이하)는 동작에 영향이 없어 그대로 두었다.
- 고치지 않은 지적이 하나 있다. Codex는 P-1이 막혔을 때 가까운 빈칸에서 출발하는 것을 결함으로 보았지만, 이것은 지시한 설계다. P-1과 출발점 사이 구간은 실행부가 `bay_out_1`(P-1 → E-1)과 E-1 ↔ 출발점 이음, 돌아올 때 `bay_in_1`로 보낸다고 문서에 적었다.

## 남은 일

- 관제 연결(역할, 배차 제외, 실행부)은 단계 2에서 한다. FollowWall 가장자리 한 바퀴도 아직 남아 있다.
- v7 셀이 잘게 나뉜다. 짧은 줄이 있고, 옆 셀의 줄과 2.5 cm만 떨어진 줄도 있다. 단계 4 Gazebo 판에서 한 바퀴 시간을 보고 정한다.
- HALL 바닥 기준 커버율(v7 63 %): 다른 구역 경계는 벽이 아니라서 FollowWall로 덮을 수 없다. 구역 경계에서 키우는 거리를 따로 둘지는 단계 2에서 정한다.
- Gazebo와 실물은 돌리지 않았다.

## 전체 시험

- `cd pc/fleet_core && ROS_DOMAIN_ID=91 python3 -m pytest -q test -p no:cacheprovider` → `1308 passed, 7 xfailed in 129.49s`(두 번째 검수 반영 뒤). main `aff00d7`의 같은 시험은 `1269 passed, 7 xfailed`이고, 늘어난 39개가 `test_coverage.py`다.

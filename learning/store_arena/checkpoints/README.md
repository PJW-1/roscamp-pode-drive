# 정책 체크포인트 백업

실물 기본 정책(4b)과 후보 정책(acd2, acd3, demohist)의 rsl_rl 체크포인트다. 다른 PC에서도 저장소만으로 이어 학습하거나
onnx를 다시 내보낼 수 있게 여기에 둔다. 나머지 체크포인트는 학습 PC의 `D:/isaac/runs/store_arena/`에만 있다.

| 정책 | 파일 | 출처(학습 PC) | 설정 기록 |
|---|---|---|---|
| store_arena_4b | `store_arena_4b/model_998.pt` | `ppo_k/c2/model_998.pt` | `store_arena_4b/run.json` (둘째 청크. 첫 청크는 처음부터 500번) |
| store_arena_acd2 | `store_arena_acd2/model_1496.pt` | `acd24/t_acd_s3_c2/model_1496.pt` | `store_arena_acd2/run.json` (둘째 청크) |
| store_arena_acd3 | `store_arena_acd3/model_1745.pt` | `acd3/t_wall/model_1745.pt` | `store_arena_acd3/run.json` |
| store_arena_demohist | `store_arena_demohist/model_2044.pt` | `n26/t_demo_hist/model_2044.pt` | `store_arena_demohist/run.json` |

- 파일마다 sha256, 반복 번호(`iter`), 입력 칸 수는 `index.json`에 있고 `test_checkpoints.py`가 파일과 맞는지 본다.
- 4b, acd2, acd3 실행은 `run_config.json`이 생기기 전 것이라 그때의 `run.json`을 같이 둔다. demohist는 `run.json`과
  학습 설정 전체가 든 `run_config.json`을 같이 둔다(`run.json`의 `run_config_sha256`이 그 파일의 해시이고 시험이 맞는지 본다).
  계보는 `../POLICIES.md`에 있다.
- 이어 학습: `train.py --init learning/store_arena/checkpoints/store_arena_acd3/model_1745.pt ...`
  (옵티마이저는 새로 시작한다. 학습 PC가 아니면 Isaac Lab 설치가 필요하다.)
- onnx 다시 내보내기: `learning/goal_reach/export_policy.py --checkpoint <이 파일> ...` (`pode/pode_rl_controller/README.md`).
- 새 후보가 생기면 폴더와 `index.json` 줄을 더한다. 파일 하나가 0.6 MB쯤이라 후보만 넣는다.
- 저장소 밖 예비 사본: Drive `ros2_edu/RL_정책_백업` 폴더. 파일 이름은 `<정책 이름>__<저장소 안 파일 이름>`이고,
  크기와 sha256은 그 폴더의 문서 「RL 정책 백업 안내」에 있다. 지금은 acd3와 demohist의 `.pt`, onnx, `policy_spec.json`,
  `run.json`(demohist는 `run_config.json`도)이 올라가 있다. 새 후보를 넣을 때 같이 올린다(`../POLICIES.md` 기록 규칙 5).

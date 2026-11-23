# 실물 키트를 nav2fleet 기준판과 나란히 두기 (2026-11-23)

브랜치 `feat/real-side-by-side-2026-11-23`(main 9cbd413에서 시작). 사용자 요청: "포드드라이브쪽은 새로파서 따로 실물테스트할수있도록해놔".

## 왜

- nav2fleet 기준판(태그 `rpp-baseline-2026-11-21`)과 이 저장소의 `deploy.sh`가 로봇의 같은 자리 `~/nav2fleet_ws`, `~/nav2fleet_real`에 썼다. 이 저장소를 배포하면 기준판이 덮였다.
- 11-23 실물 시험은 하루 종일 기준판으로 한다(Drive `실물_측정_체크리스트_2026-11-23.md`). 그 뒤 pode-drive 판을 따로 시험하고, 안 되면 다시 배포하지 않고 기준판으로 돌아갈 수 있어야 한다.

## 바꾼 것

| 파일 | 내용 |
|---|---|
| `tools/real_pode/deploy.sh` | `DEPLOY_NAME`(기본 `nav2fleet`)으로 로봇 폴더를 정한다: `~/<이름>_ws/src/...`, `~/<이름>_real/`. 이름은 `^[a-z][a-z0-9_]*$`만 받고, 틀리면 로봇에 닿기 전에 exit 2. 실행 묶음에 `ws_name`(작업공간 이름)을 남긴다 |
| `tools/real_pode/env.sh` | 옆의 `ws_name`이 가리키는 `~/<작업공간>/install/local_setup.bash`를 읽는다. 없으면 예전대로 `~/nav2fleet_ws`. 이상한 이름이면 아무것도 읽지 않고 `return 1` |
| `tools/real_pode/make_params.py` | 로봇에서 원본을 찾는 자리를 `~/<ws_name>/src`로(없으면 `~/nav2fleet_ws/src`). 이상한 이름이면 멈춘다 |
| `tools/real_pode/start.sh` | 다른 실행 묶음의 `real_pode.launch.py`가 돌고 있으면 그 명령을 보여 주고 띄우지 않는다(같은 로봇에 두 묶음이 겹쳐 뜨지 않게) |
| `tools/real_pode/pc_env.sh` | `NAV2FLEET_WS_SETUP`을 주지 않으면 이 저장소가 든 작업공간(`<ws>/src/<저장소>`, 링크를 따라간 실제 자리)의 `install/setup.bash`를 읽는다. 없으면 예전 기본값 `~/ros2camp/nav2fleet_ws/install/setup.bash` |
| `tools/real_pode/README.md` | 위 설명과 "기준판과 나란히 두기" 절 |

기본값은 그대로다. `DEPLOY_NAME`을 주지 않으면 예전과 같은 자리에 간다.

## 시험

- 시험을 먼저 썼다(`tools/test/test_real_pode.py`): 다른 이름 배포의 목적지와 `ws_name`, 기본 이름, 틀린 이름 거절, `env.sh`의 작업공간 고르기와 틀린 이름, 로봇 자리의 `make_params.py`, 다른 묶음이 돌 때 `start.sh` 거절, `pc_env.sh`의 작업공간 고르기(자기 작업공간, 없을 때 예전 기본값, 링크, 준 값이 이김).
- 고치기 전: 배포 시험과 `pc_env.sh` 시험 둘이 실패하는 것을 보았다. 나머지 새 시험은 시험을 돌리는 도중에 코드를 고쳐서, 고치기 전 실패는 보지 못했다.
- `test_env_values`는 `env.sh` 글자에 `nav2fleet_ws/install/local_setup.bash`가 있는지 보던 시험이라, 기본값 `_env_ws=nav2fleet_ws`와 `/install/local_setup.bash`를 보게 바꿨다. 기본 작업공간을 실제로 읽는지는 새 동작 시험(`test_env_reads_the_workspace_its_kit_was_deployed_with[None-nav2fleet_ws]`)이 본다.
- 결과: 고친 뒤 `tools/test` 전체 283개 중 `test_env_values` 하나만 실패. 그것을 고친 뒤 관련 시험 113개 통과.

## PC에서 한 것 (이 브랜치와 별개)

- `~/ros2camp/podedrive_ws/src/roscamp-pode-drive`에 새로 clone하고 `colcon build`(15개 패키지). `pode_rl_controller`는 이 PC에 `ros-jazzy-onnxruntime-vendor`가 없어 처음에는 뺐다.
- 로봇 배포 목록에는 `pode_rl_controller`가 없다(실물 기본 컨트롤러는 RPP).

## 실물에서 쓰는 법

README "기준판과 나란히 두기" 절. 요약:

```bash
# PC (이 저장소 루트)
DEPLOY_NAME=podedrive tools/real_pode/deploy.sh pinky@<로봇 주소>
# 로봇
cd ~/podedrive_ws && colcon build
~/nav2fleet_real/stop.sh        # 기준판이 돌고 있으면
~/podedrive_real/start.sh pode1 0.12 0.10 1.5708
```

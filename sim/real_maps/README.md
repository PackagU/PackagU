# 실측 지도 Gazebo 시뮬레이션 (`sim/real_maps`)

신공학관 실측 지도로 돌린 Gazebo E2E와 택배 집기 시뮬레이션 코드, 결과 기록(`results/`)을 담았다. 월드 생성기(`generate_worlds.py`, `start_sim.sh`)는 공개본에서 뺀 내부 지도 정제 폴더(`docs/field_review_20260915/`)를 참조한다. 이 저장소 단독 체크아웃에서는 월드를 재생성할 수 없다. 코드 구조와 과거 시뮬레이션 결과를 확인하는 참고 아카이브로 둔다.

## 구성 특징

`sim_container.sh`와 `isolation.py`는 시뮬 `/cmd_vel`이 같은 LAN의 Jetson으로 새는 사고를 막는다. Docker `--internal` 네트워크와 `ROS_LOCALHOST_ONLY=1`, 별도 `ROS_DOMAIN_ID`(77 · 79)가 모두 맞아야 컨테이너가 뜬다.

`opencr_shim.py`는 실제 `OpencrBridgeNode`의 0.5 s 타임아웃, 48 rpm 상한, 60 rpm/s 가감속 제한을 그대로 재사용한다. Nav2 출력도 실제 `nav_safety_gate` 노드를 거친다.

엘리베이터 승강 중 바퀴가 돌지 않는 조건을 맞추려고 인코더 odom과 구르는 볼캐스터 모델을 넣었다(`world_params.yaml`). 층 전환 뒤에는 새 AMCL 표본 오차와 공분산, TF 안정 구간을 확인한 뒤 다음 goal을 보낸다.

Nav2 구간 사이의 미세 동작은 `apps/jetson_skill_tests/skilltests/` 상태머신을 현장과 같이 쓴다. 시뮬레이션에서는 `sim_adapters.py`로 입출력만 바꾼다. `pickup/` 폴더는 2면 ArUco 지그와 전방 초음파로 팔레트 정렬과 상승을 따로 시험한 모듈이다(결과 `pickup/results/20261029.json`).

## 주요 파일

| 파일 | 역할 |
|---|---|
| `world_params.yaml` | 월드 · 엘리베이터 · 로봇 · 판정 기준 파라미터 |
| `generate_worlds.py` | 점유 격자에서 `building.world`와 URDF 생성 |
| `sim_container.sh` / `isolation.py` | 격리 컨테이너 실행과 네트워크 · 도메인 검사 |
| `sim_stack.py` / `opencr_shim.py` | Nav2 스택 기동 순서, 벽 거리 · RTF 계측, OpenCR 명령 제한 |
| `elevator_sim_node.py` / `fake_floor_reader.py` | 엘리베이터 문 · 층 이동(`/elevator/state`)과 층수 API 모사 |
| `run_e2e.py` / `run_pretest.py` / `autonomy_smoke.py` | 왕복 E2E, 사전 시험(G1~G3), 짧은 자율 점검 러너 |
| `sim_adapters.py` / `perception.py` / `arm_geometry.py` / `arm_lift.py` | 카메라 인식, 4 DOF 팔 기하 · 궤적, 리프트 어댑터 |
| `results/20261008_summary.md` | 2026-10-08 사전 시험 · 왕복 E2E · 층 전환 정량 결과 |
| `pickup/results/20261029.json` | 2026-10-29 독립 자동 택배 집기 결과 |

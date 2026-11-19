# 실측 지도 Gazebo 시뮬레이션 (`sim/real_maps`)

> 신공학관 실측 지도로 돌렸던 Gazebo E2E · 택배 집기 시뮬레이션 코드와 결과 기록(`results/`)입니다.  
> 월드 생성기(`generate_worlds.py`, `start_sim.sh`)가 공개본에서 뺀 내부 지도 정제 폴더(`docs/field_review_20260915/`)를 참조하므로, 이 저장소만 받아서는 월드를 새로 생성할 수 없습니다. 코드 구조와 과거 시뮬레이션 결과 참고용으로 보관합니다.

## 구성 요약

- **컨테이너 격리 (`sim_container.sh`, `isolation.py`)**: 같은 LAN에 Jetson이 물려 있어 시뮬 `/cmd_vel`이 실제 로봇으로 넘어가지 않도록 Docker `--internal` 네트워크, `ROS_LOCALHOST_ONLY=1`, 별도 `ROS_DOMAIN_ID`(77 / 79)가 맞지 않으면 기동을 막습니다.
- **구동 안전 재사용 (`opencr_shim.py`)**: 실제 `OpencrBridgeNode`의 0.5 s 타임아웃, 48 rpm 상한, 60 rpm/s 가감속 제한을 그대로 쓰고, `nav_safety_gate`도 실제 노드를 붙였습니다.
- **인코더 오도메트리와 재정위 (`sim_stack.py`)**: 엘리베이터 안에서 바퀴가 돌지 않는 조건을 맞추려고 인코더 odom과 구르는 볼캐스터 모델을 넣었습니다. 층 전환 뒤에는 새 AMCL 표본 오차·공분산·TF 안정을 확인한 뒤 다음 goal을 보냅니다.
- **공용 상태머신 (`sim_adapters.py`)**: `apps/jetson_skill_tests/skilltests/`의 자율 상태머신을 현장과 같이 쓰며 어댑터만 시뮬용으로 바꿨습니다.
- **독립 택배 집기 (`pickup/`)**: 2면 ArUco 지그와 전방 초음파로 팔레트 정렬·삽입·상승을 시험한 모듈입니다(결과: `pickup/results/20261029.json`).

## 주요 파일

| 파일 | 설명 |
|---|---|
| `world_params.yaml` | 월드 · 엘리베이터 · 로봇 · 판정 기준 파라미터 |
| `generate_worlds.py` | 점유 격자에서 Gazebo 벽 모델(`building.world`)과 URDF 생성 |
| `sim_container.sh` / `isolation.py` | 격리 컨테이너 실행 및 네트워크·도메인 검사 |
| `sim_stack.py` / `opencr_shim.py` | Nav2 스택 기동 순서, 벽 거리·RTF 계측, OpenCR 명령 제한 |
| `elevator_sim_node.py` / `fake_floor_reader.py` | 엘리베이터 문·층 이동(`/elevator/state`) 및 층수 API 모사 |
| `run_e2e.py` / `run_pretest.py` / `autonomy_smoke.py` | 왕복 E2E, 사전 시험(G1~G3), 짧은 자율 점검 러너 |
| `sim_adapters.py` / `perception.py` / `arm_geometry.py` / `arm_lift.py` | 카메라 인식, 4 DOF 팔 기하·궤적, 리프트 어댑터 |
| `results/20261008_summary.md` | 2026-10-08 사전 시험 · 왕복 E2E · 층 전환 정량 결과 |
| `pickup/results/20261029.json` | 2026-10-29 독립 자동 택배 집기 결과 |

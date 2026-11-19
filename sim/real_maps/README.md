# 실측 지도 Gazebo 시뮬레이션 (`sim/real_maps`)

> **안내 (코드 열람·과거 검증 결과 참고용):** 이 폴더는 신공학관 실측 지도를 바탕으로 구축한 Gazebo E2E·독립 택배 습득 시뮬레이션의 **설계·구현 구조와 과거 검증 결과(`results/`)를 열람하기 위한 아카이브**다. 월드 생성기(`generate_worlds.py`·`start_sim.sh`)가 공개 레포에서 제외한 내부 지도 정제 폴더(`docs/field_review_20260915/`)를 참조하므로, 이 공개 레포 단독 체크아웃에서는 월드 재생성 및 즉시 실행을 지원하지 않는다. 모든 기록은 **시뮬레이션 검증**이며 실물 성공을 뜻하지 않는다.

## 1. 격리 설계와 안전 경계

현장 Jetson과 같은 LAN 환경에서 시뮬레이션 `/cmd_vel`이 실제 로봇으로 흘러가는 사고를 막기 위해, 모든 시뮬레이션은 `sim_container.sh`와 `isolation.py`가 강제하는 격리 컨테이너 안에서만 동작하도록 설계했다.

- Docker `--internal` 전용 네트워크(`packagu_sim_isolated`)로 LAN·외부 통신 차단
- `ROS_LOCALHOST_ONLY=1`, 비-0 `ROS_DOMAIN_ID`(기본 `77`, 택배 집기 시험 `79`) 강제
- `privileged`·`host` 네트워크·시리얼 장치 전달 금지(조건 미충족 시 `isolation.assert_isolated`가 즉시 기동 거부)

## 2. 명령 경로와 센서·오도메트리 모델

```text
Nav2 /cmd_vel → nav_safety_gate(실제 노드·nav_safety.yaml) → /cmd_vel_safe → opencr_shim → /sim/cmd_vel_drive → Gazebo diff drive
```

- **OpenCR 시뮬레이션 심(`opencr_shim.py`)**: 실제 구동 브리지(`OpencrBridgeNode.send_command_tick`)의 0.5 s watchdog, 48 rpm 상한 차단, 60 rpm/s 가감속 제한을 그대로 재사용한다.
- **인코더 오도메트리 & 접촉 모델**: 엘리베이터 승강 중 바퀴가 회전하지 않는 실물 특성을 반영하기 위해 `encoder odom`과 구르는 볼캐스터 접촉 모델(`world_params.yaml`)을 적용했다. 지면 진실(`/sim/ground_truth`)은 평가 지표 계산에만 사용한다.
- **층 전환 재정위 게이트**: `/initialpose` 발행만으로 주행을 재개하지 않고, 새 AMCL 표본의 위치·방향 오차, 공분산, `map → base_footprint` TF 안정 구간이 기준을 만족할 때만 다음 구간으로 진행한다.

## 3. 자율 시퀀스 구조 (`apps/jetson_skill_tests` 공용 상태 머신)

Nav2 구간 사이의 미세 동작은 현장 시험 코드(`apps/jetson_skill_tests/skilltests/`)와 **동일한 상태 머신**을 공유하며, 입출력 어댑터(`sim_adapters.py` vs `ros_adapters.py`)만 교체한다.

| 단계 | 핵심 동작 흐름 |
|---|---|
| 택배 습득 | 보관 구역 정렬 → 지게팔 하강 → 고정 보기 자세로 팔레트·택배 탐색 → 포켓 정렬·진입 → 초음파 상한 기준 들기 |
| 엘리베이터 입장 | 호출 버튼판 인식 → 누르기 자세 계산·접근 → 티칭 사이클(`servo_protocol.py`) 실행 → 램프·문 열림 확인 → LiDAR 문 틈 정렬 후 직진 진입 |
| 탑승·층 전환 | 캐빈 내부 층 버튼 탐색·정렬·누르기 → 문 위 7-세그먼트 층 표시창 판독 → 목적층 도달 시 지도 전환·AMCL 재정위 → 하차 및 배달·복귀 |

## 4. 독립 자동 택배 집기 (`pickup/`)

엘리베이터 왕복 없이 몸체 고정 카메라의 ArUco 표식 영상·전방 초음파·인코더 오도메트리로 정렬 → 접근 → 포켓 삽입 → 상승 → 후진을 검증하는 모듈이다(`pickup/build_scene.py`, `controller.py`, `vision.py`, `config.yaml`, 결과 기록 `pickup/results/20261029.json`).

## 5. 주요 파일 구성

| 파일 | 역할 |
|---|---|
| `world_params.yaml` | 월드·엘리베이터·LiDAR·로봇·판정 기준 파라미터(출처 및 가정값 명시) |
| `generate_worlds.py` | 점유 격자 → Gazebo 상자 벽 월드(`building.world`) 및 로봇 URDF 생성기 |
| `sim_container.sh` / `isolation.py` | 격리 컨테이너 관리 및 네트워크·도메인 격리 검증 |
| `sim_stack.py` | localization → initial pose → Nav2 기동 순서, 준비 판정, 벽 거리·RTF 지표 계측 |
| `opencr_shim.py` | 실제 구동 안전 계약(`nav_safety_gate` + OpenCR 명령 제한) 시뮬레이션 심 |
| `elevator_sim_node.py` / `fake_floor_reader.py` | 엘리베이터 문·층 이동 모델(`/elevator/state`) 및 층수 인식기 HTTP API 모사 |
| `run_e2e.py` / `run_pretest.py` / `autonomy_smoke.py` | 왕복 E2E·사전 구간(G1~G3)·자율 시퀀스 러너 |
| `sim_adapters.py` / `perception.py` / `arm_geometry.py` / `arm_lift.py` | 공용 상태 머신용 시뮬 어댑터, 팔 카메라 인식, 정·역운동학 및 궤적 모델 |
| `parcel_mock.py` / `dressing.py` | 팔레트·택배 모델, 버튼 램프·7-세그먼트 표시창 시각 장식 |
| `results/` | 2026-10-08 사전 시험·E2E 왕복·층 전환 정량 결과(`20261008_summary.md`, CSV/JSON) |

## 6. 보관된 주요 결과 (`results/`)

- `results/20261008_summary.md` 및 `20261008_e2e_runs.csv`: 2026-10-08 신공학관 실측 지도 기반 사전 시험(G1~G3) 및 왕복 E2E(캐빈 `direct` 방식 완주 및 `nav2` 방식 F2 진입 실패 기록, 층 전환 AMCL 수렴 지표) 정량 기록.
- `pickup/results/20261029.json`: 2026-10-29 독립 자동 택배 집기 시뮬레이션 결과.

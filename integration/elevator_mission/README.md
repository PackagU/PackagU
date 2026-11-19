# `integration/elevator_mission` — 엘리베이터 배송 미션 상태머신

루트 ROS 2 워크스페이스(`src/`)와 분리된 overlay 패키지로, 엘리베이터를 포함한 실내 배송 미션의 Behavior Tree 상태머신과 카메라 기반 버튼/층수 연동 실행 구성을 담고 있다.

## 주요 구성

| 경로 | 역할 |
|---|---|
| `src/elevator_mission_pkg/` | Behavior Tree 미션 노드, 직교 경로 라우터, 웨이포인트 레지스트리, 카메라 미션 launch(`launch/elevator_camera_mission.launch.py`) |
| `src/elevator_sim_pkg/` | 초기 엘리베이터 상태 발행 시뮬레이션 보조 패키지 |
| `config/` | 가상 건물 웨이포인트(`kku_nav_points.yaml`), 배송 시나리오(`delivery_missions.yaml`) |
| `scripts/` | 미션 동작·직교 라우팅·Nav2 튜닝 오프라인 단위 테스트 |
| `test_mission_safety_fsm.py` | 미션 안전 상태머신 전이 검증 |

## 빌드 참고 (overlay)

루트 워크스페이스(`src/`)를 먼저 빌드·source한 뒤 이 폴더에서 별도로 빌드한다(`integration/COLCON_IGNORE`로 루트 빌드의 자동 혼입을 차단한다).

```bash
cd integration/elevator_mission
colcon build --symlink-install
source install/setup.bash
```

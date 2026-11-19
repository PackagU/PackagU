# `integration/elevator_mission`

엘리베이터 배송 미션용 Behavior Tree 상태머신과 카메라 미션 launch가 들어 있는 overlay 워크스페이스입니다.

- `src/elevator_mission_pkg/`: 미션 BT 노드, 직교 경로 라우터, 웨이포인트 레지스트리, `launch/elevator_camera_mission.launch.py`
- `src/elevator_sim_pkg/`: 초기 엘리베이터 상태 발행 보조 패키지
- `config/`: 가상 건물 웨이포인트(`kku_nav_points.yaml`), 배송 시나리오(`delivery_missions.yaml`)
- `scripts/`, `test_mission_safety_fsm.py`: 미션·라우터·안전 상태머신 오프라인 테스트

루트(`src/`)를 먼저 빌드·source한 뒤 이 폴더 안에서 따로 빌드합니다.

```bash
cd integration/elevator_mission
colcon build --symlink-install
source install/setup.bash
```

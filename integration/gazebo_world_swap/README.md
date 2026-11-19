# `integration/gazebo_world_swap`

층별 지도 전환 시 Gazebo 건물 월드도 목표층으로 같이 바꿔 주는 시뮬레이션 전용 overlay 워크스페이스입니다.

- `src/gazebo_world_swap_pkg/`: 층 전환 신호를 받아 Gazebo 건물 모델을 교체하는 ROS 2 노드
- `pedestrian/`: 복도 동적 보행자 장애물 노드·모델
- `scripts/`: 스왑 계약 테스트 및 왕복 스모크 스크립트(`run_l3_world_swap_smoke.sh`, `run_roundtrip_repeat.sh`)

```bash
cd integration/gazebo_world_swap
colcon build --symlink-install
source install/setup.bash
```

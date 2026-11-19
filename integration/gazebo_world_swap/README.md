# `integration/gazebo_world_swap` — 시뮬레이션 층별 Gazebo 월드 교체

`auto_floor_orchestrator_pkg`가 Nav2 지도를 층별로 교체할 때 Gazebo 시뮬레이션 월드도 함께 목표층 구조로 전환해 `/scan`·AMCL·Nav2가 일관되게 동작하도록 만드는 시뮬레이션 전용 overlay다.

## 주요 구성

| 경로 | 역할 |
|---|---|
| `src/gazebo_world_swap_pkg/` | 층 전환 트리거를 받아 Gazebo 건물 모델을 교체하는 ROS 2 노드 |
| `pedestrian/` | 복도 동적 보행자 장애물 시뮬레이션 보조 노드·모델 |
| `scripts/` | 스왑 트리거·월드 모델·계약 검증 오프라인 테스트 및 왕복 스모크 러너(`run_l3_world_swap_smoke.sh`, `run_roundtrip_repeat.sh`) |

## 빌드 참고 (overlay)

```bash
cd integration/gazebo_world_swap
colcon build --symlink-install
source install/setup.bash
```

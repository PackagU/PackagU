# `integration/elevator_auto_map_switch` — 층별 자동 지도 전환 오케스트레이터

엘리베이터가 목표층에 도착해 문이 열렸을 때(`/elevator/state`), Nav2 `map_server`의 지도를 해당 층 지도로 자동 교체하고 `/initialpose` 재정위까지 수행하는 ROS 2 overlay 워크스페이스다.

## 주요 구성

| 경로 | 역할 |
|---|---|
| `src/auto_floor_orchestrator_pkg/` | 층 전환 상태머신, 지도 레지스트리 해석(`floor_map_registry.py`), `/map_server/load_map` 및 AMCL 재정위 오케스트레이터 노드 |
| `config/floor_maps.yaml` | 층별(`F1`·`F2`·`F3`) 지도 경로와 엘리베이터 출입 초기 자세 정의 |
| `scripts/` | 상태머신·지도 레지스트리·하위 호환성·ROS 스모크 오프라인 테스트 |

## 빌드 참고 (overlay)

```bash
cd integration/elevator_auto_map_switch
colcon build --symlink-install
source install/setup.bash
```

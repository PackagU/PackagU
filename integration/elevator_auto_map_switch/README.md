# `integration/elevator_auto_map_switch`

엘리베이터가 목표층에 도착해 문이 열리면(`/elevator/state`) Nav2 `map_server` 지도를 바꾸고 `/initialpose`를 다시 잡는 overlay 워크스페이스입니다.

- `src/auto_floor_orchestrator_pkg/`: 층 전환 상태머신, 지도 레지스트리(`floor_map_registry.py`), 오케스트레이터 노드
- `config/floor_maps.yaml`: F1·F2·F3 지도 경로 및 엘리베이터 출입 초기 자세
- `scripts/`: 상태머신 · 지도 레지스트리 · 호환성 테스트

```bash
cd integration/elevator_auto_map_switch
colcon build --symlink-install
source install/setup.bash
```

# 빌드 및 실행 참고

공개 정리 단계에서는 호스트 Python 구문 검사(`py_compile`)와 정적 안전 감사만 마쳤다. Docker 빌드, `colcon build`, 오프라인 테스트, 실기 구동은 다시 돌리지 않았다.

[`sim/real_maps/`](../sim/real_maps/README.md)의 월드 생성기(`generate_worlds.py`, `start_sim.sh`)는 공개본에서 제외한 내부 지도 정제 폴더(`docs/field_review_20260915/`)를 읽는다. 이 저장소만 받아서는 월드를 새로 만들 수 없다. 시뮬레이션 코드와 과거 결과(`results/`)를 읽는 용도로만 둔다.

## 1. Docker 이미지 빌드

외부 레지스트리 대신 저장소의 Dockerfile로 로컬 빌드한다.

```bash
# 개발 PC (amd64: Gazebo, RViz2, Nav2, SLAM Toolbox)
docker build -f docker/Dockerfile -t packagu/ros2-humble-slam:humble docker/

# Jetson Xavier NX (aarch64: 런타임 전용, Gazebo 제외)
docker buildx build --platform linux/arm64 -f docker/Dockerfile.jetson -t packagu/ros2-humble-slam:humble-jetson --load docker/
```

Compose 실행 옵션은 [`docker/README.md`](../docker/README.md)에 적혀 있다.

## 2. ROS 2 워크스페이스 빌드

루트 패키지 4개(`src/`)와 통합 overlay 3개(`integration/`)는 따로 빌드한다. `integration/COLCON_IGNORE`가 있어 루트에서 `colcon build`를 돌려도 overlay 패키지가 섞이지 않는다.

```bash
# 루트 패키지 4개 (common_pkg, slam_pkg, drive_pkg, robot_arm_pkg)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash

# 필요한 overlay만 별도 빌드
(cd integration/elevator_mission && colcon build --symlink-install)
(cd integration/elevator_auto_map_switch && colcon build --symlink-install)
(cd integration/gazebo_world_swap && colcon build --symlink-install)
```

## 3. 현장 스크립트 진입점

Jetson Xavier NX 실기에서 쓰던 주요 명령이다. 세부 안전 수칙은 [`docs/deployment/`](deployment/04_field_mapping_navigation.md)에 있다.

```bash
./scripts/fieldctl --help
python3 scripts/field_video_rehearsal.py --list
python3 scripts/field_video_rehearsal.py --dry-run --yes
python3 scripts/check_portability.py
```

오프라인 단위 · 계약 테스트 51종은 [`scripts/run_offline_tests.sh`](../scripts/run_offline_tests.sh)에 묶여 있다.

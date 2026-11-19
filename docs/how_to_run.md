# 빌드 및 실행 참고

> 참고: 이 공개 저장소는 `Final_Demo@db73e09`에서 핵심 코드만 정리해 올린 것으로, 공개 정리 단계에서 Docker 빌드·테스트·실기 구동은 다시 돌리지 않았습니다(Python 문법 검사만 확인).  
> 또한 [`sim/real_maps/`](../sim/real_maps/README.md)의 월드 생성기(`generate_worlds.py`, `start_sim.sh`)는 공개본에서 뺀 내부 지도 정제 폴더를 필요로 해 바로 재생성할 수 없으며, 코드 구조와 과거 결과(`results/`) 참고용으로 남겨 두었습니다.

## 1. Docker 이미지 빌드

별도 레지스트리에서 받지 않고 저장소의 Dockerfile로 직접 빌드합니다.

```bash
# 개발 PC (amd64: Gazebo, RViz2, Nav2, SLAM Toolbox)
docker build -f docker/Dockerfile -t packagu/ros2-humble-slam:humble docker/

# Jetson Xavier NX (aarch64: 런타임 전용, Gazebo 제외)
docker buildx build --platform linux/arm64   -f docker/Dockerfile.jetson   -t packagu/ros2-humble-slam:humble-jetson --load docker/
```

Compose 실행 예시는 [`docker/README.md`](../docker/README.md)에 있습니다.

## 2. ROS 2 워크스페이스 빌드

루트 패키지(`src/`)와 overlay(`integration/`)는 따로 빌드합니다. `integration/COLCON_IGNORE`가 있어 루트에서 `colcon build`를 쳐도 overlay가 섞이지 않습니다.

```bash
# 루트 4개 패키지 (common_pkg, slam_pkg, drive_pkg, robot_arm_pkg)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash

# 필요한 overlay만 별도 빌드
(cd integration/elevator_mission && colcon build --symlink-install)
(cd integration/elevator_auto_map_switch && colcon build --symlink-install)
(cd integration/gazebo_world_swap && colcon build --symlink-install)
```

## 3. 현장 스크립트 참고

실기(Jetson Xavier NX + OpenCR 1.0 + RPLiDAR A1m8 + Arduino Uno + 4 DOF 팔)에서 쓰던 주요 진입 명령입니다. 세부 절차는 [`docs/deployment/`](deployment/04_field_mapping_navigation.md)를 참고하세요.

```bash
./scripts/fieldctl --help
python3 scripts/field_video_rehearsal.py --list
python3 scripts/field_video_rehearsal.py --dry-run --yes
python3 scripts/check_portability.py
```

오프라인 단위·계약 테스트 목록은 [`scripts/run_offline_tests.sh`](../scripts/run_offline_tests.sh)에 모여 있습니다.

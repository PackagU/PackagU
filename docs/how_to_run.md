# 환경 구성 및 참고 실행 명령

> **검증 범위 안내:** 이 공개 저장소(`PackagU/PackagU`)는 최종 보고서 첨부 및 코드 구조 열람을 목적으로 `Final_Demo@db73e09`에서 핵심 코드를 선별해 정리했다. 공개 레포 이관 과정에서 **Docker 컨테이너 빌드, `colcon build`, 오프라인 테스트, 실기 구동은 재실행하지 않았으며(`⚠️미확인`), 호스트 Python 구문 검사(`py_compile`)와 정적 안전 감사만 수행했다.**
>
> 또한 [`sim/real_maps/`](../sim/real_maps/README.md)의 월드 재생성 스크립트(`generate_worlds.py`·`start_sim.sh`)는 공개본에서 제외한 내부 지도 정제 폴더를 참조하므로, 이 저장소에서는 **시뮬레이션 구현 코드 열람 및 `results/` 과거 검증 결과 참고용**으로 제공한다.

## 1. Docker 이미지 빌드 (로컬 기준)

외부 비공개 레지스트리(GHCR)에 의존하지 않고 저장소의 Dockerfile로 직접 빌드한다.

```bash
# 개발용 PC (amd64: Gazebo · RViz2 · Nav2 · SLAM Toolbox)
docker build -f docker/Dockerfile -t packagu/ros2-humble-slam:humble docker/

# Jetson Xavier NX 배포용 (aarch64: 런타임 전용, Gazebo 제외)
docker buildx build --platform linux/arm64   -f docker/Dockerfile.jetson   -t packagu/ros2-humble-slam:humble-jetson --load docker/
```

컨테이너 기동 예시는 [`docker/README.md`](../docker/README.md)와 [`docker/compose/`](../docker/compose/)를 참고한다.

## 2. ROS 2 워크스페이스 빌드 구조 (참고)

루트 패키지 4개(`src/`)와 통합 overlay 3개(`integration/`)는 분리해 빌드한다. `integration/COLCON_IGNORE`가 있어 루트에서 `colcon build`를 실행해도 overlay가 섞이지 않는다.

```bash
# 1) 루트 핵심 패키지 4개 (common_pkg, slam_pkg, drive_pkg, robot_arm_pkg)
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash

# 2) 필요 시 overlay 별도 빌드
(cd integration/elevator_mission && colcon build --symlink-install)
(cd integration/elevator_auto_map_switch && colcon build --symlink-install)
(cd integration/gazebo_world_swap && colcon build --symlink-install)
```

## 3. 현장 운영 및 리허설 주요 진입점 (참고)

실제 로봇(Jetson Xavier NX + OpenCR 1.0 + RPLiDAR A1m8 + Arduino Uno + 4 DOF 팔) 환경에서의 주요 명령은 다음과 같다. 세부 절차와 안전 수칙은 [`docs/deployment/`](deployment/04_field_mapping_navigation.md)를 참고한다.

```bash
# 현장 매핑·주행·녹화 통합 CLI (scripts/fieldctl)
./scripts/fieldctl --help

# E2E 촬영 리허설 진행기 단계 목록 및 장비 없는 드라이런 확인
python3 scripts/field_video_rehearsal.py --list
python3 scripts/field_video_rehearsal.py --dry-run --yes

# 이식성 규칙(R1~R4) 정적 점검
python3 scripts/check_portability.py
```

## 4. 오프라인 계약 테스트 목록 (참고)

저장소에는 구동 프로토콜, 안전 게이트, 지도 계약, Jetson 배포 프로파일, 로봇팔/지게팔 계약, 리허설 진행기, 층 전환 상태머신을 검증하는 오프라인 테스트 스위트([`scripts/run_offline_tests.sh`](../scripts/run_offline_tests.sh), Python 51개 + 셸 1개 공용 / ROS 환경 6개 추가)가 포함되어 있다. (본 공개 레포 이관본에서는 미실행 `⚠️미확인`)

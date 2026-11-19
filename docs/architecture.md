# 시스템 구성

PackagU의 하드웨어 배치와 ROS 2 소프트웨어 구성을 정리한 문서입니다. 세부 치수는 [hardware_spec.md](hardware_spec.md)에 있습니다.

## 전체 연결

```mermaid
flowchart TB
  subgraph SENS["센서"]
    LIDAR["RPLiDAR A1m8 (/scan)"]
    IMU["OpenCR 1.0 IMU · 바퀴 엔코더"]
    CAM["웹캠 (팔 끝 · 적재 보조)"]
    US["지게팔 초음파 센서 (10 cm 제한)"]
  end

  subgraph JETSON["Jetson Xavier NX (Docker · ROS 2 Humble)"]
    SLAM["slam_pkg: SLAM Toolbox · AMCL · Nav2"]
    SAFE["drive_pkg: nav_safety_gate · opencr_bridge"]
    ARM["robot_arm_pkg: arm_sequence_node"]
    ORCH["integration/: elevator_mission · auto_floor_orchestrator"]
    APPS["apps/: button_arm_test · floor_reader · pickup_real · jetson_skill_tests"]
    CTRL["scripts/: fieldctl · field_video_rehearsal.py"]
  end

  subgraph ACT["펌웨어 · 구동기"]
    OPENCR["OpenCR 1.0 (2륜 차동 · 0.5 s 워치독)"]
    LIFT["Arduino Uno + TB6600 (lift_uno)"]
    SERVO["4 DOF 로봇팔 서보"]
  end

  LIDAR --> SLAM
  IMU --> SAFE
  CAM --> APPS
  US --> LIFT

  SLAM -->|/cmd_vel| SAFE
  SAFE -->|/cmd_vel_safe| OPENCR
  CTRL --> SLAM
  CTRL --> APPS
  CTRL --> LIFT
  ORCH -->|/map_server/load_map · /initialpose| SLAM
  ORCH --> ARM
  APPS --> ARM
  ARM --> SERVO
```

## 하드웨어

![하드웨어 구성 라벨](media/robot_labeled.jpg)

| 부품 | 사양 | 관련 경로 |
|---|---|---|
| 메인 보드 | NVIDIA Jetson Xavier NX (JetPack 위 Docker ROS 2 Humble) | `docker/Dockerfile.jetson`, `docker/compose/docker-compose.jetson.yml` |
| 구동 MCU | OpenCR 1.0 (2륜 차동, 바퀴 간격 0.51324 m, 지름 0.066 m, 9축 IMU) | `firmware/opencr/`, `firmware/opencr_imu/`, `src/drive_pkg/` |
| Z축 지게팔 | TB6600 + Arduino Uno (`u`/`d` 12,800펄스, 상승 중 초음파 10 cm 미만 정지) | `firmware/lift_uno/`, `scripts/lift_uno_adapter.py` |
| 4 DOF 로봇팔 | 리치 60 cm 서보 팔, 팔 끝 웹캠 장착 | `src/robot_arm_pkg/`, `apps/button_arm_test/` |
| LiDAR | RPLiDAR A1m8 (실내 유효 8 m) | `src/slam_pkg/` |

## 패키지 · 폴더별 역할

| 경로 | 역할 | 주요 파일 |
|---|---|---|
| `src/common_pkg/` | 로봇 URDF, 시뮬레이션 월드·맵 원본 설정 | `urdf/delivery_robot.urdf.xacro`, `config/kku_pre_simulation_map.yaml` |
| `src/drive_pkg/` | OpenCR 시리얼 브리지, 오도메트리, 속도 안전 게이트 | `drive_pkg/opencr_protocol.py`, `opencr_bridge.py`, `nav_safety_gate.py` |
| `src/slam_pkg/` | SLAM Toolbox · AMCL · Nav2 launch, 신공학관 F1·F2 지도 | `launch/field_base.launch.py`, `kku_navigation.launch.py`, `config/nav2_params.yaml` |
| `src/robot_arm_pkg/` | 4 DOF 팔 PWM 프로토콜, 시퀀스 실행 노드 | `robot_arm_pkg/servo_protocol.py`, `arm_sequence_node.py`, `config/camera_views.json` |
| `integration/` | 루트와 분리 빌드하는 미션·층 전환 overlay 3종 | `elevator_mission/`, `elevator_auto_map_switch/`, `gazebo_world_swap/` |
| `apps/` | 버튼 웹/API, 층수 판독, 팔레트 집기, 공용 상태머신 | `button_arm_test/app.py`, `floor_reader/app.py`, `pickup_real/pickup_real.py`, `jetson_skill_tests/` |
| `scripts/` | 현장 CLI, 26단계 E2E 진행기, 이식성·계약 테스트 | `fieldctl`, `field_video_rehearsal.py`, `check_portability.py`, `run_offline_tests.sh` |
| `firmware/` | MCU 스케치 | `opencr/opencr_drive_bridge.ino`, `opencr_imu/`, `lift_uno/lift_uno.ino` |
| `sim/real_maps/` | 신공학관 실측 지도 Gazebo 시뮬 코드 및 결과 기록 | `sim_stack.py`, `run_e2e.py`, `opencr_shim.py`, `results/20261008_summary.md` |

## 설계 시 신경 쓴 부분

- **속도 명령 안전 차단**: Nav2의 `/cmd_vel`을 OpenCR로 바로 보내지 않고 `nav_safety_gate`를 거칩니다. `/scan`이나 `/odom`이 끊기면 즉시 0을 보내고, OpenCR 펌웨어에서도 0.5초 타임아웃과 48 rpm 상한을 따로 걸었습니다([02_opencr_serial_protocol.md](deployment/02_opencr_serial_protocol.md)).
- **엘리베이터 버튼 직접 조작**: 승강기 통신 대신 팔 끝 웹캠으로 버튼과 램프를 보고, 미리 잡아 둔 PWM 시퀀스(`servo_protocol.py`)로 누릅니다.
- **지도 교체 확인**: 층 이동 뒤 `/map_server/load_map`과 `/initialpose`만 호출하고 넘어가면 이전 층 costmap이 남는 문제가 있어, 지도 크기·원점과 costmap 갱신을 확인한 뒤 다음 goal을 보냅니다(`scripts/field_video/ros_map_switch.py`).
- **워크스페이스 분리**: 실기용 기본 패키지 4개(`src/`)와 통합 overlay(`integration/`)가 섞여 빌드되지 않도록 `integration/COLCON_IGNORE`를 두었습니다.
- **현장·시뮬 코드 공유**: `apps/jetson_skill_tests/skilltests/`의 상태머신은 입출력 어댑터(`ros_adapters.py`, `sim_adapters.py`)만 바꿔 현장과 시뮬레이션에 같이 씁니다.

현장 실행 세부 절차는 [08_elevator_camera_mission.md](deployment/08_elevator_camera_mission.md)와 [09_field_video_rehearsal.md](deployment/09_field_video_rehearsal.md)에 있습니다.

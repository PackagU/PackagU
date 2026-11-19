# 개발 과정 타임라인 (2026)

이 공개 저장소(`PackagU/PackagU`)는 최종 시연 코드(`PackagU/Final_Demo@db73e09`)에서 핵심 파일만 선별해 새 커밋 이력으로 구성했다. 아래 타임라인은 개발 과정의 주요 마일스톤과 기술적 전환점을 날짜순으로 요약한다.

| 시기 | 단계 | 주요 내용과 성과 |
|---|---|---|
| **2026-03 ~ 05** | 기획 · 기구 설계 · ROS 2 기반 구축 | 건국대학교 전기전자공학부 2026 종합설계 5조 프로젝트 착수. 엘리베이터 API 없이 로봇팔로 버튼을 직접 조작하는 콘셉트 확정, Fusion 360 섀시·Z축 리프트 설계, `common_pkg`·`slam_pkg`·`drive_pkg`·`robot_arm_pkg` 및 Docker 개발 환경 구축 |
| **2026-05 ~ 06** | 미션 상태머신 · 층별 지도 자동 전환 PoC | 루트 워크스페이스와 분리된 overlay(`integration/elevator_mission`, `elevator_auto_map_switch`, `gazebo_world_swap`) 구현. `/elevator/state` 도착 이벤트를 받아 Nav2 `map_server` 지도 교체 + AMCL `/initialpose` 재정위 + Gazebo 월드 전환을 연동 |
| **2026-07** | 하드웨어 사양 기준화 · Jetson 이식성 정책 | [hardware_spec.md](hardware_spec.md) 작성 및 `Dockerfile.jetson`(aarch64)·이식성 검사기(`scripts/check_portability.py`) 수립. 데스크톱 시뮬과 Jetson 실기 컨테이너를 동일 코드로 운용하는 규칙(R1~R4) 정착 |
| **2026-08** | 왕복 시뮬 강건성 · 분산 부하 검증 | 바퀴 물리 드리프트(`fdir1` 회전·캐스터 마찰) 수정으로 정지 드리프트 `0.0000 m/90 s` 달성. 가상 건물 왕복(보행자 5 + 정적 장애물 2 + 팔/리프트 mock) 데스크톱 **10/10 PASS**, 데스크톱 Gazebo + Jetson 실전 스택 분산 E2E **3/3 PASS** (`2026-08-21`) |
| **2026-09** | 신공학관 실물 매핑 · 구동 안전 · Nav2 시험 | OpenCR 1.0 구동 브리지 + `nav_safety_gate` + `fieldctl` 운영 도구 구축. 신공학관 F1·F2 실측 SLAM 지도 제작 및 실물 Nav2 구간 시험(5회 중 2회 성공, 3회 실패 원인 분석, `2026-09-16`), 버튼 인식·4 DOF 로봇팔 실물 시험 |
| **2026-10** | 실측 지도 E2E 시뮬 · 자율 스킬 · E2E 진행기 | 신공학관 실측 지도 기반 격리 Gazebo E2E 시뮬 구축(`sim/real_maps/`, `2026-10-08` 사전 시험 19/19·direct E2E 4/4 완주·층 전환 AMCL 12/12 수렴). 현장·시뮬 공용 상태머신(`apps/jetson_skill_tests/`), 택배 습득(`apps/pickup_real/`), E2E 촬영 리허설 진행기(`scripts/field_video_rehearsal.py`) 및 `lift_uno` 초음파 높이 제한 펌웨어(`2026-10-29`) 반영 |
| **2026-11** | 최종 시연 준비 · 강의실 v3 검증 · 공개 정리 | 강의실 v3 배치도 기반 대규모 시뮬 검증(4,112회 반복, `2026-11-04` 기준 커밋 `db73e09`) 및 최종 발표. 보고서 제출용 공개 레포 `PackagU/PackagU` 정리 (`2026-11-19`) |

## 팀과 역할 (건국대학교 전기전자공학부 2026 종합설계 5조)

| 이름 | GitHub | 담당 영역 |
|---|---|---|
| 이준형 | [@JunhyungLee25](https://github.com/JunhyungLee25) | SLAM · Nav2 · 층별 지도 전환 · 시뮬레이션 · 현장 운영/통합 |
| 한수민 | [@inonewater](https://github.com/inonewater) | Fusion 360 기구 설계 · 구동부 · Z축 지게팔(리프트) |
| 이성덕 | [@DuckFrog123](https://github.com/DuckFrog123) | 4 DOF 로봇팔 · 엘리베이터 버튼 인식 및 조작 |

# 개발 타임라인 (2026)

공개 저장소는 최종 코드(`Final_Demo@db73e09`)만 골라 새 커밋으로 시작했다. 3월부터 11월까지의 작업 순서를 아래 표에 남겼다.

| 시기 | 작업 내용 |
|---|---|
| 2026-03 ~ 05 | 종합설계 5조 기획과 Fusion 360 섀시 · 리프트 설계. ROS 2 Humble Docker 환경과 기본 패키지 4개(`common_pkg`, `slam_pkg`, `drive_pkg`, `robot_arm_pkg`) 작성 |
| 2026-05 ~ 06 | `integration/` overlay 3종 구현. `/elevator/state` 이벤트를 받아 `/map_server/load_map` 지도 교체, AMCL 재정위, Gazebo 월드 전환 연결 |
| 2026-07 | [hardware_spec.md](hardware_spec.md) 작성. Jetson용 `Dockerfile.jetson`과 이식성 검사기(`scripts/check_portability.py`) 추가 |
| 2026-08 | 바퀴 마찰과 캐스터 모델을 고쳐 정지 드리프트 해소. 가상 건물 왕복 시뮬 10/10 통과, PC Gazebo + Jetson 분산 부하 시험 3/3 통과(8/21) |
| 2026-09 | OpenCR 브리지, `nav_safety_gate`, `fieldctl` 정비. 신공학관 F1 · F2 실측 지도 작성과 복도 Nav2 시험(9/16), 4 DOF 로봇팔 · 버튼 인식 시험 |
| 2026-10 | 신공학관 실측 지도 Gazebo E2E 시뮬(10/08), 현장 · 시뮬 공용 상태머신(`apps/jetson_skill_tests/`), 팔레트 집기(`apps/pickup_real/`), 26단계 E2E 진행기 및 `lift_uno` 초음파 높이 제한 펌웨어(10/29) 반영 |
| 2026-11 | 강의실 v3 배치도 시뮬 반복 검증(11/04, 4,112회)과 최종 발표. 보고서용 공개 저장소 `PackagU/PackagU` 정리(11/19) |

## 팀원

건국대학교 전기전자공학부 2026 종합설계 5조

- 이준형([@JunhyungLee25](https://github.com/JunhyungLee25)): SLAM, Nav2, 층별 지도 전환, 시뮬레이션, 현장 통합
- 한수민([@inonewater](https://github.com/inonewater)): Fusion 360 기구 설계, 구동부, Z축 지게팔
- 이성덕([@DuckFrog123](https://github.com/DuckFrog123)): 4 DOF 로봇팔, 버튼 인식과 조작

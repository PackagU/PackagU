# 개발 타임라인 (2026)

공개 저장소는 최종 코드(`Final_Demo@db73e09`)만 골라 새 커밋으로 올렸기 때문에, 3월부터 11월까지의 개발 흐름을 여기에 간단히 남깁니다.

| 시기 | 진행 내용 |
|---|---|
| **2026-03 ~ 05** | 종합설계 5조 기획 및 Fusion 360 섀시·리프트 설계. ROS 2 Humble + Docker 기본 환경과 `common_pkg`, `slam_pkg`, `drive_pkg`, `robot_arm_pkg` 구성 |
| **2026-05 ~ 06** | `integration/` overlay 3종 구현. `/elevator/state` 이벤트를 받아 Nav2 지도 교체(`/map_server/load_map`), AMCL 재정위, Gazebo 월드 전환을 연동 |
| **2026-07** | 하드웨어 사양서([hardware_spec.md](hardware_spec.md)) 정리, Jetson용 `Dockerfile.jetson` 및 이식성 검사(`scripts/check_portability.py`) 도입 |
| **2026-08** | 바퀴 마찰·캐스터 모델 수정으로 정지 드리프트 해소. 가상 건물 왕복 시뮬 10/10 통과, PC Gazebo + Jetson 분산 부하 시험 3/3 통과(8/21) |
| **2026-09** | OpenCR 구동 브리지·`nav_safety_gate`·`fieldctl` 정비. 신공학관 F1·F2 SLAM 지도 작성 및 실물 Nav2 구간 시험(9/16), 4 DOF 로봇팔·버튼 인식 시험 |
| **2026-10** | 신공학관 실측 지도 Gazebo E2E 시뮬(10/08), 현장·시뮬 공용 자율 스킬(`apps/jetson_skill_tests/`), 팔레트 집기(`apps/pickup_real/`), 26단계 E2E 리허설 진행기 및 `lift_uno` 초음파 높이 제한 펌웨어(10/29) 반영 |
| **2026-11** | 강의실 v3 배치도 시뮬 반복 검증(11/04, 4,112회) 및 최종 발표. 보고서 제출용 공개 저장소 `PackagU/PackagU` 정리(11/19) |

## 팀원 (건국대학교 전기전자공학부 2026 종합설계 5조)

- 이준형 ([@JunhyungLee25](https://github.com/JunhyungLee25)): SLAM, Nav2, 층별 지도 전환, 시뮬레이션, 시스템 통합
- 한수민 ([@inonewater](https://github.com/inonewater)): Fusion 360 기구 설계, 구동부, Z축 지게팔
- 이성덕 ([@DuckFrog123](https://github.com/DuckFrog123)): 4 DOF 로봇팔, 엘리베이터 버튼 인식 및 조작

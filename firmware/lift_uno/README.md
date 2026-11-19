# 지게팔(Z축 리프트) 펌웨어 — `lift_uno`

`lift_uno.ino` 는 2026-10-29 사용자가 초음파를 붙여 실물 테스트를 마친 코드를 그대로 옮긴 것이다. Jetson 에 연결된 지게팔 보드(FT232R USB, ATmega328P)의 플래시를 읽어, 출력 문자열 13개와 펄스 시간(HIGH 10 µs, LOW 302 µs), 12800 펄스, 320 펄스마다 거리 확인, 10 cm 정지 기준이 이 소스와 같음을 확인했다.

## 1. 배선

| 기능 | 핀 |
|---|---|
| TB6600 PUL | D7 |
| TB6600 DIR | D6 (HIGH = 상승) |
| TB6600 ENA | D5 (HIGH 유지) |
| 초음파 TRIG | D9 |
| 초음파 ECHO | D10 |

## 2. 동작

- 기본 코드(9/13 `uno_user_reference_20260913.ino`)와 같은 방식이다. `u`·`d` 한 번에 12800 펄스를 보낸다.
- 펄스 LOW 시간은 365 µs 에서 302 µs 로 줄였다. 12800 펄스가 약 4.8 초에서 4.0 초로 빨라졌고, 사용자가 이 속도를 유지하기로 했다.
- 상승(`u`): 시작 전 약 100 ms 를 기다린 뒤 거리를 재고, 320 펄스(약 100 ms)마다 다시 잰다. 거리가 0 cm 초과 10 cm 미만이면 그 자리에서 멈춘다(박스 높이 제한). 응답이 없으면(30 ms 안에 에코 없음) 멈추지 않는다.
- 하강(`d`): 초음파를 보지 않는다.
- 대기 중에는 초음파를 재지 않는다.

## 3. 시리얼 약속 (115200 baud)

Jetson 쪽 코드(`scripts/field_video/backends.py`, `scripts/lift_uno_position_terminal.py`, `scripts/lift_uno_adapter.py`)는 아래 줄을 그대로 기다린다. 문구를 바꾸면 `scripts/test_lift_uno_contract.py` 가 실패한다.

| 때 | 보드가 보내는 줄 |
|---|---|
| 전원·포트 열기(리셋) | `Motor ready`, `u: Up (ultrasonic stop)`, `d: Down (no ultrasonic stop)`, `Sensor: Up only; No echo does not stop Up`, `Up stop distance: 10.0 cm` |
| `u` 받음 | `Up start` |
| 상승 중 거리 측정 | `Echo: <µs> us \| Distance: <cm> cm` (응답이 있을 때만) |
| 상승 끝 | `Up complete` |
| 높이 제한 정지 | `Up stopped: distance below stop threshold` (이때 `Up complete` 는 없다) |
| `d` 받음 / 하강 끝 | `Down start` / `Down complete` |

## 4. 알아 둘 한계

- 정지·상태 명령이 없다. 이동 중에는 시리얼을 읽지 않으므로 Jetson 이 멈출 수 없다. 이동은 최대 약 6 초 안에 펌웨어가 스스로 끝낸다. 위험하면 물리 비상정지를 쓴다.
- 위치를 기억하지 않는다. 높이 제한으로 덜 올라간 뒤 `d` 를 보내면 12800 펄스를 다 내려서 아래 끝을 지나칠 수 있다. 진행기는 이 경우 내리기 전에 운영자 확인을 받는다.
- 이동 중 들어온 `u`·`d` 는 버퍼에 남았다가 이동이 끝난 뒤 실행된다. Jetson 쪽 코드는 끝 줄을 보기 전에는 다음 명령을 보내지 않는다.
- 포트를 열면 보드가 리셋된다(약 2 초 뒤 부팅 안내 줄).

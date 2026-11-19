#include <Arduino.h>

int PUL = 7;
int DIR = 6;
int ENA = 5;

const byte TRIG_PIN = 9;
const byte ECHO_PIN = 10;

// 이 거리 미만이면 상승 중단
const float STOP_DISTANCE_CM = 10.0;

// 상승 중 320펄스마다 거리 확인
// 현재 펄스 설정에서 출력 시간 기준 약 100ms
const unsigned int UP_CHECK_EVERY_STEPS = 320;

void moveMotor(unsigned int steps);
void readCountedDown(char value);
bool readingDown = false;
unsigned int downSteps = 0;
bool invalidDown = false;
unsigned int upSteps = 0;
bool moveMotorUp();
bool checkUpDistance();
float ultrasonic();

void setup()
{
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(PUL, LOW);
  digitalWrite(ENA, HIGH);

  Serial.begin(115200);

  Serial.println("Motor ready");
  Serial.println("Counted lift v1");
  Serial.println("u: Up (ultrasonic stop)");
  Serial.println("d: Down (no ultrasonic stop)");
  Serial.println("Sensor: Up only; No echo does not stop Up");

  Serial.print("Up stop distance: ");
  Serial.print(STOP_DISTANCE_CM, 1);
  Serial.println(" cm");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);
}

void loop()
{
  if (Serial.available() > 0)
  {
    char command = Serial.read();

    if (readingDown)
    {
      readCountedDown(command);
    }
    else if (command == 'x')
    {
      readingDown = true;
      downSteps = 0;
      invalidDown = false;
    }
    else if (command == 'd' || command == 'D')
    {
      Serial.println("Down start");

      digitalWrite(DIR, LOW);
      delayMicroseconds(20);

      moveMotor(12800);

      Serial.println("Down complete");
    }
    else if (command == 'u' || command == 'U')
    {
      Serial.println("Up start");

      digitalWrite(DIR, HIGH);
      delayMicroseconds(20);

      upSteps = 0;
      if (moveMotorUp())
      {
        Serial.println("Up complete");
      }
      Serial.print("Up pulses: ");
      Serial.println(upSteps);
    }
  }

  // 대기 중에는 초음파를 측정하지 않음
}

// 하강: 초음파 확인 없이 기존대로 이동
void readCountedDown(char value)
{
  if (value == '\n')
  {
    readingDown = false;
    if (invalidDown || downSteps == 0)
    {
      Serial.println("Down pulses rejected");
      return;
    }
    digitalWrite(DIR, LOW);
    delayMicroseconds(20);
    moveMotor(downSteps);
    Serial.print("Down pulses: ");
    Serial.println(downSteps);
    return;
  }

  if (value < '0' || value > '9' || downSteps > 1280)
  {
    invalidDown = true;
    return;
  }
  downSteps = downSteps * 10 + value - '0';
  if (downSteps > 12800)
    invalidDown = true;
}

void moveMotor(unsigned int steps)
{
  for (unsigned int i = 0; i < steps; i++)
  {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(10);

    digitalWrite(PUL, LOW);
    delayMicroseconds(302);
  }
}

// 상승: 시작 전과 이동 중에 거리 확인
bool moveMotorUp()
{
  // Up 명령이 연속으로 들어와도
  // 초음파 측정 간격을 확보하도록 약 100ms 대기
  for (unsigned int waitMs = 0; waitMs < 100; waitMs++)
  {
    delayMicroseconds(1000);
  }

  // 움직이기 전에 먼저 확인
  if (!checkUpDistance())
  {
    return false;
  }

  unsigned int stepsSinceCheck = 0;

  for (int i = 0; i < 12800; i++)
  {
    // 320펄스마다 거리 확인
    if (stepsSinceCheck >= UP_CHECK_EVERY_STEPS)
    {
      if (!checkUpDistance())
      {
        return false;
      }

      stepsSinceCheck = 0;
    }

    digitalWrite(PUL, HIGH);
    delayMicroseconds(10);

    digitalWrite(PUL, LOW);
    delayMicroseconds(302);

    stepsSinceCheck++;
    upSteps++;
  }

  return true;
}

// 상승을 계속할 수 있는지 판단
bool checkUpDistance()
{
  float distanceCm = ultrasonic();

  // 응답으로 얻은 거리가 10cm 미만일 때만 중단
  // 응답 없음(-1)은 정지 조건에서 제외
  if (distanceCm > 0 && distanceCm < STOP_DISTANCE_CM)
  {
    digitalWrite(PUL, LOW);

    Serial.println("Up stopped: distance below stop threshold");

    return false;
  }

  return true;
}

// 거리 측정
// 응답이 있으면 계산한 거리(cm), 없으면 -1 반환
float ultrasonic()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0)
  {
    // 응답이 없으면 메시지 없이 상승 계속
    return -1.0;
  }

  float distanceCm = duration / 58.0;

  Serial.print("Echo: ");
  Serial.print(duration);
  Serial.print(" us | Distance: ");
  Serial.print(distanceCm, 2);
  Serial.println(" cm");

  return distanceCm;
}

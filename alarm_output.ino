/*
 * FLOODGUARD - 출력/경보 담당 Arduino
 *
 * Raspberry Pi -> Arduino Serial Protocol
 *
 * IDLE
 * LOW
 * MID
 * HIGH
 * ESCAPE
 *
 * Serial baud rate: 9600
 *
 * Opening-delay measurement extension:
 * MPU6050 is mounted on the foam board and continuously emits only detected
 * motion events as MOTION,<sequence>,<arduino_micros>,<delta_g>.
 * The Pi test scripts decide whether an event belongs to a relay pulse by
 * accepting only events received after the Pi has turned the relay on.
 */

#include <Wire.h>

// =====================================================
// 핀 설정
// =====================================================

const int BUZZER_PIN = 8;

// RGB LED
const int LED_R_PIN = 6;
const int LED_G_PIN = 9;
const int LED_B_PIN = 5;

// 진동 모터
const int MOTOR_PIN = 10;

// 공통 캐소드 = false
const bool LED_COMMON_ANODE = false;

// =====================================================
// Foam-board MPU6050: Uno SDA=A4, SCL=A5
// =====================================================

const byte MPU6050_ADDRESS = 0x68;
const byte MPU6050_ACCEL_REGISTER = 0x3B;
const float MOTION_DELTA_G = 0.06f;
const byte MOTION_CONFIRM_SAMPLES = 3;
const unsigned long IMU_SAMPLE_INTERVAL_MS = 5;

float baselineAx = 0.0f;
float baselineAy = 0.0f;
float baselineAz = 1.0f;
unsigned long lastImuSampleMs = 0;
unsigned long motionSequence = 0;
byte motionConfirmCount = 0;
bool motionLatched = false;

// =====================================================
// 상태 정의
// =====================================================

enum State {
  IDLE,
  LOW_RISK,
  MID_RISK,
  HIGH_RISK,
  ESCAPE
};

State currentState = IDLE;

// =====================================================
// 타이밍 변수
// =====================================================

unsigned long lastToggleTime = 0;

bool blinkOn = false;

// =====================================================
// MPU6050 helpers
// =====================================================

void writeMpuRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t joinInt16(byte highByte, byte lowByte) {
  return (int16_t)(((int16_t)highByte << 8) | lowByte);
}

bool readMpuAcceleration(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL_REGISTER);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU6050_ADDRESS, (byte)6) != 6) {
    return false;
  }
  int16_t rawAx = joinInt16(Wire.read(), Wire.read());
  int16_t rawAy = joinInt16(Wire.read(), Wire.read());
  int16_t rawAz = joinInt16(Wire.read(), Wire.read());
  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;
  return true;
}

bool calibrateMpuBaseline() {
  const int samples = 200;
  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az;
    if (readMpuAcceleration(ax, ay, az)) {
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      validSamples++;
    }
    delay(5);
  }
  if (validSamples < samples / 2) {
    return false;
  }
  baselineAx = sumAx / validSamples;
  baselineAy = sumAy / validSamples;
  baselineAz = sumAz / validSamples;
  return true;
}

void reportFoamBoardMotion() {
  unsigned long nowMs = millis();
  if (nowMs - lastImuSampleMs < IMU_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastImuSampleMs = nowMs;

  float ax, ay, az;
  if (!readMpuAcceleration(ax, ay, az)) {
    return;
  }
  float dx = ax - baselineAx;
  float dy = ay - baselineAy;
  float dz = az - baselineAz;
  float deltaG = sqrt(dx * dx + dy * dy + dz * dz);

  if (deltaG >= MOTION_DELTA_G) {
    if (motionConfirmCount < MOTION_CONFIRM_SAMPLES) {
      motionConfirmCount++;
    }
    if (!motionLatched && motionConfirmCount >= MOTION_CONFIRM_SAMPLES) {
      motionLatched = true;
      motionSequence++;
      Serial.print("MOTION,");
      Serial.print(motionSequence);
      Serial.print(",");
      Serial.print(micros());
      Serial.print(",");
      Serial.println(deltaG, 4);
    }
  } else {
    motionConfirmCount = 0;
    if (deltaG < MOTION_DELTA_G * 0.5f) {
      motionLatched = false;
    }
  }
}

// =====================================================
// LED 채널 출력
// =====================================================

void writeLedChannel(int pin, int value) {

  int output;

  if (LED_COMMON_ANODE) {
    output = 255 - value;
  } 
  else {
    output = value;
  }

  analogWrite(pin, constrain(output, 0, 255));
}

// =====================================================
// RGB 색상 설정
// =====================================================

void setColor(int r, int g, int b) {

  writeLedChannel(LED_R_PIN, r);
  writeLedChannel(LED_G_PIN, g);
  writeLedChannel(LED_B_PIN, b);
}

// =====================================================
// 모든 출력 OFF
// =====================================================

void allOff() {

  noTone(BUZZER_PIN);

  setColor(0, 0, 0);

  analogWrite(MOTOR_PIN, 0);
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(20);

  Wire.begin();
  writeMpuRegister(0x6B, 0x00);  // wake MPU6050

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  pinMode(MOTOR_PIN, OUTPUT);

  // 시작할 때 모든 출력 OFF
  allOff();

  lastToggleTime = millis();

  if (calibrateMpuBaseline()) {
    Serial.println("IMU_READY");
  } else {
    Serial.println("IMU_ERROR");
  }
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // Raspberry Pi 명령 확인
  readSerialCommand();

  // This never controls the relay. It only reports foam-board motion.
  reportFoamBoardMotion();

  // 현재 상태에 따른 출력
  switch (currentState) {

    case IDLE:
      handleIdle();
      break;

    case LOW_RISK:
      handleLow();
      break;

    case MID_RISK:
      handleMid();
      break;

    case HIGH_RISK:
      handleHigh();
      break;

    case ESCAPE:
      handleEscape();
      break;
  }
}

// =====================================================
// Raspberry Pi 명령 수신
// =====================================================

void readSerialCommand() {

  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');

  cmd.trim();

  State newState = currentState;

  if (cmd == "IDLE") {

    newState = IDLE;
  }

  else if (cmd == "LOW") {

    newState = LOW_RISK;
  }

  else if (cmd == "MID") {

    newState = MID_RISK;
  }

  else if (cmd == "HIGH") {

    newState = HIGH_RISK;
  }

  else if (cmd == "ESCAPE") {

    newState = ESCAPE;
  }

  else {

    // 알 수 없는 명령
    return;
  }

  // 상태가 변경된 경우
  if (newState != currentState) {

    currentState = newState;

    // 이전 상태의 출력 제거
    allOff();

    // 새로운 상태 타이머 초기화
    lastToggleTime = millis();

    blinkOn = false;
  }
}

// =====================================================
// IDLE
// 모든 출력 OFF
// =====================================================

void handleIdle() {

  allOff();
}

// =====================================================
// LOW
//
// 노란색 고정
// 진동 없음
// 1초마다 짧은 경고음
// =====================================================

void handleLow() {

  // 노란색
  setColor(255, 200, 0);

  // 진동 OFF
  analogWrite(MOTOR_PIN, 0);

  // 1초마다 경고음
  if (millis() - lastToggleTime >= 1000) {

    tone(BUZZER_PIN, 1500, 150);

    lastToggleTime = millis();
  }
}

// =====================================================
// MID
//
// 주황색 점멸
// 강한 경고음
// 진동
// =====================================================

void handleMid() {

  const unsigned long interval = 400;

  if (millis() - lastToggleTime >= interval) {

    blinkOn = !blinkOn;

    lastToggleTime = millis();

    if (blinkOn) {

      // 주황색
      setColor(255, 100, 0);

      // 경고음
      tone(BUZZER_PIN, 2500, 300);

      // 진동
      analogWrite(MOTOR_PIN, 200);
    }

    else {

      // LED OFF
      setColor(0, 0, 0);

      // 진동 OFF
      analogWrite(MOTOR_PIN, 0);
    }
  }
}

// =====================================================
// HIGH
//
// 빨간색 빠른 점멸
// 최대 진동
// 지속 경보음
// =====================================================

void handleHigh() {

  const unsigned long interval = 150;

  if (millis() - lastToggleTime >= interval) {

    blinkOn = !blinkOn;

    lastToggleTime = millis();

    if (blinkOn) {

      // 밝은 빨간색
      setColor(255, 0, 0);

      // 3kHz 경보음
      tone(BUZZER_PIN, 3000);
    }

    else {

      // 약한 빨간색
      setColor(80, 0, 0);

      // 부저 OFF
      noTone(BUZZER_PIN);
    }
  }

  // 진동 최대
  analogWrite(MOTOR_PIN, 255);
}

// =====================================================
// ESCAPE
//
// 매우 빠른 빨간색 스트로브
// 최대 진동
// 지속 경보음
// =====================================================

void handleEscape() {

  const unsigned long interval = 80;

  if (millis() - lastToggleTime >= interval) {

    blinkOn = !blinkOn;

    lastToggleTime = millis();

    if (blinkOn) {

      // 빨간색 ON
      setColor(255, 0, 0);
    }

    else {

      // LED OFF
      setColor(0, 0, 0);
    }
  }

  // 3.5kHz 지속 경보
  tone(BUZZER_PIN, 3500);

  // 진동 최대
  analogWrite(MOTOR_PIN, 255);
}

/*
 * FLOODGUARD - 출력/경보 담당 (아두이노)
 * 출처: 승현(C) "C. 출력/경보 담당" 페이지의 alarm_output.ino 원본.
 * 로직(논블로킹 millis() 기반 부저/LED/진동)은 승현 원본 그대로다.
 * 2026-08-25: "수압차 로직" 공식 페이지(서연) 기준으로 정리함.
 *
 * !! 핀 번호 수정함 !!
 * 승현 원본: LED_R_PIN=5, LED_G_PIN=6, LED_B_PIN=3
 * HW 구조도.pdf 실측 + 서연이 확정한 배선: R=2, G=3, B=4
 * (부저=9, 진동모터=10은 원본과 실측이 일치해서 그대로 둠)
 * 원본 그대로 업로드하면 실제 배선과 안 맞아 LED가 안 켜지거나
 * 엉뚱한 색이 나올 수 있다. 아래는 수정된 버전이다.
 * -> 승현에게 확인: 원본은 어떤 배선 기준으로 작성한 것인지?
 *
 * !! 프로토콜 정리 (2026-08-25) !!
 * 처음에는 "수압차 로직" 페이지(IDLE/LOW/MID/HIGH 4단계만 설명)
 * 기준으로 ESCAPE를 제거했었는데, 이후 "Pi_아두이노_시리얼프로토콜"
 * 공식 문서에 ESCAPE 상태를 정식으로 추가하기로 해서 다시 넣었다.
 * (문서: 탈출 유도 단계, HIGH 이후 실제 탈출을 안내하는 단계)
 * 1) ESCAPE 상태와 handleEscape() - 승현 원본 로직 그대로 복원함.
 *    2026-08-25 갱신: HIGH->ESCAPE 전환 기준도 확정됨 (서연 결정,
 *    README_불일치보고서.md 9번 참고) - 창문이 실제로 열리는 순간
 *    fsm_controller.escalate_to_escape()가 호출되고,
 *    serial_sender.send_state("ESCAPE")로 이 아두이노까지 실제로
 *    전송된다. 더 이상 "Pi 쪽 로직 없음" 상태가 아니다.
 * 2) 상태 전환 시 "ACK:<state>\n"를 Pi로 되돌려 보내는 응답은 계속
 *    제거된 상태다 - 공식 프로토콜(Pi->Arduino 단방향, 응답 없음)에
 *    없고 Pi 쪽도 읽지 않기 때문. ESCAPE 추가와는 별개 사안.
 *
 * 시리얼 프로토콜: 라즈베리파이 -> 아두이노, 개행문자('\n')로 끝나는 문자열
 * 예) "IDLE\n", "LOW\n", "MID\n", "HIGH\n", "ESCAPE\n"
 *
 * !! "Pi_아두이노_시리얼프로토콜" 공식 페이지와 다른 점 (참고용, 오류 아님) !!
 * 그 문서의 예제 코드는 digitalWrite()로 LED를 단순 ON/OFF
 * (LOW=초록, MID=노랑, HIGH=빨강 고정)하고 부저도 digitalWrite로
 * 켠다. 승현(C)의 이 코드는 그보다 훨씬 정교하다 - 단계별 점멸
 * 패턴, tone()으로 주파수를 바꾼 경보음, analogWrite로 진동 세기
 * 조절까지 들어가 있다. 프로토콜(메시지 형식)은 동일해서 호환에는
 * 문제 없지만, 공식 문서의 "주의: 부저가 수동(passive) 타입이면
 * digitalWrite 대신 tone()/noTone()을 써야 한다"는 지적을 C의
 * 코드는 이미 tone()/noTone()으로 반영하고 있어 오히려 문서의
 * 기본 예제보다 더 안전한 버전이다.
 */

// ===== 핀 설정 =====
const int BUZZER_PIN = 9;      // 부저 (PWM 가능 핀, tone() 사용) - 실측과 일치
const int LED_R_PIN = 2;       // RGB LED - Red   (수정: 5 -> 2, 실측 반영)
const int LED_G_PIN = 3;       // RGB LED - Green (수정: 6 -> 3, 실측 반영)
const int LED_B_PIN = 4;       // RGB LED - Blue  (수정: 3 -> 4, 실측 반영)
const int MOTOR_PIN = 10;      // 진동모터 (PWM, 트랜지스터/모터드라이버 경유) - 실측과 일치

// ===== 상태 정의 (Pi_아두이노_시리얼프로토콜 문서 2026-08-25 갱신 기준) =====
enum State { IDLE, LOW_RISK, MID_RISK, HIGH_RISK, ESCAPE };
State currentState = IDLE;

// ===== 비차단(non-blocking) 타이밍용 변수 =====
unsigned long lastToggleTime = 0;
bool blinkOn = false;

// 색상 헬퍼: common-cathode 기준 (common-anode면 255-value로 반전)
void setColor(int r, int g, int b) {
  analogWrite(LED_R_PIN, r);
  analogWrite(LED_G_PIN, g);
  analogWrite(LED_B_PIN, b);
}

void allOff() {
  noTone(BUZZER_PIN);
  setColor(0, 0, 0);
  analogWrite(MOTOR_PIN, 0);
}

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  allOff();
}

void loop() {
  readSerialCommand();

  switch (currentState) {
    case IDLE:       handleIdle();   break;
    case LOW_RISK:    handleLow();    break;
    case MID_RISK:    handleMid();    break;
    case HIGH_RISK:   handleHigh();   break;
    case ESCAPE:      handleEscape(); break;
  }
}

// ===== 시리얼 명령 수신 =====
void readSerialCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  State newState = currentState;
  if (cmd == "IDLE") newState = IDLE;
  else if (cmd == "LOW") newState = LOW_RISK;
  else if (cmd == "MID") newState = MID_RISK;
  else if (cmd == "HIGH") newState = HIGH_RISK;
  else if (cmd == "ESCAPE") newState = ESCAPE;
  else return; // 알 수 없는 명령은 무시

  if (newState != currentState) {
    currentState = newState;
    allOff();          // 상태 전이 시 잔여 출력 초기화
    lastToggleTime = millis();
    blinkOn = false;
  }
}

// ===== 단계별 동작 =====

// IDLE: 모든 출력 정지
void handleIdle() {
  allOff();
}

// LOW: 부저·LED 1차 경고 (노란색, 저강도 단발 경고음)
void handleLow() {
  setColor(255, 200, 0); // 노란색 고정
  analogWrite(MOTOR_PIN, 0); // 진동 없음

  if (millis() - lastToggleTime >= 1000) {
    tone(BUZZER_PIN, 1500, 150); // 1.5kHz, 150ms
    lastToggleTime = millis();
  }
}

// MID: 강한 경고음 + 진동모터 동작 (주황색 점멸)
void handleMid() {
  unsigned long interval = 400; // 점멸 주기
  if (millis() - lastToggleTime >= interval) {
    blinkOn = !blinkOn;
    lastToggleTime = millis();

    if (blinkOn) {
      setColor(255, 100, 0);       // 주황색
      tone(BUZZER_PIN, 2500, 300); // 강한 경고음
      analogWrite(MOTOR_PIN, 200); // 진동 ON
    } else {
      setColor(0, 0, 0);
      analogWrite(MOTOR_PIN, 0);   // 진동 OFF (점멸형 패턴)
    }
  }
}

// HIGH: 최대 경보 (빨간색 빠른 점멸, 최대 진동, 연속 경보음)
void handleHigh() {
  unsigned long interval = 150; // 빠른 점멸
  if (millis() - lastToggleTime >= interval) {
    blinkOn = !blinkOn;
    lastToggleTime = millis();

    if (blinkOn) {
      setColor(255, 0, 0);
      tone(BUZZER_PIN, 3000);      // 지속음 (duration 생략 = 계속 울림)
    } else {
      setColor(80, 0, 0);          // 완전히 끄지 않고 은은하게(잔상 효과)
      noTone(BUZZER_PIN);
    }
  }
  analogWrite(MOTOR_PIN, 255); // 진동 최대 지속
}

// ESCAPE: 탈출 유도 - 레드 스트로브 + 최대 진동 + 경보음 지속
// 2026-08-25: Pi_아두이노_시리얼프로토콜 문서에 정식 추가되어 복원함.
// 전환 기준도 확정되어 main.py -> fsm_controller.escalate_to_escape()
// -> serial_sender가 실제로 "ESCAPE\n"을 보낸다 (위 헤더 주석 참고).
void handleEscape() {
  unsigned long interval = 80; // 매우 빠른 스트로브
  if (millis() - lastToggleTime >= interval) {
    blinkOn = !blinkOn;
    lastToggleTime = millis();
    setColor(blinkOn ? 255 : 0, 0, 0);
  }
  tone(BUZZER_PIN, 3500);
  analogWrite(MOTOR_PIN, 255);
}

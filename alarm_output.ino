const int BUZZER_PIN = 9;
const int LED_R_PIN = 2;
const int LED_G_PIN = 3;
const int LED_B_PIN = 4;
const int MOTOR_PIN = 10;

enum State { IDLE, LOW_RISK, MID_RISK, HIGH_RISK, ESCAPE };
State currentState = IDLE;

unsigned long lastToggleTime = 0;
bool blinkOn = false;

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
    case IDLE: handleIdle(); break;
    case LOW_RISK: handleLow(); break;
    case MID_RISK: handleMid(); break;
    case HIGH_RISK: handleHigh(); break;
    case ESCAPE: handleEscape(); break;
  }
}

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
  else return;

  if (newState != currentState) {
    currentState = newState;
    allOff();
    lastToggleTime = millis();
    blinkOn = false;
  }
}

void handleIdle() {
  allOff();
}

void handleLow() {
  setColor(255, 200, 0);
  analogWrite(MOTOR_PIN, 0);
  if (millis() - lastToggleTime >= 1000) {
    tone(BUZZER_PIN, 1500, 150);
    lastToggleTime = millis();
  }
}

void handleMid() {
  unsigned long interval = 400;
  if (millis() - lastToggleTime >= interval) {
    blinkOn = !blinkOn;
    lastToggleTime = millis();
    if (blinkOn) {
      setColor(255, 100, 0);
      tone(BUZZER_PIN, 2500, 300);
      analogWrite(MOTOR_PIN, 200);
    } else {
      setColor(0, 0, 0);
      analogWrite(MOTOR_PIN, 0);
    }
  }
}

void handleHigh() {
  unsigned long interval = 150;
  if (millis() - lastToggleTime >= interval) {
    blinkOn = !blinkOn;
    lastToggleTime = millis();
    if (blinkOn) {
      setColor(255, 0, 0);
      tone(BUZZER_PIN, 3000);
    } else {
      setColor(80, 0, 0);
      noTone(BUZZER_PIN);
    }
  }
  analogWrite(MOTOR_PIN, 255);
}

void handleEscape() {
  unsigned long interval = 80;
  if (millis() - lastToggleTime >= interval) {
    blinkOn = !blinkOn;
    lastToggleTime = millis();
    setColor(blinkOn ? 255 : 0, 0, 0);
  }
  tone(BUZZER_PIN, 3500);
  analogWrite(MOTOR_PIN, 255);
}

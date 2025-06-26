#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Encoder ====
#define ENCODER_A 26
#define ENCODER_B 25
#define ENCODER_B_A 14
#define ENCODER_B_B 13

// ==== Motor ====
#define MOTOR_PWM 5
#define MOTOR_IN1 19
#define MOTOR_IN2 18
#define MOTOR_B_PWM 23
#define MOTOR_B_IN1 21
#define MOTOR_B_IN2 22

// ==== Buttons ====
#define BUTTON_P1 4
#define BUTTON_P2 0
#define BUTTON_P3 33
#define BUTTON_P4 32

// ==== HC-SR04 ====
#define TRIG_PIN 15
#define ECHO_PIN 27

// ==== PID Params ====
float KpA = 0.0001, KiA = 0.05, KdA = 0.001;
float lastErrorA = 0, integralA = 0;

float KpB = 0.0001, KiB = 0.05, KdB = 0.001;
float lastErrorB = 0, integralB = 0;

float setpoint = 0.0;

// ==== Fuzzy ====
float targetDistance = 20;
bool stopMotorInFuzzy = false;

// ==== Encoder ====
volatile int encoderCount = 0;
volatile int encoderCountB = 0;
bool lastStateB, lastStateBB;
float encoder_resolution = 96;
float gear_ratio = 1.0;
float expectedPulses = encoder_resolution * gear_ratio;

// ==== Timing ====
unsigned long lastTime = 0;
unsigned long lastButtonTime = 0;
const unsigned long debounceDelay = 200;
int mode = 1;

// ==== Encoder ISR ====
void IRAM_ATTR readEncoder() {
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);
  encoderCount += (stateA != lastStateB) ? -1 : 1;
  lastStateB = stateB;
}

void IRAM_ATTR readEncoderB() {
  int stateA = digitalRead(ENCODER_B_A);
  int stateB = digitalRead(ENCODER_B_B);
  encoderCountB += (stateA != lastStateBB) ? -1 : 1;
  lastStateBB = stateB;
}

// ==== Read Distance ====
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  return constrain(distance, 0, 500);
}

// ==== Motor PWM ====
void setMotorPWM(int pwmA, int pwmB) {
  bool forwardA = pwmA >= 0;
  bool forwardB = pwmB >= 0;

  digitalWrite(MOTOR_IN1, forwardA);
  digitalWrite(MOTOR_IN2, !forwardA);
  analogWrite(MOTOR_PWM, abs(pwmA));

  digitalWrite(MOTOR_B_IN1, !forwardB);
  digitalWrite(MOTOR_B_IN2, forwardB);
  analogWrite(MOTOR_B_PWM, abs(pwmB));
}

void setup() {
  Serial.begin(115200);

  Wire.begin(16, 17);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Init");

  pinMode(BUTTON_P1, INPUT_PULLUP);
  pinMode(BUTTON_P2, INPUT_PULLUP);
  pinMode(BUTTON_P3, INPUT_PULLUP);
  pinMode(BUTTON_P4, INPUT_PULLUP);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_B_A, INPUT_PULLUP);
  pinMode(ENCODER_B_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_A), readEncoderB, CHANGE);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastTime = millis();

  // Gửi dòng tiêu đề cho Python
  Serial.println("errorA,errorB,errorDistance");
}

void loop() {
  if (millis() - lastButtonTime > debounceDelay) {
    if (digitalRead(BUTTON_P1) == LOW) {
      if (mode == 1) setpoint = 0;
      else stopMotorInFuzzy = true;
      lastButtonTime = millis();
    }

    if (digitalRead(BUTTON_P2) == LOW) {
      mode = (mode == 1) ? 2 : 1;
      stopMotorInFuzzy = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MODE: ");
      lcd.print(mode == 1 ? "PID" : "Fuzzy");
      lastButtonTime = millis();
    }

    if (digitalRead(BUTTON_P3) == LOW) {
      if (mode == 1) setpoint += 500;
      else targetDistance += 5;
      lastButtonTime = millis();
    }

    if (digitalRead(BUTTON_P4) == LOW) {
      if (mode == 1) setpoint -= 500;
      else targetDistance -= 5;
      lastButtonTime = millis();
    }
  }

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  if (deltaTime >= 0.1) {
    float rpmA = (encoderCount / expectedPulses) * (60.0 / deltaTime);
    float rpmB = (encoderCountB / expectedPulses) * (60.0 / deltaTime);
    encoderCount = 0;
    encoderCountB = 0;

    int pwmA = 0, pwmB = 0;

    if (mode == 1) {
      float errorA = setpoint - rpmA;
      integralA += errorA * deltaTime;
      float derivativeA = (errorA - lastErrorA) / deltaTime;
      float outputA = KpA * errorA + KiA * integralA + KdA * derivativeA;
      lastErrorA = errorA;

      float errorB = setpoint - rpmB;
      integralB += errorB * deltaTime;
      float derivativeB = (errorB - lastErrorB) / deltaTime;
      float outputB = KpB * errorB + KiB * integralB + KdB * derivativeB;
      lastErrorB = errorB;

      pwmA = constrain(outputA, -255, 255);
      pwmB = constrain(outputB, -255, 255);

      // Gửi dữ liệu PID
      Serial.print(errorA);
      Serial.print(",");
      Serial.print(errorB);
      Serial.println(",");
    } else {
      float currentDistance = readDistanceCM();
      float errorDistance = currentDistance - targetDistance;

      if (stopMotorInFuzzy) {
        pwmA = 0;
        pwmB = 0;
      } else {
        pwmA = pwmB = constrain(errorDistance * 10.0, -255, 255);
      }

      // Gửi dữ liệu Fuzzy
      Serial.print(",");
      Serial.print(",");
      Serial.println(errorDistance);
    }

    setMotorPWM(pwmA, pwmB);

    // LCD hiển thị
    lcd.clear();
    lcd.setCursor(0, 0);
    if (mode == 1) {
      lcd.print("A:");
      lcd.print((int)rpmA);
      lcd.print(" B:");
      lcd.print((int)rpmB);
    } else {
      lcd.print("Fuzzy D:");
      lcd.print((int)readDistanceCM());
    }
    lcd.setCursor(0, 1);
    lcd.print("Set:");
    lcd.print(mode == 1 ? (int)setpoint : (int)targetDistance);

    lastTime = currentTime;
  }
}

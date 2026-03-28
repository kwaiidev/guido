#include <Wire.h>
#include <MPU6050.h>

// ── Motor driver (TB6612 / Elegoo Smart Car Shield v1.1) ─────────
#define PIN_MOTOR_STBY   3
#define MOTOR_A_PWM      5   // PWMA
#define MOTOR_B_PWM      6   // PWMB
#define MOTOR_A_IN1      7   // AIN1
#define MOTOR_A_IN2      4   // AIN2
#define MOTOR_B_IN1      8   // BIN1
#define MOTOR_B_IN2      9   // BIN2

// ── Encoders ────────────────────────────────────────────────────
#define ENC_A  2   // Left  – hardware INT0
#define ENC_B  10  // Right – pin-change IRQ (pin 3 is STBY on this shield)

// ── Constants ───────────────────────────────────────────────────
#define WHEEL_RADIUS      0.033
#define WHEEL_BASE        0.17
#define TICKS_PER_REV     20

#define SWAP_LEFT_RIGHT   0
#define INVERT_LEFT_PWM   0
#define INVERT_RIGHT_PWM  0

// ── Motion test (set to 0 when running with Jetson) ──────────────
#define MOTION_TEST           0
#define MOTION_TEST_PWM       150
#define MOTION_TEST_MS_DRIVE  2000
#define MOTION_TEST_MS_PAUSE  400
#define MOTION_TEST_MS_TURN   1500

// ── Globals ─────────────────────────────────────────────────────
volatile long encA = 0, encB = 0;
volatile uint8_t enc_b_prev = 0;
MPU6050 mpu;
float x = 0, y = 0, theta = 0;
unsigned long lastTime = 0;

// ── Encoder ISRs ────────────────────────────────────────────────
void isrA() { encA++; }

ISR(PCINT0_vect) {
  uint8_t v = digitalRead(ENC_B);
  if (v && !enc_b_prev) encB++;
  enc_b_prev = v;
}

// ── Motor helpers ───────────────────────────────────────────────
void setMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}

void setMotors(int leftSpeed, int rightSpeed) {
#if SWAP_LEFT_RIGHT
  int tmp = leftSpeed; leftSpeed = rightSpeed; rightSpeed = tmp;
#endif
#if INVERT_LEFT_PWM
  leftSpeed = -leftSpeed;
#endif
#if INVERT_RIGHT_PWM
  rightSpeed = -rightSpeed;
#endif
  setMotor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, leftSpeed);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, rightSpeed);
}

// ── Motion test state machine ────────────────────────────────────
#if MOTION_TEST
void motion_test_update() {
  static uint8_t phase = 0;
  static unsigned long phaseStart = 0;
  static uint8_t lastPhase = 255;
  const unsigned long now = millis();
  if (phaseStart == 0) phaseStart = now;

  if (phase != lastPhase) {
    lastPhase = phase;
    switch (phase) {
      case 0: Serial.println(F("MOTION_TEST: forward")); break;
      case 1: Serial.println(F("MOTION_TEST: stop")); break;
      case 2: Serial.println(F("MOTION_TEST: back")); break;
      case 3: Serial.println(F("MOTION_TEST: stop")); break;
      case 4: Serial.println(F("MOTION_TEST: spin right")); break;
      case 5: Serial.println(F("MOTION_TEST: stop")); break;
      case 6: Serial.println(F("MOTION_TEST: spin left")); break;
      case 7: Serial.println(F("MOTION_TEST: stop")); break;
    }
  }

  const int p = MOTION_TEST_PWM;
  unsigned long dur = MOTION_TEST_MS_DRIVE;
  switch (phase) {
    case 0: setMotors(p, p);   dur = MOTION_TEST_MS_DRIVE; break;
    case 2: setMotors(-p, -p); dur = MOTION_TEST_MS_DRIVE; break;
    case 4: setMotors(p, -p);  dur = MOTION_TEST_MS_TURN;  break;
    case 6: setMotors(-p, p);  dur = MOTION_TEST_MS_TURN;  break;
    case 1: case 3: case 5: case 7:
      setMotors(0, 0); dur = MOTION_TEST_MS_PAUSE; break;
    default: phase = 0; phaseStart = now; return;
  }

  if (now - phaseStart >= dur) {
    phase = (phase + 1) % 8;
    phaseStart = now;
    lastPhase = 255;
  }
}
#endif

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(25);

  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT); pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT); pinMode(MOTOR_B_PWM, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, RISING);
  enc_b_prev = digitalRead(ENC_B);
  PCICR  |= bit(PCIE0);
  PCMSK0 |= bit(PCINT4); // D10

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) Serial.println("MPU6050 connection failed");

  lastTime = millis();
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {

#if MOTION_TEST
  motion_test_update();
#else
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "STOP") { setMotors(0, 0); continue; }
    if (!cmd.startsWith("L")) continue;
    int rPos = cmd.indexOf('R');
    if (rPos <= 1) continue;
    String leftStr  = cmd.substring(1, rPos);  leftStr.trim();
    String rightStr = cmd.substring(rPos + 1); rightStr.trim();
    setMotors(leftStr.toInt(), rightStr.toInt());
  }
#endif

  unsigned long now = millis();
  if (now - lastTime >= 50) {
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    noInterrupts();
    long dA = encA; long dB = encB;
    encA = 0; encB = 0;
    interrupts();

    float distA = (2.0 * PI * WHEEL_RADIUS * dA) / TICKS_PER_REV;
    float distB = (2.0 * PI * WHEEL_RADIUS * dB) / TICKS_PER_REV;
    float distCenter = (distA + distB) / 2.0;

    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float gyroZ = gz / 131.0 * (PI / 180.0);
    theta += gyroZ * dt;
    x += distCenter * cos(theta);
    y += distCenter * sin(theta);

    Serial.print("ODOM ");
    Serial.print(x, 4);             Serial.print(" ");
    Serial.print(y, 4);             Serial.print(" ");
    Serial.print(theta, 4);         Serial.print(" ");
    Serial.print(distCenter / dt, 4); Serial.print(" ");
    Serial.println(gyroZ, 4);
  }
}
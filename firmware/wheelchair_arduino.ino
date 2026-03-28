#include <Wire.h>
#include <MPU6050.h>

// ── Motor A (Left) ──────────────────────────────────────────────
#define MOTOR_A_IN1  4
#define MOTOR_A_IN2  5
#define MOTOR_A_PWM  6   // ENA

// ── Motor B (Right) ─────────────────────────────────────────────
#define MOTOR_B_IN1  7
#define MOTOR_B_IN2  8
#define MOTOR_B_PWM  9   // ENB

// ── Encoders ────────────────────────────────────────────────────
#define ENC_A  2   // Left  encoder – interrupt pin
#define ENC_B  3   // Right encoder – interrupt pin

// ── Constants ───────────────────────────────────────────────────
#define WHEEL_RADIUS      0.033   // metres (adjust to your wheel)
#define WHEEL_BASE        0.17    // metres between wheels (adjust)
#define TICKS_PER_REV     20      // encoder ticks per full wheel revolution

// ── Globals ─────────────────────────────────────────────────────
volatile long encA = 0, encB = 0;
MPU6050 mpu;

float x = 0, y = 0, theta = 0;
unsigned long lastTime = 0;

// ── Encoder ISRs ────────────────────────────────────────────────
void isrA() { encA++; }
void isrB() { encB++; }

// ── Motor helpers ───────────────────────────────────────────────
void setMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  setMotor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, leftSpeed);
  setMotor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, rightSpeed);
}

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, RISING);

  // MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  }

  lastTime = millis();
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {

  // ── Read serial command from Jetson ──
  // Format: "L<leftSpeed> R<rightSpeed>\n"  e.g. "L150 R150\n"
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("L") && cmd.indexOf('R') != -1) {
      int spaceIdx = cmd.indexOf(' ');
      int leftSpeed  = cmd.substring(1, spaceIdx).toInt();
      int rightSpeed = cmd.substring(cmd.indexOf('R') + 1).toInt();
      setMotors(leftSpeed, rightSpeed);
    } else if (cmd == "STOP") {
      setMotors(0, 0);
    }
  }

  // ── Publish odometry at ~20 Hz ──
  unsigned long now = millis();
  if (now - lastTime >= 50) {
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Snapshot and reset encoders atomically
    noInterrupts();
    long dA = encA;
    long dB = encB;
    encA = 0;
    encB = 0;
    interrupts();

    // Distance each wheel travelled
    float distA = (2.0 * PI * WHEEL_RADIUS * dA) / TICKS_PER_REV;
    float distB = (2.0 * PI * WHEEL_RADIUS * dB) / TICKS_PER_REV;
    float distCenter = (distA + distB) / 2.0;

    // Read gyro Z for heading
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float gyroZ = gz / 131.0 * (PI / 180.0); // deg/s -> rad/s
    theta += gyroZ * dt;

    // Dead-reckoning position
    x += distCenter * cos(theta);
    y += distCenter * sin(theta);

    // Velocities
    float linearVel  = distCenter / dt;
    float angularVel = gyroZ;

    // Send to Jetson:
    // "ODOM x y theta linearVel angularVel\n"
    Serial.print("ODOM ");
    Serial.print(x, 4);       Serial.print(" ");
    Serial.print(y, 4);       Serial.print(" ");
    Serial.print(theta, 4);   Serial.print(" ");
    Serial.print(linearVel, 4); Serial.print(" ");
    Serial.println(angularVel, 4);
  }
}

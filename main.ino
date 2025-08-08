
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include "Adafruit_VL6180X.h"

#define PWM_MOTOR1 4
#define PWM_MOTOR2 5
#define MOTOR1_IN1 15
#define MOTOR1_IN2 21
#define MOTOR2_IN1 22
#define MOTOR2_IN2 23

#define ENCA1 16
#define ENCA2 17
#define ENCB1 18
#define ENCB2 19

#define SDA_PIN 25
#define SCL_PIN 26

// ===== ToF =====
#define MUX_ADDR 0x70
#define NUM_SENSORS 5
const char* TOF_NAME[NUM_SENSORS] = {"Left", "ALeft 45°", "Front", "ARight 45°", "Right"};
const uint8_t TOF_CH[NUM_SENSORS] = {0, 1, 2, 3, 4};
Adafruit_VL6180X tof;

// ===== IMU =====
MPU9250_asukiaaa mpu;
float yawAngle = 0.0;
unsigned long lastIMUms = 0;

// ===== Encoder state =====
volatile long encLeftCount = 0;
volatile long encRightCount = 0;

// ===== Helpers =====
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool tofBeginOnChannel(uint8_t ch) {
  selectMuxChannel(ch);
  delay(3);
  return tof.begin();
}

int16_t readToF(uint8_t ch) {
  selectMuxChannel(ch);
  delay(2);
  uint8_t range = tof.readRange();
  uint8_t status = tof.readRangeStatus();
  return (status == VL6180X_ERROR_NONE) ? (int16_t)range : -1;
}

// ===== Encoder ISRs =====
void IRAM_ATTR isrLeftA() {
  int a = digitalRead(ENCA1);
  int b = digitalRead(ENCA2);
  if (a == b) encLeftCount++; else encLeftCount--;
}

void IRAM_ATTR isrRightA() {
  int a = digitalRead(ENCB1);
  int b = digitalRead(ENCB2);
  if (a == b) encRightCount++; else encRightCount--;
}

// ===== Motor helpers =====
void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(PWM_MOTOR1, speed);
  } else {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    analogWrite(PWM_MOTOR1, -speed);
  }
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    analogWrite(PWM_MOTOR2, speed);
  } else {
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
    analogWrite(PWM_MOTOR2, -speed);
  }
}

void motorsStop() {
  analogWrite(PWM_MOTOR1, 0);
  analogWrite(PWM_MOTOR2, 0);
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
}

// ===== IMU update =====
void imuUpdateYaw() {
  mpu.gyroUpdate();
  float gz = mpu.gyroZ();
  unsigned long now = millis();
  float dt = (now - lastIMUms) / 1000.0;
  lastIMUms = now;
  yawAngle += gz * dt;
}

// ===== Tests =====
void testMotorsAndEncoders() {
  Serial.println("\n[Motor + Encoder Test]");
  encLeftCount = encRightCount = 0;
  int speed = 180;

  unsigned long t0 = millis();
  Serial.println(" Forward...");
  motorLeft(speed);
  motorRight(speed);
  while (millis() - t0 < 5000) delay(1); // 5s forward
  motorsStop();
  Serial.printf(" Counts after FWD: L=%ld  R=%ld\n", encLeftCount, encRightCount);

  delay(500);

  encLeftCount = encRightCount = 0;
  t0 = millis();
  Serial.println(" Reverse...");
  motorLeft(-speed);
  motorRight(-speed);
  while (millis() - t0 < 5000) delay(1); // 5s reverse
  motorsStop();
  Serial.printf(" Counts after REV: L=%ld  R=%ld\n", encLeftCount, encRightCount);
}

void testToF() {
  Serial.println("\n[ToF Sensors Test]");
  for (int i = 0; i < NUM_SENSORS; i++) {
    bool ok = tofBeginOnChannel(TOF_CH[i]);
    Serial.printf(" %s: %s\n", TOF_NAME[i], ok ? "OK" : "NOT FOUND");
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 30000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int16_t d = readToF(TOF_CH[i]);
      if (d >= 0) Serial.printf("%s=%3d  ", TOF_NAME[i], d);
      else        Serial.printf("%s=ERR  ", TOF_NAME[i]);
    }
    Serial.println();
    delay(200);
  }
}

void testIMU() {
  Serial.println("\n[IMU Yaw Test]");
  yawAngle = 0;
  lastIMUms = millis();
  unsigned long t0 = millis();
  while (millis() - t0 < 10000) {
    imuUpdateYaw();
    Serial.printf("Yaw = %.2f deg\n", yawAngle);
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  motorsStop();

  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), isrRightA, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  mpu.setWire(&Wire);
  mpu.beginGyro();
  mpu.beginAccel();
  lastIMUms = millis();

  Serial.println("=== Auto Hardware Check Starting (10s each) ===");

  // Run all tests automatically
  testMotorsAndEncoders();
  testToF();
  testIMU();

  Serial.println("=== Tests Complete ===");
}

void loop() {

}

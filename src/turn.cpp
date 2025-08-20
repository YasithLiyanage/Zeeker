#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <MPU9250_asukiaaa.h>

// --- Motor pins ---
#define PWM_LEFT    4
#define PWM_RIGHT   5
#define IN1_LEFT    15
#define IN2_LEFT    21
#define IN1_RIGHT   22
#define IN2_RIGHT   23

// --- I2C / MUX ---
#define SDA_PIN     25
#define SCL_PIN     26
#define MUX_ADDR    0x70

// MUX channels (adjust to your wiring)
#define CH_LEFT      0
#define CH_ALEFT45   1
#define CH_FRONT     2
#define CH_ARIGHT45  3
#define CH_RIGHT     4

Adafruit_VL6180X tof;
MPU9250_asukiaaa mpu;

// ---- Calibration: mm = a*raw + b ----
struct Cal { float a, b; };
enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };

Cal CAL[5] = {
  {1.00f, -42.0f},  // LEFT
  {1.00f,  20.0f},  // ALEFT45 (update when you refine)
  {1.00f, -35.0f},  // FRONT
  {1.00f,   0.0f},  // ARIGHT45 (update when you refine)
  {1.00f, -28.0f}   // RIGHT
};

// ---- Driving params ----
#define NO_WALL_MM        100   // > this => treat as "no wall"
#define TARGET_SIDE_MM    50
#define FRONT_SLOW_MM     80
#define FRONT_STOP_MM     35

// ---- Square-up params ----
#define A45_TOL_MM        6
#define SQUARE_TURN_PWM   50
#define SQUARE_MAX_MS     1500
#define GAP_TOL_MM        4
#define NUDGE_PWM         60

// ---- IMU state ----
static float gyroBiasZ = 0.0f;

// ---------- Helpers: MUX + ToF ----------
void mux(uint8_t ch){
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}
bool tofBegin(uint8_t ch){ mux(ch); delay(3); return tof.begin(); }

float readMM(SensorId sid, uint8_t ch){
  mux(ch); delay(2);
  uint8_t r = tof.readRange();
  if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
  float mm = CAL[sid].a * r + CAL[sid].b;
  if (mm < 0) mm = 0;
  if (mm > NO_WALL_MM) return -1.0f;
  return mm;
}

// ---------- Motors ----------
void motorL(int pwm){
  pwm = constrain(pwm, -255, 255);
  bool fwd = (pwm >= 0);
  digitalWrite(IN1_LEFT,  fwd ? HIGH : LOW);
  digitalWrite(IN2_LEFT,  fwd ? LOW  : HIGH);
  analogWrite(PWM_LEFT, abs(pwm));
}
void motorR(int pwm){
  pwm = constrain(pwm, -255, 255);
  bool fwd = (pwm >= 0);
  digitalWrite(IN1_RIGHT, fwd ? HIGH : LOW);
  digitalWrite(IN2_RIGHT, fwd ? LOW  : HIGH);
  analogWrite(PWM_RIGHT, abs(pwm));
}
void motorsStop(){ motorL(0); motorR(0); }

// ---------- Wall-follow (PD) ----------
void driveCentered(int basePWM, float err){
  static float err_prev = 0, err_sum = 0;
  static unsigned long last = millis();
  unsigned long now = millis();
  float dt = (now - last) / 1000.0f; if (dt <= 0) dt = 0.001f;
  last = now;

  const float Kp = 0.8f, Ki = 0.0f, Kd = 0.1f;
  err_sum += err * dt;
  float derr = (err - err_prev) / dt;
  float turn = Kp*err + Ki*err_sum + Kd*derr;
  err_prev = err;

  int l = constrain((int)(basePWM + turn), 0, 255);
  int r = constrain((int)(basePWM - turn), 0, 255);
  motorL(l); motorR(r);
}

// ---------- Square-up (use 45° sensors) ----------
void squareUp(){
  Serial.println(">>> Square-up START");
  unsigned long t0 = millis();

  while (millis() - t0 < SQUARE_MAX_MS){
    float al = readMM(SID_ALEFT45,  CH_ALEFT45);
    float ar = readMM(SID_ARIGHT45, CH_ARIGHT45);
    if (al < 0 || ar < 0) break;

    float diff = ar - al;  // positive => rotate left
    if (fabs(diff) <= A45_TOL_MM) break;

    if (diff > 0){  // left is farther -> rotate left
      motorL(-SQUARE_TURN_PWM); motorR(SQUARE_TURN_PWM);
    } else {        // right farther -> rotate right
      motorL(SQUARE_TURN_PWM);  motorR(-SQUARE_TURN_PWM);
    }
    delay(20);
  }
  motorsStop();

  // Set final front gap
  float f = readMM(SID_FRONT, CH_FRONT);
  if (f > 0 && fabs(f - FRONT_STOP_MM) > GAP_TOL_MM){
    int dir = (f > FRONT_STOP_MM) ? +1 : -1; // too far -> forward; too close -> back
    motorL(dir * NUDGE_PWM); motorR(dir * NUDGE_PWM);
    delay(150);
    motorsStop();
  }
  Serial.println(">>> Square-up DONE");
}

// ---------- IMU turn (enable only during turn) ----------
void imuBegin(){
  mpu.setWire(&Wire);
  mpu.beginGyro();
  // bias (keep robot still)
  unsigned long t0 = millis();
  double sum=0; long n=0;
  while (millis()-t0 < 1200){
    mpu.gyroUpdate();
    sum += mpu.gyroZ();
    n++; delay(2);
  }
  gyroBiasZ = (n>0) ? (float)(sum/n) : 0.0f;
  Serial.printf("IMU bias Z=%.3f deg/s\n", gyroBiasZ);
}

// +angleDeg = left/CCW; -angleDeg = right/CW
void turnIMU(float angleDeg, int basePWM=80, int timeout_ms=1500){
  if (angleDeg == 0) return;
  int dir = (angleDeg > 0) ? +1 : -1;
  float target = fabsf(angleDeg);

  float yaw = 0.0f;
  unsigned long last = millis();
  unsigned long t0 = last;

  const float slowBand = 18.0f; // start slowing when within this many degrees
  const int   minPWM   = 40;

  while (true){
    if ((int)(millis()-t0) > timeout_ms) break;

    mpu.gyroUpdate();
    float gz = mpu.gyroZ() - gyroBiasZ; // deg/s
    unsigned long now = millis();
    float dt = (now - last)/1000.0f; if (dt <= 0) dt = 0.001f;
    last = now;

    yaw += fabsf(gz)*dt;
    if (yaw >= target) break;

    float remaining = target - yaw;
    float scale = (remaining < slowBand) ? max(remaining/slowBand, 0.2f) : 1.0f;
    int pwm = max((int)(basePWM*scale), minPWM);

    // in-place spin
    motorL((dir>0)? -pwm : +pwm);
    motorR((dir>0)? +pwm : -pwm);

    delay(2);
  }
  motorsStop();
  delay(60);
}

// ---------- Turn decision (simple left-hand rule) ----------
int decideTurn(float Lmm, float Rmm){
  bool leftOpen  = (Lmm < 0);  // -1 => no wall within NO_WALL_MM
  bool rightOpen = (Rmm < 0);

  if (leftOpen)        return +90;   // prefer left
  else if (rightOpen)  return -90;   // otherwise right
  else                 return 180;   // dead end -> U-turn
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT,OUTPUT);
  pinMode(IN2_RIGHT,OUTPUT);
  motorsStop();

  // ToF sensors
  tofBegin(CH_LEFT);
  tofBegin(CH_ALEFT45);
  tofBegin(CH_FRONT);
  tofBegin(CH_ARIGHT45);
  tofBegin(CH_RIGHT);

  // IMU (ensure robot is still)
  imuBegin();

  Serial.println("Zeeker: drive-centered + square-up + IMU turn (auto loop).");
}

// ---------- Main loop ----------
void loop(){
  float L = readMM(SID_LEFT,  CH_LEFT);
  float F = readMM(SID_FRONT, CH_FRONT);
  float R = readMM(SID_RIGHT, CH_RIGHT);

  int base = 150;

  // lateral error selection
  float err = 0.0f;
  bool leftOK  = (L >= 0.0f);
  bool rightOK = (R >= 0.0f);
  bool frontOK = (F >= 0.0f);

  if (leftOK && rightOK)        err = (L - R);
  else if (leftOK && !rightOK)  err = (L - TARGET_SIDE_MM);
  else if (!leftOK && rightOK)  err = -(R - TARGET_SIDE_MM);
  else                          err = 0.0f;

  // front-wall handling
  if (frontOK && F < FRONT_STOP_MM){
    motorsStop();
    // 1) align
    squareUp();
    delay(150);

    // 2) choose a turn (left-hand rule)
    //    Re-read sides after squaring (a bit more reliable)
    float L2 = readMM(SID_LEFT,  CH_LEFT);
    float R2 = readMM(SID_RIGHT, CH_RIGHT);
    int turnDeg = decideTurn(L2, R2);
    Serial.printf("Turn decision: %d deg (L2=%.1f, R2=%.1f)\n", turnDeg, L2, R2);

    // 3) turn using IMU
    if (turnDeg ==  90) turnIMU(+90, 80, 1600);
    else if (turnDeg == -90) turnIMU(-90, 80, 1600);
    else                     turnIMU(180, 85, 1800);

    // 4) tiny settle, then resume driving
    motorsStop();
    delay(150);
    return;  // resume loop and drive forward
  }

  // slow down when approaching front wall
  if (frontOK && F < FRONT_SLOW_MM) base = 70;

  // drive centered
  driveCentered(base, err);

  // debug
  Serial.printf("L=%.1f R=%.1f F=%.1f err=%.2f base=%d\n", L, R, F, err, base);
  delay(10);
}

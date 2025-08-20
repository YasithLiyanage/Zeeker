#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <MPU9250_asukiaaa.h>
#include <math.h>

// ========== Motor pins ==========
#define PWM_LEFT    4
#define PWM_RIGHT   5
#define IN1_LEFT    15
#define IN2_LEFT    21
#define IN1_RIGHT   22
#define IN2_RIGHT   23

// ========== Encoders (quadrature, A & B) ==========
#define ENCA1 16  // Left encoder A
#define ENCA2 17  // Left encoder B
#define ENCB1 18  // Right encoder A
#define ENCB2 19  // Right encoder B

volatile int posA = 0;           // left ticks
volatile int posB = 0;           // right ticks
volatile int lastEncodedA = 0;   // last AB snapshot (left)
volatile int lastEncodedB = 0;   // last AB snapshot (right)

void IRAM_ATTR updateEncoderA() {
  int msb = digitalRead(ENCA1);
  int lsb = digitalRead(ENCA2);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncodedA << 2) | encoded;

  // Gray code decode
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posA++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posA--;

  lastEncodedA = encoded;
}
void IRAM_ATTR updateEncoderB() {
  int msb = digitalRead(ENCB1);
  int lsb = digitalRead(ENCB2);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncodedB << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posB++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posB--;

  lastEncodedB = encoded;
}

// ----- Buzzer (non-blocking, dedicated LEDC channel) -----
#define BUZZER_PIN 2
#define BUZZ_CH    7      // use a channel not used by your motor PWMs
bool BUZZ_ENABLED = true;

static unsigned long buzz_end_ms = 0;

void buzzerBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZ_CH, 2000 /*Hz*/, 8 /*bits*/);
  ledcAttachPin(BUZZER_PIN, BUZZ_CH);
  ledcWriteTone(BUZZ_CH, 0);     // ensure off
}

// start a single beep without blocking
void buzzStart(int freqHz, int durMs) {
  if (!BUZZ_ENABLED) return;
  ledcWriteTone(BUZZ_CH, freqHz);
  buzz_end_ms = millis() + durMs;
}

// call this every loop() to auto-stop tone
void buzzUpdate() {
  if (buzz_end_ms != 0 && (long)(millis() - buzz_end_ms) >= 0) {
    ledcWriteTone(BUZZ_CH, 0);
    buzz_end_ms = 0;
  }
}

// optional helpers (short/ok and double-beep patterns, non-blocking)
void buzzOK()        { buzzStart(1500, 80); }     // quick chirp
void buzzAttention() { buzzStart(1000, 120); }    // slightly longer


// ========== I2C / MUX ==========
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

// ========== Calibration: mm = a*raw + b ==========
struct Cal { float a, b; };
enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };

Cal CAL[5] = {
  {1.00f, -42.0f},  // LEFT
  {1.00f,  20.0f},  // ALEFT45
  {1.00f, -35.0f},  // FRONT
  {1.00f,   0.0f},  // ARIGHT45
  {1.00f, -28.0f}   // RIGHT
};

// ========== Driving params ==========
#define NO_WALL_MM        100   // > this => treat as "no wall"
#define TARGET_SIDE_MM    50
#define FRONT_SLOW_MM     80
#define FRONT_STOP_MM     35

// ========== Square-up params ==========
#define A45_TOL_MM        6
#define SQUARE_TURN_PWM   50
#define SQUARE_MAX_MS     1500
#define GAP_TOL_MM        4
#define NUDGE_PWM         60

// ========== IMU state ==========
static float gyroBiasZ = 0.0f;

// ======================================================
// ================  MUX + ToF helpers  =================
// ======================================================
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

// ======================================================
/**                     Motors                          */
// ======================================================
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





// ======================================================
// ============  Wall-follow (your PD)  =================
// ======================================================
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

// ======================================================
// ================  Square-up routine  =================
// ======================================================
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

// ======================================================
// ====================  IMU turns  =====================
// ======================================================
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
    float scale = (remaining < slowBand) ? fmaxf(remaining/slowBand, 0.2f) : 1.0f;
    int pwm = (int)fmaxf((basePWM*scale), (float)minPWM);

    // in-place spin
    motorL((dir>0)? -pwm : +pwm);
    motorR((dir>0)? +pwm : -pwm);

    delay(2);
  }
  motorsStop();
  delay(60);
}




// ======================================================
// ============== Optional: one-cell helper =============
// (uses posA/posB; sign-agnostic; NOT called by loop)
// ======================================================
float TICKS_PER_MM_L = 4.00f;   // set after calibration
float TICKS_PER_MM_R = 4.00f;   // set after calibration
int   CELL_MM        = 180;

int ONE_CELL_TICKS_L(){ return (int)roundf(TICKS_PER_MM_L * CELL_MM); }
int ONE_CELL_TICKS_R(){ return (int)roundf(TICKS_PER_MM_R * CELL_MM); }

bool driveOneCell(){
  int startA = posA, startB = posB;

  while ( (abs(posA - startA) < ONE_CELL_TICKS_L()) ||
          (abs(posB - startB) < ONE_CELL_TICKS_R()) ) {

    float L = readMM(SID_LEFT,  CH_LEFT);
    float F = readMM(SID_FRONT, CH_FRONT);
    float R = readMM(SID_RIGHT, CH_RIGHT);

    int base = 100;
    if (F >= 0 && F < FRONT_SLOW_MM) base = 70;

    float err = 0;
    bool leftOK = (L >= 0), rightOK = (R >= 0);
    if (leftOK && rightOK)        err = L - R;
    else if (leftOK && !rightOK)  err = L - TARGET_SIDE_MM;
    else if (!leftOK && rightOK)  err = -(R - TARGET_SIDE_MM);

    if (F >= 0 && F < FRONT_STOP_MM) {
      motorsStop();
      Serial.println("[driveOneCell] Early stop: front wall.");
      return false;
    }

    // small tick balancing (keeps straight if no walls)
    int dA = posA - startA, dB = posB - startB;
    int tickErr = dA - dB;
    base = constrain(base - tickErr * 1, 60, 180);

    driveCentered(base, err);
    delay(10);
  }
  motorsStop();
  Serial.printf("[driveOneCell] Done: posA=%d posB=%d target L=%d R=%d\n",
                posA, posB, ONE_CELL_TICKS_L(), ONE_CELL_TICKS_R());
  return true;
}

// ======================================================
// ====================  Setup  =========================
// ======================================================
void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // motors & buzzer
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT,OUTPUT);
  pinMode(IN2_RIGHT,OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  motorsStop();

  // ToF sensors
  tofBegin(CH_LEFT);
  tofBegin(CH_ALEFT45);
  tofBegin(CH_FRONT);
  tofBegin(CH_ARIGHT45);
  tofBegin(CH_RIGHT);

  // IMU (ensure robot is still)
  imuBegin();

  // Encoders: set INPUT (or INPUT_PULLUP if your encoders are open-collector)
  pinMode(ENCA1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCB2, INPUT);
  // attach both lines for each wheel to the same ISR (CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

  // little boot chirp
buzzerBegin();
buzzOK();   // quick chirp at boot (non-blocking)

  Serial.println("Zeeker: centered-drive + square-up + IMU turns + QUAD encoders + buzzer.");
}

// ======================================================
// ====================  Main loop  =====================
// (unchanged behavior; now also chirps when front wall hit)
// ======================================================
void loop(){
    buzzUpdate();

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
    buzzAttention();   // non-blocking, won’t pause square-up or driving
    motorsStop();

    // 1) align
    squareUp();
    delay(150);

    // 2) choose turn (left-hand rule using openings)
    float L2 = readMM(SID_LEFT,  CH_LEFT);
    float R2 = readMM(SID_RIGHT, CH_RIGHT);
    bool leftOpen  = (L2 < 0);
    bool rightOpen = (R2 < 0);
    int turnDeg = leftOpen ? +90 : (rightOpen ? -90 : 180);
    Serial.printf("Turn decision: %d deg (L2=%.1f, R2=%.1f) | posA=%d posB=%d\n",
                  turnDeg, L2, R2, posA, posB);

    // 3) IMU turn
    if (turnDeg ==  90) turnIMU(+90, 80, 1600);
    else if (turnDeg == -90) turnIMU(-90, 80, 1600);
    else                     turnIMU(180, 85, 1800);

    motorsStop();
    delay(150);
    return;  // resume forward cycle
  }

  // slow near front wall
  if (frontOK && F < FRONT_SLOW_MM) base = 70;

  // centered drive
  driveCentered(base, err);

  // debug
  Serial.printf("L=%.1f R=%.1f F=%.1f err=%.2f base=%d | posA=%d posB=%d\n",
                L, R, F, err, base, posA, posB);
  delay(10);
}

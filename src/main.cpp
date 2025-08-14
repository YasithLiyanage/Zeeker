#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include "Adafruit_VL6180X.h"

// === Pins ===
#define PWM_LEFT   4
#define PWM_RIGHT  5
#define IN1_LEFT   15
#define IN2_LEFT   21
#define IN1_RIGHT  22
#define IN2_RIGHT  23

#define SDA_PIN    25
#define SCL_PIN    26

// === ToF MUX channels ===
#define MUX_ADDR   0x70
#define CH_LEFT    0
#define CH_RIGHT   1
#define CH_FRONT   2

Adafruit_VL6180X tof;
MPU9250_asukiaaa mpu;

// === Per-sensor calibration (edge_mm = a*raw + b) ===
struct Cal { float a, b; };
enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };
#define NUM_SENSORS 5
Cal CAL[NUM_SENSORS] = {
  /* LEFT     */ {1.00f, -42.0f},
  /* ALeft45  */ {1.00f,   0.0f},  // unused
  /* FRONT    */ {1.00f, -35.0f},
  /* ARight45 */ {1.00f,   0.0f},  // unused
  /* RIGHT    */ {1.00f, -73.0f}
};

// === Distance / logic thresholds (mm) ===
#define STOP_DIST         35    // full stop here
#define SLOW_DIST        100    // start slowing here
#define DESIRED_CENTER    40    // your chosen offset from each side
#define NO_WALL_MM       100    // > this => treat as NO WALL (prevents seeing across openings)

// === Presence debounce (consecutive samples) ===
#define PRESENT_N          3    // need N valid in a row to mark present
#define LOST_N             3    // need N invalid in a row to mark lost

// === Speed & control ===
#define MAX_SPEED          100
#define MIN_SPEED          70

// Trim & polarity (tune if it veers)
#define INVERT_LEFT    false
#define INVERT_RIGHT   false
int TRIM_LEFT  = 0;     // + makes left faster
int TRIM_RIGHT = 10;    // + makes right faster (increase if it pulls left)

// === IMU state ===
float yaw = 0.0f, gyroBias = 0.0f, targetYaw = 0.0f;
unsigned long lastIMUms = 0;

// ---------- Motor helpers ----------
void motorL_raw(int pwm) {
  pwm = constrain(pwm, -255, 255);
  bool fwd = (pwm >= 0);
  int mag = abs(pwm);
  bool A = fwd ^ INVERT_LEFT;
  digitalWrite(IN1_LEFT,  A ? HIGH : LOW);
  digitalWrite(IN2_LEFT,  A ? LOW  : HIGH);
  analogWrite(PWM_LEFT, mag);
}
void motorR_raw(int pwm) {
  pwm = constrain(pwm, -255, 255);
  bool fwd = (pwm >= 0);
  int mag = abs(pwm);
  bool A = fwd ^ INVERT_RIGHT;
  digitalWrite(IN1_RIGHT, A ? HIGH : LOW);
  digitalWrite(IN2_RIGHT, A ? LOW  : HIGH);
  analogWrite(PWM_RIGHT, mag);
}
void motorL(int pwm){ motorL_raw(pwm + TRIM_LEFT); }
void motorR(int pwm){ motorR_raw(pwm + TRIM_RIGHT); }
void motorsStop(){ motorL_raw(0); motorR_raw(0); }

// ---------- MUX / ToF ----------
void mux(uint8_t ch){ Wire.beginTransmission(MUX_ADDR); Wire.write(1 << ch); Wire.endTransmission(); }
bool tofBegin(uint8_t ch){ mux(ch); delay(3); return tof.begin(); }
int tofRaw(uint8_t ch){
  mux(ch); delay(2);
  uint8_t r = tof.readRange();
  return (tof.readRangeStatus() == VL6180X_ERROR_NONE) ? (int)r : -1;
}

// Apply per-sensor calibration, cutoff far readings as NO WALL
float edgeMM_withCutoff(SensorId sid, uint8_t ch) {
  int raw = tofRaw(ch);
  if (raw < 0) return -1;

  // --- per-sensor raw inversion for RIGHT ---
  if (sid == SID_RIGHT) {
    const int RAW_MAX = 200; // VL6180X practical limit
    raw = RAW_MAX - raw;     // invert: closer => smaller mm
  }

  float mm = CAL[sid].a * raw + CAL[sid].b;
  if (mm < 0) mm = 0;
  if (mm > NO_WALL_MM) return -1;   // opening detected -> treat as NO WALL
  return mm;
}

float leftMM()  { return edgeMM_withCutoff(SID_LEFT,  CH_LEFT ); }
float rightMM() { return edgeMM_withCutoff(SID_RIGHT, CH_RIGHT); }
float frontMM() { return edgeMM_withCutoff(SID_FRONT, CH_FRONT); }

// ---------- IMU ----------
void imuBias(){
  unsigned long t0 = millis(); long n=0; double s=0;
  while(millis()-t0 < 1500){ mpu.gyroUpdate(); s += mpu.gyroZ(); n++; delay(2); }
  gyroBias = (n>0)? (float)(s/n) : 0.0f;
}
void imuUpdate(){
  mpu.gyroUpdate();
  float gz = mpu.gyroZ() - gyroBias; // deg/s
  unsigned long now = millis();
  float dt = (now - lastIMUms)/1000.0f; lastIMUms = now;
  yaw += gz * dt;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

// avoid C++20 std::lerp clash
static inline float myLerp(float a,float b,float t){ if(t<0)t=0; if(t>1)t=1; return a + (b-a)*t; }

// ---------- Presence debounce ----------
bool L_present=false, R_present=false;
int  L_cntP=0, L_cntL=0, R_cntP=0, R_cntL=0;
void updatePresence(float val, bool& present, int& cntP, int& cntL){
  if (val >= 0){
    cntP++; cntL = 0;
    if (!present && cntP >= PRESENT_N) present = true;
  } else {
    cntL++; cntP = 0;
    if (present && cntL >= LOST_N) present = false;
  }
}

// ---------- Drive ----------
void driveStraight(float baseSpeed, float headingErrDeg){
  float turn = headingErrDeg * 0.5f;       // P gain (deg -> PWM)
  if (turn > 40) turn = 40; if (turn < -40) turn = -40;
  int l = (int)(baseSpeed - turn);
  int r = (int)(baseSpeed + turn);
  l = constrain(l, 0, 255);
  r = constrain(r, 0, 255);
  motorL(l); motorR(r);
}

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(IN1_LEFT, OUTPUT); pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT,OUTPUT); pinMode(IN2_RIGHT,OUTPUT);
  motorsStop();

  // Init the 3 sensors
  tofBegin(CH_LEFT);
  tofBegin(CH_RIGHT);
  tofBegin(CH_FRONT);

  // IMU
  mpu.setWire(&Wire);
  mpu.beginGyro();
  lastIMUms = millis();
  imuBias();
  yaw = 0.0f;
  targetYaw = yaw;  // hold the *current* heading
  Serial.println("Ready. Debounced side walls, NO_WALL cutoff, single/dual-side control, smooth stop.");
}

void loop(){
  imuUpdate();

  float f = frontMM();
  float l = leftMM();
  float r = rightMM();

  // Update presence with debounce (prevents grabbing far wall at openings)
  updatePresence(l, L_present, L_cntP, L_cntL);
  updatePresence(r, R_present, R_cntP, R_cntL);

  // Smooth speed between SLOW_DIST and STOP_DIST
  float speed = MAX_SPEED;
  if (f > 0 && f < SLOW_DIST){
    float t = (f - STOP_DIST) / (SLOW_DIST - STOP_DIST); // 0..1
    speed = myLerp(MIN_SPEED, MAX_SPEED, t);
  }

  // Hard stop at STOP_DIST
  if (f > 0 && f <= STOP_DIST){
    motorsStop();
    Serial.printf("STOP  F=%.1f  L=%.1f(%d/%d)  R=%.1f(%d/%d)\n", f, l, L_present, L_cntP, r, R_present, R_cntP);
    delay(20);
    return;
  }

  // Control logic:
  if (L_present && R_present){
    // Center using both sides
    float err = (l - DESIRED_CENTER) - (r - DESIRED_CENTER); // l - r
    float headingErrDeg = -0.5f * err; // steer toward center
    driveStraight(speed, headingErrDeg);
    Serial.printf("CENTER L=%.1f R=%.1f F=%.1f  err=%.1f spd=%.0f\n", l, r, f, err, speed);
  }
  else if (L_present && !R_present){
    // Follow left wall only
    float errL = l - DESIRED_CENTER;     // <0 => too close to left -> steer right
    float headingErrDeg = -0.6f * errL;  // single-side gain
    driveStraight(speed, headingErrDeg);
    Serial.printf("LEFT   L=%.1f F=%.1f spd=%.0f\n", l, f, speed);
  }
  else if (!L_present && R_present){
    // Follow right wall only
    float errR = r - DESIRED_CENTER;     // <0 => too close to right -> steer left
    float headingErrDeg = +0.6f * errR;  // single-side gain
    driveStraight(speed, headingErrDeg);
    Serial.printf("RIGHT  R=%.1f F=%.1f spd=%.0f\n", r, f, speed);
  }
  else {
    // No side walls — hold IMU heading
    float headingErrDeg = targetYaw - yaw;
    if (headingErrDeg > 180) headingErrDeg -= 360;
    if (headingErrDeg < -180) headingErrDeg += 360;
    driveStraight(speed, headingErrDeg);
    Serial.printf("IMU    yaw=%.2f tgt=%.2f  F=%.1f  spd=%.0f\n", yaw, targetYaw, f, speed);
  }

  delay(20);
}

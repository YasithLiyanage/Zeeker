// #include <Wire.h>
// #include <MPU9250_asukiaaa.h>
// #include "Adafruit_VL6180X.h"

// // === Pins ===
// #define PWM_LEFT   4
// #define PWM_RIGHT  5
// #define IN1_LEFT   15
// #define IN2_LEFT   21
// #define IN1_RIGHT  22
// #define IN2_RIGHT  23

// #define SDA_PIN    25
// #define SCL_PIN    26

// // === ToF MUX channels ===
// #define MUX_ADDR   0x70
// #define CH_LEFT    0
// #define CH_RIGHT   4
// #define CH_FRONT   2

// Adafruit_VL6180X tof;
// MPU9250_asukiaaa mpu;

// // === Per-sensor calibration (edge_mm = a*raw + b) ===
// struct Cal { float a, b; };
// enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };
// #define NUM_SENSORS 5
// Cal CAL[NUM_SENSORS] = {
//   /* LEFT     */ {1.00f, -42.0f},
//   /* ALeft45  */ {1.00f,   0.0f},  // unused
//   /* FRONT    */ {1.00f, -35.0f},
//   /* ARight45 */ {1.00f,   0.0f},  // unused
//   /* RIGHT    */ {1.00f, -28.0f}
// };

// // === Distance / logic thresholds (mm) ===
// #define STOP_DIST         45
// #define SLOW_DIST        100
// #define DESIRED_CENTER    40
// #define NO_WALL_MM       100

// // === Presence debounce ===
// #define PRESENT_N          3
// #define LOST_N             3

// // === Speed & control ===
// #define MAX_SPEED          100
// #define MIN_SPEED          70

// // Trim & polarity
// #define INVERT_LEFT    false
// #define INVERT_RIGHT   false
// int TRIM_LEFT  = 0;
// int TRIM_RIGHT = 10;

// // === IMU state ===
// float yaw = 0.0f, gyroBias = 0.0f, targetYaw = 0.0f;
// unsigned long lastIMUms = 0;

// // ---------- Motor helpers ----------
// void motorL_raw(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   int mag = abs(pwm);
//   bool A = fwd ^ INVERT_LEFT;
//   digitalWrite(IN1_LEFT,  A ? HIGH : LOW);
//   digitalWrite(IN2_LEFT,  A ? LOW  : HIGH);
//   analogWrite(PWM_LEFT, mag);
// }
// void motorR_raw(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   int mag = abs(pwm);
//   bool A = fwd ^ INVERT_RIGHT;
//   digitalWrite(IN1_RIGHT, A ? HIGH : LOW);
//   digitalWrite(IN2_RIGHT, A ? LOW  : HIGH);
//   analogWrite(PWM_RIGHT, mag);
// }
// void motorL(int pwm){ motorL_raw(pwm + TRIM_LEFT); }
// void motorR(int pwm){ motorR_raw(pwm + TRIM_RIGHT); }
// void motorsStop(){ motorL_raw(0); motorR_raw(0); }

// // ---------- MUX / ToF ----------
// void mux(uint8_t ch){ Wire.beginTransmission(MUX_ADDR); Wire.write(1 << ch); Wire.endTransmission(); }
// bool tofBegin(uint8_t ch){ mux(ch); delay(3); return tof.begin(); }
// int tofRaw(uint8_t ch){
//   mux(ch); delay(2);
//   uint8_t r = tof.readRange();
//   return (tof.readRangeStatus() == VL6180X_ERROR_NONE) ? (int)r : -1;
// }

// // Apply per-sensor calibration, cutoff far readings as NO WALL
// float edgeMM_withCutoff(SensorId sid, uint8_t ch) {
//   int raw = tofRaw(ch);
//   if (raw < 0) return -1;
//   float mm = CAL[sid].a * raw + CAL[sid].b;
//   if (mm < 0) mm = 0;
//   if (mm > NO_WALL_MM) return -1;
//   return mm;
// }

// float leftMM()  { return edgeMM_withCutoff(SID_LEFT,  CH_LEFT ); }
// float rightMM() { return edgeMM_withCutoff(SID_RIGHT, CH_RIGHT); }
// float frontMM() { return edgeMM_withCutoff(SID_FRONT, CH_FRONT); }

// // ---------- IMU ----------
// void imuBias(){
//   unsigned long t0 = millis(); long n=0; double s=0;
//   while(millis()-t0 < 1500){ mpu.gyroUpdate(); s += mpu.gyroZ(); n++; delay(2); }
//   gyroBias = (n>0)? (float)(s/n) : 0.0f;
// }
// void imuUpdate(){
//   mpu.gyroUpdate();
//   float gz = mpu.gyroZ() - gyroBias; // deg/s
//   unsigned long now = millis();
//   float dt = (now - lastIMUms)/1000.0f; lastIMUms = now;
//   yaw += gz * dt;
//   if (yaw > 180) yaw -= 360;
//   if (yaw < -180) yaw += 360;
// }

// // ---------- helpers ----------
// static inline float myLerp(float a,float b,float t){ if(t<0)t=0; if(t>1)t=1; return a + (b-a)*t; }
// static inline float angNorm(float a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }

// // ---------- Drive ----------
// void driveStraight(float baseSpeed, float headingErrDeg){
//   float turn = headingErrDeg * 1.0f;
//   if (turn > 40) turn = 40; if (turn < -40) turn = -40;
//   int l = (int)(baseSpeed - turn);
//   int r = (int)(baseSpeed + turn);
//   l = constrain(l, 0, 255);
//   r = constrain(r, 0, 255);
//   motorL(l); motorR(r);
// }

// // ---------- Presence debounce ----------
// bool L_present=false, R_present=false;
// int  L_cntP=0, L_cntL=0, R_cntP=0, R_cntL=0;
// void updatePresence(float val, bool& present, int& cntP, int& cntL){
//   if (val >= 0){
//     cntP++; cntL = 0;
//     if (!present && cntP >= PRESENT_N) present = true;
//   } else {
//     cntL++; cntP = 0;
//     if (present && cntL >= LOST_N) present = false;
//   }
// }

// // ---------- Simple IMU-based turn ----------
// // void turn(float deg){
// //   // deg > 0 => turn RIGHT (CW), deg < 0 => turn LEFT (CCW)
// //   const int SPIN_PWM = 110;
// //   const float TOL_DEG = 3.0f;
// //   const unsigned long TIMEOUT_MS = 4000;
// //   const int TURN_SIGN = +1; // flip to -1 if your robot turns the wrong way

// //   float target = angNorm(yaw + deg);
// //   motorsStop(); delay(20);

// //   unsigned long t0 = millis();
// //   while (millis() - t0 < TIMEOUT_MS) {
// //     imuUpdate();
// //     float err = angNorm(target - yaw);
// //     if (fabsf(err) <= TOL_DEG) break;

// //     int dir = ((deg >= 0) ? +1 : -1) * TURN_SIGN;
// //     motorL(+dir * SPIN_PWM);
// //     motorR(-dir * SPIN_PWM);
// //     delay(5);
// //   }

// //   motorsStop();
// //   delay(100);
// //   // targetYaw = yaw; // enable if you want heading hold later
// // }


// // Drop this in to replace your existing turn()

// void turn(float deg){
//   // Tweak if needed
//   const int   PWM_FAST      = 100;    // coarse spin
//   const int   PWM_SLOW      = 60;     // finish spin
//   const float DECEL_ANGLE   = 50.0f;  // switch to slow when |err| <= this
//   const float TOL_DEG       = 20.0f;   // final angle tolerance
//   const float STOP_RATE_DPS = 10.0f;  // must also be rotating slowly
//   const unsigned long STABLE_MS = 200;
//   const unsigned long TIMEOUT_MS = 1500;
//   const int TURN_SIGN = +1;           // set to -1 if your wiring turns the wrong way

//   // local helper: signed shortest-path error [-180,180]
//   auto angErr = [](float target, float now){
//     float e = target - now;
//     if (e > 180) e -= 360;
//     if (e < -180) e += 360;
//     return e;
//   };

//   float target = angNorm(yaw + deg);

//   motorsStop();
//   delay(20);

//   unsigned long t0 = millis();

//   // Phase 1: fast until near target
//   while (millis() - t0 < TIMEOUT_MS) {
//     imuUpdate();
//     float err = angErr(target, yaw);
//     if (fabsf(err) <= DECEL_ANGLE) break;

//     int dir = ((err >= 0) ? +1 : -1) * TURN_SIGN; // choose by *current* error sign
//     motorL(+dir * PWM_FAST);
//     motorR(-dir * PWM_FAST);
//     delay(5);
//   }

//   // Phase 2: slow finish with rate gate
//   unsigned long stableStart = 0;
//   while (millis() - t0 < TIMEOUT_MS) {
//     imuUpdate();
//     float err  = angErr(target, yaw);
//     float aerr = fabsf(err);
//     float rate = mpu.gyroZ() - gyroBias; // deg/s

//     bool inside = (aerr <= TOL_DEG) && (fabsf(rate) <= STOP_RATE_DPS);
//     if (inside) {
//       if (!stableStart) stableStart = millis();
//       if (millis() - stableStart >= STABLE_MS) break;
//     } else {
//       stableStart = 0;
//     }

//     int dir = ((err >= 0) ? +1 : -1) * TURN_SIGN;
//     motorL(+dir * PWM_SLOW);
//     motorR(-dir * PWM_SLOW);
//     delay(5);
//   }

//   motorsStop();
//   delay(120);

//   // Snap integrated yaw to the commanded target so small residuals don't accumulate
//   yaw = target;  // or: yaw = angNorm(yaw + angErr(target, yaw));
//   // targetYaw = yaw; // enable if you use heading-hold while driving
// }







// void setup(){
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   pinMode(IN1_LEFT, OUTPUT); pinMode(IN2_LEFT, OUTPUT);
//   pinMode(IN1_RIGHT,OUTPUT); pinMode(IN2_RIGHT,OUTPUT);
//   motorsStop();

//   // Init the 3 sensors
//   tofBegin(CH_LEFT);
//   tofBegin(CH_RIGHT);
//   tofBegin(CH_FRONT);

//   // IMU
//   mpu.setWire(&Wire);
//   mpu.beginGyro();
//   lastIMUms = millis();
//   imuBias();
//   yaw = 0.0f;
//   targetYaw = yaw;
//   Serial.println("Ready.");
// }

// void loop(){
//   imuUpdate();

//   float f = frontMM();
//   float l = leftMM();
//   float r = rightMM();

//   // Update presence with debounce
//   updatePresence(l, L_present, L_cntP, L_cntL);
//   updatePresence(r, R_present, R_cntP, R_cntL);

//   // Smooth speed between SLOW_DIST and STOP_DIST
//   float speed = MAX_SPEED;
//   if (f > 0 && f < SLOW_DIST){
//     float t = (f - STOP_DIST) / (SLOW_DIST - STOP_DIST);
//     speed = myLerp(MIN_SPEED, MAX_SPEED, t);
//   }

//   // --- Front-wall handling and driving ---
//   if (f > 0 && f <= STOP_DIST) {
//     motorsStop();

//     if (!L_present && R_present) {
//       Serial.println("FRONT BLOCKED: turning LEFT (-90)");
//       turn(-90);
//     } else if (!R_present && L_present) {
//       Serial.println("FRONT BLOCKED: turning RIGHT (+90)");
//       turn(+90);
//     } else if (!L_present && !R_present) {
//       Serial.println("FRONT BLOCKED: both sides open, turning LEFT (-90)");
//       turn(-90);
//     } else {
//       Serial.println("DEAD END: turning 180");
//       turn(180);
//     }
//     return; // done this loop
//   }

//   // Otherwise, drive as before
//   if (L_present && R_present){
//     float err = (l - DESIRED_CENTER) - (r - DESIRED_CENTER);
//     float headingErrDeg = -0.5f * err;
//     driveStraight(speed, headingErrDeg);
//   }
//   else if (L_present && !R_present){
//     float errL = l - DESIRED_CENTER;
//     float headingErrDeg = -0.6f * errL;
//     driveStraight(speed, headingErrDeg);
//   }
//   else if (!L_present && R_present){
//     float errR = r - DESIRED_CENTER;
//     float headingErrDeg = +0.6f * errR;
//     driveStraight(speed, headingErrDeg);
//   }
//   else {
//     float headingErrDeg = angNorm(targetYaw - yaw);
//     driveStraight(speed, headingErrDeg);
//   }

//   delay(20);
// }

// #include <Wire.h>
// #include <MPU9250_asukiaaa.h>
// #include "Adafruit_VL6180X.h"
// #include <math.h>

// #define PWM_LEFT   4
// #define PWM_RIGHT  5
// #define IN1_LEFT   15
// #define IN2_LEFT   21
// #define IN1_RIGHT  22
// #define IN2_RIGHT  23

// #define SDA_PIN    25
// #define SCL_PIN    26

// #define MUX_ADDR   0x70
// #define CH_LEFT    0
// #define CH_RIGHT   4
// #define CH_FRONT   2

// Adafruit_VL6180X tof;
// MPU9250_asukiaaa mpu;

// #define NO_WALL_MM       100




// /////////////
// // ---------- Filtering & availability ----------
// // Exponential Moving Average (EMA) + median-of-3 + simple outlier rejection.

// struct EMA { float y=0; bool has=false; float alpha=0.35f; }; // tune alpha 0.25–0.5
// struct SensorFilt {
//   EMA ema;
//   float hist[3] = {0,0,0};  // ring buffer for median-of-3
//   int   idx = 0;
//   bool  ok = false;         // "available" (within range and not rejected)
//   float mm = -1;            // latest filtered mm (valid only if ok==true)
// };

// inline bool inRangeMM(float mm){
//   return (mm >= 0.0f && mm <= NO_WALL_MM);
// }

// inline float median3(float a, float b, float c){
//   // fast median of three
//   if (a > b) { float t=a; a=b; b=t; }
//   if (b > c) { float t=b; b=c; c=t; }
//   if (a > b) b = a;
//   return b;
// }

// // Update one sensor with new sample (mm), do validity + filtering
// void updateSensor(SensorFilt &s, float newMM, float outlierMM=25.0f){
//   // validity by range first
//   if(!inRangeMM(newMM)){ s.ok=false; return; }

//   // outlier rejection (skip if deviates too much from EMA)
//   if(s.ema.has && fabsf(newMM - s.ema.y) > outlierMM){
//     // reject this sample, keep previous state
//     s.ok = true;  // still have a valid last value
//     return;
//   }

//   // push into ring buffer, compute median
//   s.hist[s.idx] = newMM;
//   s.idx = (s.idx + 1) % 3;
//   float med = median3(s.hist[0], s.hist[1], s.hist[2]);

//   // EMA smoothing
//   if(!s.ema.has){ s.ema.y = med; s.ema.has = true; }
//   else{
//     s.ema.y = s.ema.alpha * med + (1.0f - s.ema.alpha) * s.ema.y;
//   }

//   s.mm = s.ema.y;
//   s.ok = true;
// }

// // ---- Forward declarations for ToF helpers (needed by edgeMM_raw) ----
// void mux(uint8_t ch);
// bool tofBegin(uint8_t ch);
// int  tofRaw(uint8_t ch);

// /////////////

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


// float edgeMM_raw(SensorId sid, uint8_t ch){
//   int raw = tofRaw(ch);
//   if (raw < 0) return -1;
//   float mm = CAL[sid].a * raw + CAL[sid].b;
//   if (mm < 0) mm = 0;     // clamp negative to 0 mm
//   return mm;              // no NO_WALL cutoff here
// }

// // Raw per-sensor (no cutoff)
// float leftMM_raw()  { return edgeMM_raw(SID_LEFT,  CH_LEFT ); }
// float rightMM_raw() { return edgeMM_raw(SID_RIGHT, CH_RIGHT); }
// float frontMM_raw() { return edgeMM_raw(SID_FRONT, CH_FRONT); }

// ////////////////


// SensorFilt filtL, filtR, filtF;  // left, right, front filters

// ////////////////



// // === IMU state ===
// float yaw = 0.0f, gyroBias = 0.0f, targetYaw = 0.0f;
// unsigned long lastIMUms = 0;





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







// // ---------- Motors ----------

// int shapePWM(int u, int minPWM=40){          // adjust minPWM (35–55) to suit your motors
//   u = constrain(u, -255, 255);
//   if(u > 0)  u = map(u, 1, 255, minPWM, 255);
//   if(u < 0)  u = -map(-u, 1, 255, minPWM, 255);
//   return u;
// }

// void motorL(int pwm){
//   pwm = shapePWM(pwm);                       // apply deadband
//   bool fwd = (pwm >= 0);
//   int mag = abs(pwm);
//   digitalWrite(IN1_LEFT,  fwd ? HIGH : LOW);
//   digitalWrite(IN2_LEFT,  fwd ? LOW  : HIGH);
//   analogWrite(PWM_LEFT, mag);
// }

// void motorR(int pwm){
//   pwm = shapePWM(pwm);                       // apply deadband
//   bool fwd = (pwm >= 0);
//   int mag = abs(pwm);
//   digitalWrite(IN1_RIGHT, fwd ? HIGH : LOW);
//   digitalWrite(IN2_RIGHT, fwd ? LOW  : HIGH);
//   analogWrite(PWM_RIGHT, mag);
// }
  

// void motorsStop(){
//   motorL(0);
//   motorR(0);
// }   


// void driveStraight(float baseSpeed, float headingErrDeg){

// float turn = headingErrDeg * 1.0f;
// int l = (int)(baseSpeed - turn);
// int r = (int)(baseSpeed + turn);
//   l = constrain(l, 0, 255);
//   r = constrain(r, 0, 255);
//   motorL(l); motorR(r);
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


//   /////////

//   // Give a sensible starting point so median-of-3 has something to work with (optional)
// filtL = SensorFilt{};
// filtR = SensorFilt{};
// filtF = SensorFilt{};
// // Example: make front a tad smoother
// filtF.ema.alpha = 0.30f;







// }

// void loop(){
//   imuUpdate();

// // Old:
// // float f = frontMM();
// // float l = leftMM();
// // float r = rightMM();

// // New: read raw (no cutoff), then filter + availability
// float l_raw = leftMM_raw();
// float r_raw = rightMM_raw();
// float f_raw = frontMM_raw();

// // Update filters (25mm outlier gate for side walls; 35mm for front is often nicer)
// updateSensor(filtL, l_raw, 25.0f);
// updateSensor(filtR, r_raw, 25.0f);
// updateSensor(filtF, f_raw, 35.0f);

// // Use only if available
// bool leftOK  = filtL.ok;
// bool rightOK = filtR.ok;
// bool frontOK = filtF.ok;

// float l = leftOK  ? filtL.mm : -1.0f;
// float r = rightOK ? filtR.mm : -1.0f;
// float f = frontOK ? filtF.mm : -1.0f;

// float speed = 100;

// // Only use heading error if BOTH side walls are valid
// float headingErrDeg = 0.0f;
// if (leftOK && rightOK) {
//   headingErrDeg = l - r;
// } else if (leftOK && !rightOK) {
//   headingErrDeg = (l - 35);     // follow-left setpoint (mm)
// } else if (!leftOK && rightOK) {
//   headingErrDeg = -(r - 35);    // follow-right setpoint (mm)
// }
// // else: no walls → keep headingErrDeg = 0 (go straight)

// // Front slow-down (very helpful)
// if (frontOK){
//   if (f < 110) speed = min(speed, 90.0f);
//   if (f <  90) speed = min(speed, 60.0f);
//   if (f <  70) speed = min(speed, 30.0f);
//   if (f <  55) speed = 0.0f;
// }

// // Clamp and steer
// headingErrDeg = constrain(headingErrDeg, -40.0f, 40.0f);
// float turn = -headingErrDeg * 0.2f;      // gain
// turn = constrain(turn, -80.0f, 80.0f);

// // (Optional) Slew-limit turn to reduce oscillation
// static float turnPrev = 0;
// float maxStep = 12.0f;                    // max PWM change per loop
// float turnCmd = turnPrev + constrain(turn - turnPrev, -maxStep, maxStep);
// turnPrev = turnCmd;

// driveStraight(speed, turnCmd);            // <-- single call

// Serial.printf("L=%.2f(%d)  R=%.2f(%d)  F=%.2f(%d)  err=%.2f  v=%.0f  t=%.1f\n",
//               l, leftOK, r, rightOK, f, frontOK, headingErrDeg, speed, turnCmd);


// }
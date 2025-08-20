// #include <Wire.h>
// #include "Adafruit_VL6180X.h"

// // --- Motor pins ---
// #define PWM_LEFT    4
// #define PWM_RIGHT   5
// #define IN1_LEFT    15
// #define IN2_LEFT    21
// #define IN1_RIGHT   22
// #define IN2_RIGHT   23

// // --- I2C MUX ---
// #define SDA_PIN     25
// #define SCL_PIN     26
// #define MUX_ADDR    0x70

// // MUX channels
// #define CH_LEFT      0
// #define CH_ALEFT45   1
// #define CH_FRONT     2
// #define CH_ARIGHT45  3
// #define CH_RIGHT     4

// Adafruit_VL6180X tof;

// // ---- Calibration ----
// struct Cal { float a, b; };
// enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };

// Cal CAL[5] = {
//   {1.00f, -42.0f},  // LEFT
//   {1.00f,  20.0f},  // ALEFT45
//   {1.00f, -35.0f},  // FRONT
//   {1.00f,   0.0f},  // ARIGHT45
//   {1.00f, -28.0f}   // RIGHT
// };

// // ---- Params ----
// #define NO_WALL_MM        100
// #define TARGET_SIDE_MM    50
// #define FRONT_SLOW_MM     80
// #define FRONT_STOP_MM     35

// // Square-up params
// #define A45_TOL_MM        6
// #define SQUARE_TURN_PWM   50
// #define SQUARE_MAX_MS     1500
// #define GAP_TOL_MM        4
// #define NUDGE_PWM         60

// // --- MUX helpers ---
// void mux(uint8_t ch) {
//   Wire.beginTransmission(MUX_ADDR);
//   Wire.write(1 << ch);
//   Wire.endTransmission();
//   delayMicroseconds(200);
// }

// bool tofBegin(uint8_t ch) {
//   mux(ch);
//   delay(3);
//   return tof.begin();
// }

// float readMM(SensorId sid, uint8_t ch) {
//   mux(ch);
//   delay(2);
//   uint8_t r = tof.readRange();
//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
//   float mm = CAL[sid].a * r + CAL[sid].b;
//   if (mm < 0) mm = 0;
//   if (mm > NO_WALL_MM) return -1.0f;
//   return mm;
// }

// // --- Motors ---
// void motorL(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   digitalWrite(IN1_LEFT,  fwd ? HIGH : LOW);
//   digitalWrite(IN2_LEFT,  fwd ? LOW  : HIGH);
//   analogWrite(PWM_LEFT, abs(pwm));
// }
// void motorR(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   digitalWrite(IN1_RIGHT, fwd ? HIGH : LOW);
//   digitalWrite(IN2_RIGHT, fwd ? LOW  : HIGH);
//   analogWrite(PWM_RIGHT, abs(pwm));
// }
// void motorsStop() { motorL(0); motorR(0); }

// // --- Wall-follow drive (PID) ---
// void driveCentered(int basePWM, float err) {
//   static float err_prev = 0;
//   static float err_sum  = 0;
//   unsigned long now = millis();
//   static unsigned long last = now;
//   float dt = (now - last) / 1000.0f;
//   last = now;

//   const float Kp = 0.8f;
//   const float Ki = 0.0f;
//   const float Kd = 0.1f;

//   err_sum += err * dt;
//   float derr = (err - err_prev) / dt;

//   float turn = Kp * err + Ki * err_sum + Kd * derr;
//   err_prev = err;

//   int l = basePWM + turn;
//   int r = basePWM - turn;

//   motorL(constrain(l, 0, 255));
//   motorR(constrain(r, 0, 255));
// }

// // --- Square-up routine ---
// void squareUp() {
//   Serial.println(">>> Square-up START");
//   unsigned long t0 = millis();

//   while (millis() - t0 < SQUARE_MAX_MS) {
//     float al = readMM(SID_ALEFT45, CH_ALEFT45);
//     float ar = readMM(SID_ARIGHT45, CH_ARIGHT45);
//     if (al < 0 || ar < 0) break;

//     float diff = ar - al;
//     Serial.printf("Square diff=%.1f al=%.1f ar=%.1f\n", diff, al, ar);

//     if (fabs(diff) <= A45_TOL_MM) break;

//     if (diff > 0) { // left farther → rotate left
//       motorL(-SQUARE_TURN_PWM);
//       motorR(SQUARE_TURN_PWM);
//     } else {        // right farther → rotate right
//       motorL(SQUARE_TURN_PWM);
//       motorR(-SQUARE_TURN_PWM);
//     }
//     delay(20);
//   }
//   motorsStop();

//   // adjust distance to front wall
//   float f = readMM(SID_FRONT, CH_FRONT);
//   if (f > 0 && fabs(f - FRONT_STOP_MM) > GAP_TOL_MM) {
//     int dir = (f > FRONT_STOP_MM) ? 1 : -1;
//     motorL(dir * NUDGE_PWM);
//     motorR(dir * NUDGE_PWM);
//     delay(200);
//     motorsStop();
//   }
//   Serial.println(">>> Square-up DONE");
// }

// // --- Setup ---
// void setup() {
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   pinMode(IN1_LEFT, OUTPUT);
//   pinMode(IN2_LEFT, OUTPUT);
//   pinMode(IN1_RIGHT,OUTPUT);
//   pinMode(IN2_RIGHT,OUTPUT);

//   motorsStop();

//   tofBegin(CH_LEFT);
//   tofBegin(CH_ALEFT45);
//   tofBegin(CH_FRONT);
//   tofBegin(CH_ARIGHT45);
//   tofBegin(CH_RIGHT);

//   Serial.println("Zeeker drive + square-up ready (continuous loop).");
// }

// // --- Main loop ---
// void loop() {
//   float L = readMM(SID_LEFT,  CH_LEFT);
//   float F = readMM(SID_FRONT, CH_FRONT);
//   float R = readMM(SID_RIGHT, CH_RIGHT);

//   int base = 150;

//   float err = 0.0f;
//   bool leftOK  = (L >= 0.0f);
//   bool rightOK = (R >= 0.0f);
//   bool frontOK = (F >= 0.0f);

//   if (leftOK && rightOK) err = (L - R);
//   else if (leftOK && !rightOK) err = (L - TARGET_SIDE_MM);
//   else if (!leftOK && rightOK) err = -(R - TARGET_SIDE_MM);

//   if (frontOK && F < FRONT_STOP_MM) {
//     motorsStop();
//     squareUp();       // perform square-up
//     delay(500);       // pause before resuming
//   } else {
//     if (frontOK && F < FRONT_SLOW_MM) base = 70;
//     driveCentered(base, err);
//   }

//   Serial.printf("L=%.1f R=%.1f F=%.1f err=%.2f base=%d\n", L, R, F, err, base);
//   delay(10);
// }

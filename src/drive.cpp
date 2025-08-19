// #include <Wire.h>
// #include "Adafruit_VL6180X.h"

// #define PWM_LEFT    4
// #define PWM_RIGHT   5
// #define IN1_LEFT    15
// #define IN2_LEFT    21
// #define IN1_RIGHT   22
// #define IN2_RIGHT   23

// #define SDA_PIN     25
// #define SCL_PIN     26

// // ---- I2C MUX channels ----
// #define MUX_ADDR    0x70

// #define CH_LEFT     0
// #define CH_ALEFT45  1    
// #define CH_FRONT    2
// #define CH_ARIGHT45 3   
// #define CH_RIGHT    4

// Adafruit_VL6180X tof;

// // ---- Basic ranges/targets ----
// #define NO_WALL_MM        100     // readings > this = treat as "no wall"
// #define TARGET_SIDE_MM    50      // single-side follow setpoint (tune!)
// #define FRONT_SLOW_MM     80      // start slowing down
// #define FRONT_STOP_MM     55      // stop distance to front wall

// // square-up params
// #define A45_TOL_MM        6     // equality tolerance for A45L vs A45R
// #define SQUARE_TURN_PWM   70    // gentle in-place turn PWM
// #define SQUARE_MAX_MS     1200  // bail-out guard
// #define GAP_TOL_MM        3     // final front-gap tolerance
// #define NUDGE_PWM         70    // small forward/back nudge


// // ---- Simple calibration per sensor (mm = a*raw + b) ----
// struct Cal { float a, b; };
// enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4};

// Cal CAL[5] = {
//   /* LEFT    */ {1.00f, -42.0f},
//   /* ALEFT45 */ {1.00f,   20.0f},  
//   /* FRONT   */ {1.00f, -35.0f},
//   /* ARIGHT45*/ {1.00f,   0.0f}, 
//   /* RIGHT   */ {1.00f, -28.0f}
// };

// // ---- MUX helpers ----
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

// // return calibrated mm; -1 if invalid or "no wall"
// float readMM(SensorId sid, uint8_t ch) {
//   mux(ch);
//   delay(2);
//   uint8_t r = tof.readRange();               // raw mm (0..~200)
//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;

//   float mm = CAL[sid].a * (float)r + CAL[sid].b;
//   if (mm < 0) mm = 0;
//   if (mm > NO_WALL_MM) return -1.0f;         // treat as "no wall"
//   return mm;
// }

// // ---- Motors (very simple: no deadband shaping) ----
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

// // ---- Drive mix: base speed + simple proportional turn ----
// void driveCentered(int basePWM, float err) {
//   // Proportional gain from mm error to PWM turn; start small and tune
//  static float err_prev = 0;
// static float err_sum  = 0;
// unsigned long now = millis();
// static unsigned long last = now;
// float dt = (now - last) / 1000.0f;
// last = now;

// const float Kp = 0.8f;
// const float Ki = 0.0f;   // start with 0
// const float Kd = 0.1f;   // start with 0

// err_sum += err * dt;
// float derr = (err - err_prev) / dt;

// float turn = Kp * err + Ki * err_sum + Kd * derr;
// err_prev = err;
//              // left-right PWM delta

//   int l = basePWM + turn;
//   int r = basePWM - turn;

//   // clamp
//   l = constrain(l, 0, 255);
//   r = constrain(r, 0, 255);

//   motorL(l);
//   motorR(r);
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   pinMode(IN1_LEFT, OUTPUT);
//   pinMode(IN2_LEFT, OUTPUT);
//   pinMode(IN1_RIGHT,OUTPUT);
//   pinMode(IN2_RIGHT,OUTPUT);
//   motorsStop();

//   // init ToF sensors on each channel we use
//   bool okL = tofBegin(CH_LEFT);
//   bool okF = tofBegin(CH_FRONT);
//   bool okR = tofBegin(CH_RIGHT);

//   Serial.println("Zeeker minimal drive ready.");
//   Serial.printf("ToF init: L=%d F=%d R=%d\n", okL, okF, okR);
// }

// void loop() {
//   // read side & front distances (mm); -1 means "no wall"
//   float L = readMM(SID_LEFT,  CH_LEFT);
//   float F = readMM(SID_FRONT, CH_FRONT);
//   float R = readMM(SID_RIGHT, CH_RIGHT);

//   // base speed (PWM)
//   int base = 150;                             // 0..255, tune as needed

//   // compute lateral error:
//   //  - both sides valid: center = L - R (want 0)
//   //  - left only: follow-left = (L - TARGET_SIDE_MM)
//   //  - right only: follow-right = -(R - TARGET_SIDE_MM)
//   float err = 0.0f;
//   bool leftOK  = (L >= 0.0f);
//   bool rightOK = (R >= 0.0f);
//   bool frontOK = (F >= 0.0f);

//   if (leftOK && rightOK) {
//     err = (L - R);
//   } else if (leftOK && !rightOK) {
//     err = (L - TARGET_SIDE_MM);
//   } else if (!leftOK && rightOK) {
//     err = -(R - TARGET_SIDE_MM);
//   } else {
//     err = 0.0f; // no side walls -> just go straight
//   }

//   // simple front-wall handling: slow, then stop
//   if (frontOK) {
//     if (F < FRONT_STOP_MM) {
//       motorsStop();
//       Serial.printf("STOP  L=%.1f R=%.1f F=%.1f\n", L, R, F);
//       delay(50);
//       return; // hold stop (you can add state logic later)
//     } else if (F < FRONT_SLOW_MM) {
//       base = 70;                              // slow down near wall
//     }
//   }

//   // drive with current error
//   driveCentered(base, err);

//   // telemetry
//   Serial.printf("L=%.1f(%d)  R=%.1f(%d)  F=%.1f(%d)  err=%.2f  base=%d\n",
//                 L, leftOK, R, rightOK, F, frontOK, err, base);

//   delay(10); // ~100 Hz loop
// }

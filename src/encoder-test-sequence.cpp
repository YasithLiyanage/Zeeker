// #include <Wire.h>
// #include "Adafruit_VL6180X.h"
// #include <math.h>

// // ========== Motor pins ==========
// #define PWM_LEFT    4
// #define PWM_RIGHT   5
// #define IN1_LEFT    15
// #define IN2_LEFT    21
// #define IN1_RIGHT   22
// #define IN2_RIGHT   23

// // ========== Encoders (quadrature, A & B) ==========
// #define ENCA1 16  // Left encoder A
// #define ENCA2 17  // Left encoder B
// #define ENCB1 18  // Right encoder A
// #define ENCB2 19  // Right encoder B

// volatile int posA = 0;           // left ticks (signed)
// volatile int posB = 0;           // right ticks (signed)
// volatile int lastEncodedA = 0;   // last AB snapshot (left)
// volatile int lastEncodedB = 0;   // last AB snapshot (right)

// void IRAM_ATTR updateEncoderA() {
//   int msb = digitalRead(ENCA1);
//   int lsb = digitalRead(ENCA2);
//   int encoded = (msb << 1) | lsb;
//   int sum = (lastEncodedA << 2) | encoded;

//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posA++;
//   else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posA--;

//   lastEncodedA = encoded;
// }
// void IRAM_ATTR updateEncoderB() {
//   int msb = digitalRead(ENCB1);
//   int lsb = digitalRead(ENCB2);
//   int encoded = (msb << 1) | lsb;
//   int sum = (lastEncodedB << 2) | encoded;

//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posB--;
//   else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posB++;
//   lastEncodedB = encoded;
// }

// // ----- Buzzer (non-blocking, dedicated LEDC channel) -----
// #define BUZZER_PIN 2
// #define BUZZ_CH    7
// bool BUZZ_ENABLED = true;
// static unsigned long buzz_end_ms = 0;

// void buzzerBegin() {
//   pinMode(BUZZER_PIN, OUTPUT);
//   ledcSetup(BUZZ_CH, 2000 /*Hz*/, 8 /*bits*/);
//   ledcAttachPin(BUZZER_PIN, BUZZ_CH);
//   ledcWriteTone(BUZZ_CH, 0);
// }
// void buzzStart(int freqHz, int durMs) {
//   if (!BUZZ_ENABLED) return;
//   ledcWriteTone(BUZZ_CH, freqHz);
//   buzz_end_ms = millis() + durMs;
// }
// void buzzUpdate() {
//   if (buzz_end_ms != 0 && (long)(millis() - buzz_end_ms) >= 0) {
//     ledcWriteTone(BUZZ_CH, 0);
//     buzz_end_ms = 0;
//   }
// }
// void buzzOK()        { buzzStart(1500, 80); }      // quick chirp
// void buzzAttention() { buzzStart(1000, 120); }     // longer
// void buzzDouble()    { buzzStart(1800, 80); }      // call twice w/ spacing
// void buzzDoubleCue(){ buzzStart(1800, 80); delay(120); buzzStart(2200, 80); }

// // ========== I2C / ToF (front only is required here) ==========
// #define SDA_PIN     25
// #define SCL_PIN     26

// Adafruit_VL6180X tof;

// // ---- Simple calibration for VL6180X: mm = a*raw + b
// struct Cal { float a, b; };
// Cal CAL_FRONT = {1.00f, -35.0f};   // tune if needed

// // Read calibrated front distance (mm). Returns -1 if invalid.
// float frontMM(){
//   uint8_t r = tof.readRange();
//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
//   float mm = CAL_FRONT.a * r + CAL_FRONT.b;
//   if (mm < 0) mm = 0;
//   return mm;
// }

// // ======================================================
// /**                     Motors                          */
// // ======================================================
// void motorL(int pwm){
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   digitalWrite(IN1_LEFT,  fwd ? HIGH : LOW);
//   digitalWrite(IN2_LEFT,  fwd ? LOW  : HIGH);
//   analogWrite(PWM_LEFT, abs(pwm));
// }
// void motorR(int pwm){
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   digitalWrite(IN1_RIGHT, fwd ? HIGH : LOW);
//   digitalWrite(IN2_RIGHT, fwd ? LOW  : HIGH);
//   analogWrite(PWM_RIGHT, abs(pwm));
// }
// void motorsStop(){ motorL(0); motorR(0); }
// void motorsFwd(int pwm){ motorL(pwm); motorR(pwm); }

// // ======================================================
// // =========== Encoder Calibration (ToF-gated) ==========
// // ======================================================
// //
// // Physical setup per trial:
// // - Bot front axle sits ~50 mm behind START line, with a temporary wall so ToF≈50 mm.
// // - Press 's' to arm.
// // - Remove the start wall → bot auto-starts.
// // - Place an end wall at the 300 mm mark → ToF≈50 mm → bot auto-stops.
// //
// // Repeats 3 trials and prints averages + suggested TICKS_PER_MM.
// //
// enum CalState {
//   CAL_IDLE,
//   CAL_WAIT_START_NEAR,   // wait for ToF ~50 mm (start wall present)
//   CAL_WAIT_LEAVE_START,  // wait until ToF indicates "no wall" (wall removed)
//   CAL_RUN,               // driving forward
//   CAL_DONE_TRIAL         // completed a trial
// };

// CalState calState = CAL_IDLE;

// const float START_MM      = 50.0f;  // target gating distance
// const float START_TOL     = 8.0f;   // ± tolerance window
// const float STOP_MM       = 50.0f;  // same as start
// const float STOP_TOL      = 8.0f;

// const float NOWALL_EDGE   = 120.0f; // treat >120 mm as start "cleared"
// const int   CONSIST_SAMPLES = 3;    // consecutive reads to validate near distances

// const int   PWM_TEST      = 70;     // slow & steady
// const unsigned long SAFETY_MS = 10000;

// int trial = 0;            // 0..2
// long sumA = 0, sumB = 0;  // accumulated ticks
// long startA = 0, startB = 0;
// unsigned long tRunStart = 0;

// int nearCount = 0;        // consecutive near-wall samples
// int stopNearCount = 0;    // consecutive near-stop samples

// bool inWindow(float mm, float set, float tol){
//   return (mm >= 0) && (fabsf(mm - set) <= tol);
// }

// void printLive(bool running){
//   static unsigned long last = 0;
//   if (millis() - last < 120) return;
//   last = millis();

//   float f = frontMM();
//   if (!running) {
//     Serial.printf("[READY %d/3] F=%.1f mm  posA=%d posB=%d        \r", trial+1, f, posA, posB);
//   } else {
//     Serial.printf("[RUN %d/3]   F=%.1f mm  dA=%d dB=%d           \r", trial+1, f, posA - startA, posB - startB);
//   }
// }

// void encoderCalToF300mmLoop(){
//   printLive(calState == CAL_RUN);
//   buzzUpdate();

//   // Serial control to arm/reset/abort
//   if (Serial.available()){
//     char c = (char)Serial.read();
//     if (c == 'q'){
//       motorsStop();
//       Serial.println("\nAbort. Power-cycle or press 's' to arm again.");
//       calState = CAL_IDLE;
//       trial = 0; sumA = sumB = 0;
//       return;
//     }
//     if (c == 'r'){
//       motorsStop();
//       Serial.println("\nTrial reset. Place start wall (~50 mm) and press 's'.");
//       calState = CAL_IDLE;
//       nearCount = stopNearCount = 0;
//       return;
//     }
//     if (c == 's' && calState == CAL_IDLE){
//       Serial.printf("\nArmed for Trial %d/3. Place start wall at ~50 mm.\n", trial+1);
//       buzzDoubleCue(); // double beep
//       calState = CAL_WAIT_START_NEAR;
//       nearCount = 0;
//       stopNearCount = 0;
//       return;
//     }
//   }

//   float fmm = frontMM();

//   switch (calState){
//     case CAL_IDLE:
//       // idle; wait for 's'
//       break;

//     case CAL_WAIT_START_NEAR:
//       // wait until front ToF is ~50 mm for a few consecutive reads
//       if (inWindow(fmm, START_MM, START_TOL)){
//         if (++nearCount >= CONSIST_SAMPLES){
//           Serial.println("Start wall detected at ~50 mm. Remove it now to begin.");
//           buzzOK();
//           calState = CAL_WAIT_LEAVE_START;
//           nearCount = 0;
//         }
//       } else {
//         nearCount = 0;
//       }
//       break;

//     case CAL_WAIT_LEAVE_START:
//       // look for "no wall" (wall has been removed) to begin motion
//       if (fmm < 0 || fmm > NOWALL_EDGE){
//         // start driving
//         startA = posA;
//         startB = posB;
//         tRunStart = millis();
//         motorsFwd(PWM_TEST);
//         Serial.println("Wall removed → RUNNING...");
//         calState = CAL_RUN;
//       }
//       break;

//     case CAL_RUN:
//       // running forward until we see stop wall ~50 mm again
//       if (inWindow(fmm, STOP_MM, STOP_TOL)){
//         if (++stopNearCount >= CONSIST_SAMPLES){
//           motorsStop();
//           int dA = posA - startA;
//           int dB = posB - startB;
//           sumA += dA;
//           sumB += dB;
//           Serial.printf("\nTrial %d/3 complete. dA=%d  dB=%d\n", trial+1, dA, dB);
//           buzzOK();
//           calState = CAL_DONE_TRIAL;
//         }
//       } else {
//         stopNearCount = 0;
//       }
//       // safety guard
//       if (millis() - tRunStart > SAFETY_MS){
//         motorsStop();
//         Serial.println("\n[SAFETY STOP] Took too long. Press 'r' to retry, or 's' to re-arm.");
//         calState = CAL_IDLE;
//       }
//       break;

//     case CAL_DONE_TRIAL:
//       trial++;
//       if (trial >= 3){
//         // report
//         float avgA = sumA / 3.0f;
//         float avgB = sumB / 3.0f;
//         const float dist_mm = 300.0f;
//         float ticks_per_mm_L = avgA / dist_mm;
//         float ticks_per_mm_R = avgB / dist_mm;

//         Serial.println("\n=== ENCODER CAL RESULTS (300 mm, ToF-gated) ===");
//         Serial.printf("Left avg ticks:  %.2f   -> TICKS_PER_MM_L = %.4f\n", avgA, ticks_per_mm_L);
//         Serial.printf("Right avg ticks: %.2f   -> TICKS_PER_MM_R = %.4f\n", avgB, ticks_per_mm_R);
//         const int CELL_MM = 180;
//         Serial.printf("One cell (180 mm):  L ≈ %d ticks,  R ≈ %d ticks\n",
//           (int)roundf(ticks_per_mm_L * CELL_MM),
//           (int)roundf(ticks_per_mm_R * CELL_MM));
//         Serial.println("-----------------------------------------------");
//         Serial.println("Copy these into your main code:");
//         Serial.printf("float TICKS_PER_MM_L = %.4f;\n", ticks_per_mm_L);
//         Serial.printf("float TICKS_PER_MM_R = %.4f;\n", ticks_per_mm_R);
//         Serial.println("Done. Power-cycle or press 's' to run again (will reset).");
//         buzzAttention();
//         // hold in IDLE but keep results printed; reset counters so a new 's' starts fresh
//         trial = 0; sumA = sumB = 0; calState = CAL_IDLE;
//       } else {
//         Serial.printf("Reposition for Trial %d/3. Place start wall at ~50 mm and press 's'.\n", trial+1);
//         buzzDoubleCue();
//         calState = CAL_IDLE;
//       }
//       break;
//   }
// }

// // ======================================================
// // ====================  Setup  =========================
// // ======================================================
// void setup(){
//   Serial.begin(115200);

//   // motors & buzzer
//   pinMode(IN1_LEFT, OUTPUT);
//   pinMode(IN2_LEFT, OUTPUT);
//   pinMode(IN1_RIGHT,OUTPUT);
//   pinMode(IN2_RIGHT,OUTPUT);
//   motorsStop();
//   buzzerBegin();
//   buzzOK();

//   // Encoders
//   pinMode(ENCA1, INPUT);
//   pinMode(ENCA2, INPUT);
//   pinMode(ENCB1, INPUT);
//   pinMode(ENCB2, INPUT);
//   attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

//   // ToF (front only)
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);
//   if (!tof.begin()){
//     Serial.println("ERROR: Front VL6180X not found. Check wiring!");
//   } else {
//     Serial.println("Front VL6180X ready.");
//   }

//   Serial.println("\nEncoder calibration (ToF gated @50mm) ready.");
//   Serial.println("Place bot ~50 mm behind START with a temp wall at ~50 mm.");
//   Serial.println("Press 's' to arm, then remove start wall to begin.");
//   Serial.println("Put end wall at 300 mm mark; bot will stop when ToF~50 mm.");
//   Serial.println("Press 'q' to abort, 'r' to reset current trial.");
// }

// // ======================================================
// void loop(){
//   buzzUpdate();
//   encoderCalToF300mmLoop();  // ONLY running the ToF-gated calibration flow
//   delay(5);
// }

// #include <Wire.h>
// #include "Adafruit_VL6180X.h"

// // --- I2C / MUX ---
// #define SDA_PIN      25
// #define SCL_PIN      26
// #define MUX_ADDR     0x70

// // --- MUX channels (set to your wiring) ---
// #define CH_LEFT      0
// #define CH_ALEFT45   1
// #define CH_FRONT     2
// #define CH_ARIGHT45  3
// #define CH_RIGHT     4

// Adafruit_VL6180X tof;

// // ---- Calibration per sensor: mm = a*raw + b ----
// struct Cal { float a, b; };
// enum SensorId { SID_LEFT=0, SID_FRONT=1, SID_RIGHT=2, SID_ALEFT45=3, SID_ARIGHT45=4 };

// Cal CAL[5] = {
//   /* LEFT      */ {1.00f, -42.0f},
//   /* FRONT     */ {1.00f, -35.0f},
//   /* RIGHT     */ {1.00f, -28.0f},
//   /* ALEFT45   */ {1.00f,   20.0f},   // TODO: replace with your 45° left fit
//   /* ARIGHT45  */ {1.00f,   0.0f}    // TODO: replace with your 45° right fit
// };

// // Optional: treat very large values as “no wall”
// #define NO_WALL_MM   100

// // --- Helpers ---
// void mux(uint8_t ch) {
//   Wire.beginTransmission(MUX_ADDR);
//   Wire.write(1 << ch);
//   Wire.endTransmission();
// }

// bool tofBegin(uint8_t ch) {
//   mux(ch);
//   delay(3);
//   return tof.begin();
// }

// // raw (mm) from VL6180X; returns -1 if invalid
// int readRaw(uint8_t ch) {
//   mux(ch);
//   delay(2);
//   uint8_t r = tof.readRange();
//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1;
//   return (int)r;
// }

// // apply per-sensor calibration; returns -1 if raw invalid or beyond NO_WALL_MM
// float toMM(int raw, SensorId sid, bool cutoff=true) {
//   if (raw < 0) return -1.0f;
//   float mm = CAL[sid].a * (float)raw + CAL[sid].b;
//   if (mm < 0) mm = 0;
//   if (cutoff && mm > NO_WALL_MM) return -1.0f;  // comment out if you always want a number
//   return mm;
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   Serial.println("Init VL6180X on all 5 MUX channels...");
//   if (!tofBegin(CH_LEFT))     Serial.println("Left init FAILED");
//   if (!tofBegin(CH_ALEFT45))  Serial.println("A-Left45 init FAILED");
//   if (!tofBegin(CH_FRONT))    Serial.println("Front init FAILED");
//   if (!tofBegin(CH_ARIGHT45)) Serial.println("A-Right45 init FAILED");
//   if (!tofBegin(CH_RIGHT))    Serial.println("Right init FAILED");
//   Serial.println("Ready. Printing CALIBRATED distances (mm), -1 = no wall/invalid");
// }

// void loop() {
//   // RAW reads
//   int rawL   = readRaw(CH_LEFT);
//   int rawAL  = readRaw(CH_ALEFT45);
//   int rawF   = readRaw(CH_FRONT);
//   int rawAR  = readRaw(CH_ARIGHT45);
//   int rawR   = readRaw(CH_RIGHT);

//   // Calibrated mm (with NO_WALL cutoff)
//   float Lmm  = toMM(rawL,  SID_LEFT);
//   float ALmm = toMM(rawAL, SID_ALEFT45);
//   float Fmm  = toMM(rawF,  SID_FRONT);
//   float ARmm = toMM(rawAR, SID_ARIGHT45);
//   float Rmm  = toMM(rawR,  SID_RIGHT);

//   // Print calibrated (and raw for reference)
//   Serial.printf("L:%6.1f  || AL:%6.1f |||  F:%6.1f  ||| AR:%6.1f ||  R:%6.1f   \n",
//                 Lmm, ALmm, Fmm, ARmm, Rmm);

//   delay(200);
// }

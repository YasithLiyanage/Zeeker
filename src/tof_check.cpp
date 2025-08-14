// #include <Wire.h>
// #include "Adafruit_VL6180X.h"

// // --- MUX & Sensor setup ---
// #define MUX_ADDR   0x70
// #define CH_LEFT    0
// #define CH_FRONT   2
// #define CH_RIGHT   4

// #define SDA_PIN    25
// #define SCL_PIN    26

// Adafruit_VL6180X tof;

// // Select MUX channel
// void mux(uint8_t ch) {
//   Wire.beginTransmission(MUX_ADDR);
//   Wire.write(1 << ch);
//   Wire.endTransmission();
// }

// // Init sensor on a channel
// bool tofBegin(uint8_t ch) {
//   mux(ch);
//   delay(3);
//   return tof.begin();
// }

// // Read raw distance (mm), -1 if invalid
// int tofRead(uint8_t ch) {
//   mux(ch);
//   delay(2);
//   uint8_t range = tof.readRange();
//   uint8_t status = tof.readRangeStatus();

//   if (status != VL6180X_ERROR_NONE) return -1;
//   return (int)range;
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   Serial.println("Initializing ToF sensors...");
//   if (!tofBegin(CH_LEFT))  Serial.println("Left sensor init failed!");
//   if (!tofBegin(CH_RIGHT)) Serial.println("Right sensor init failed!");
//   if (!tofBegin(CH_FRONT)) Serial.println("Front sensor init failed!");
//   Serial.println("Ready! Printing raw distances (mm, -1 = no reading)");
// }

// void loop() {
//   int left  = tofRead(CH_LEFT);
//   int right = tofRead(CH_RIGHT);
//   int front = tofRead(CH_FRONT);

//   Serial.printf("Left: %d mm   Front: %d mm   Right: %d mm\n", left, front, right);
//   delay(200);
// }

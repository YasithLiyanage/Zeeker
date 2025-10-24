#include <Arduino.h>   // ✅ Always include this for PlatformIO or Arduino core

// === Pin Configuration ===
#define BUZZER_PIN 27   // Change to 13, 14, or 23 if needed
#define BUZZ_CH    7    // Any free LEDC channel (0–15 on ESP32)

// === Setup ===
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Buzzer Test ===");

  // Set up PWM channel for buzzer
  ledcSetup(BUZZ_CH, 2000, 8);    // channel, frequency (Hz), resolution (8-bit)
  ledcAttachPin(BUZZER_PIN, BUZZ_CH);

  Serial.println("Buzzer initialized. Starting tests...");
}

// === Helper Functions ===
void beep(int freq, int duration) {
  Serial.printf("Beep: %d Hz for %d ms\n", freq, duration);
  ledcWriteTone(BUZZ_CH, freq);
  delay(duration);
  ledcWriteTone(BUZZ_CH, 0);  // stop sound
  delay(100);
}

// === Loop ===
void loop() {
  // Simple short beeps
  beep(1000, 150);
  beep(1500, 150);
  delay(500);

  // Sweep test (for passive buzzer)
  Serial.println("Frequency sweep...");
  for (int freq = 400; freq <= 2000; freq += 200) {
    ledcWriteTone(BUZZ_CH, freq);
    delay(150);
  }
  ledcWriteTone(BUZZ_CH, 0);
  delay(1000);
}

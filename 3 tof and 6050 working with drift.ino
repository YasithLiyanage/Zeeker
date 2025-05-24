#include <Wire.h>
#include <MPU6050.h>
#include "Adafruit_VL6180X.h"

#define SDA_PIN 33  
#define SCL_PIN 32  

// Shutdown pins for 3 sensors
#define SHUT_PIN_0 25  // Sensor 0 (Front)
#define SHUT_PIN_1 26  // Sensor 1 (Left)
#define SHUT_PIN_2 27  // Sensor 2 (Right)

MPU6050 mpu;
Adafruit_VL6180X sensor;

float angleZ = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  buzz(1);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }

  // Set up shutdown pins for VL6180X TOF sensors
  pinMode(SHUT_PIN_0, OUTPUT);
  pinMode(SHUT_PIN_1, OUTPUT);
  pinMode(SHUT_PIN_2, OUTPUT);

  // Ensure all sensors are OFF before starting
  digitalWrite(SHUT_PIN_0, LOW);
  digitalWrite(SHUT_PIN_1, LOW);
  digitalWrite(SHUT_PIN_2, LOW);
  delay(50);

  lastTime = millis();
}

bool initSensor(int shutPin) {
  digitalWrite(shutPin, LOW);
  delayMicroseconds(500);  // Reduce delay for speed
  digitalWrite(shutPin, HIGH);
  delayMicroseconds(1000);
  //delay(500);  
  return sensor.begin();
}

void loop() {
  int16_t gx_dummy, gy_dummy, gz;
  mpu.getRotation(&gx_dummy, &gy_dummy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Calculate yaw angle (Z-axis)
  float gyroZ = gz / 131.0;  // Scale factor for gyro in degrees/s
  angleZ += gyroZ * dt;

  // Read TOF sensors (distance)
  int dist0 = -1, dist1 = -1, dist2 = -1;

  // Read Sensor 0 (Front)
  digitalWrite(SHUT_PIN_1, LOW);
  digitalWrite(SHUT_PIN_2, LOW);
  delayMicroseconds(200);
  if (initSensor(SHUT_PIN_0)) dist0 = sensor.readRange();
  digitalWrite(SHUT_PIN_0, LOW);
    
  // Read Sensor 1 (Left)
  digitalWrite(SHUT_PIN_0, LOW);
  digitalWrite(SHUT_PIN_2, LOW);
  delayMicroseconds(200);
  if (initSensor(SHUT_PIN_1)) dist1 = sensor.readRange();
  digitalWrite(SHUT_PIN_1, LOW);

  // Read Sensor 2 (Right)
  digitalWrite(SHUT_PIN_0, LOW);
  digitalWrite(SHUT_PIN_1, LOW);
  delayMicroseconds(200);
  if (initSensor(SHUT_PIN_2)) dist2 = sensor.readRange();
  digitalWrite(SHUT_PIN_2, LOW);

  // Print yaw angle and TOF sensor distances in one line
  Serial.printf("Yaw Angle (Z): %6.2f° ", angleZ);
  Serial.print(dist0); Serial.print(" ");
  Serial.print(dist1); Serial.print(" ");
  Serial.println(dist2);

  delay(100);  // Delay to give time for serial output
}

void buzz(int no) {
  switch (no) {
    case 1:
      tone(2, 1500, 100);
      delay(200);
      tone(2, 1000, 100);
      delay(100);
      break;
    case 2:
      tone(2, 1000, 100);
      delay(150);
      tone(2, 1000, 100);
      delay(150);
      break;
  }
}

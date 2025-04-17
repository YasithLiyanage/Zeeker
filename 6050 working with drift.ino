#include <Wire.h>
#include <MPU6050.h>

#define SDA_PIN 33  
#define SCL_PIN 32  

MPU6050 mpu;

float angleZ = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  buzz(1);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }

  lastTime = millis();
}

void loop() {
  int16_t gx_dummy, gy_dummy, gz;
  mpu.getRotation(&gx_dummy, &gy_dummy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gyroZ = gz / 131.0;
  angleZ += gyroZ * dt;

  Serial.printf("Yaw Angle (Z): %6.2f°\n", angleZ);

  delay(10);
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

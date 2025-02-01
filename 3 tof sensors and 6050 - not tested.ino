#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <MPU6050.h>

// Pin Definitions for shutdown pins (for ToF sensors)
#define SHUT_PIN_0 25
#define SHUT_PIN_1 26
#define SHUT_PIN_2 27

// I2C initialization
#define SDA_PIN 21  
#define SCL_PIN 22  

Adafruit_VL6180X sensor;
MPU6050 mpu;

bool initSensor(int shutPin);


void setup() {
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize ToF sensor and MPU6050
  if (!sensor.begin()) {
    Serial.println("Failed to initialize VL6180X sensor.");
  }
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
  }

  // Set shutdown pins for ToF sensors
  pinMode(SHUT_PIN_0, OUTPUT);
  pinMode(SHUT_PIN_1, OUTPUT);
  pinMode(SHUT_PIN_2, OUTPUT);
  
  // Ensure all sensors are OFF
  digitalWrite(SHUT_PIN_0, LOW);
  digitalWrite(SHUT_PIN_1, LOW);
  digitalWrite(SHUT_PIN_2, LOW);
  delay(50);
}

bool initSensor(int shutPin) {
    // Power up the sensor by toggling the shutdown pin
    digitalWrite(shutPin, LOW);  // Ensure the sensor is off
    delayMicroseconds(500);       // Small delay
    digitalWrite(shutPin, HIGH); // Power it up
    delayMicroseconds(1000);      // Wait for it to power up
    return sensor.begin();        // Initialize the sensor
}


void loop() {
  // Read MPU6050 Data (Accelerometer and Gyroscope)
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  // Print MPU6050 data
  Serial.print("Accel X: "); Serial.print(ax); 
  Serial.print(" | Accel Y: "); Serial.print(ay); 
  Serial.print(" | Accel Z: "); Serial.println(az);
  
  // Read ToF Sensors (Front, Left, Right)
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
  
  // Print ToF sensor data
  Serial.print("Dist 0: "); Serial.print(dist0);
  Serial.print(" | Dist 1: "); Serial.print(dist1);
  Serial.print(" | Dist 2: "); Serial.println(dist2);

  delay(100);  // Delay for stability
}

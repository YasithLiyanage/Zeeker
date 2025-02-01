#include <Wire.h>
#include "Adafruit_VL6180X.h"

#define SDA_PIN 21  
#define SCL_PIN 22  

// Shutdown pins for 3 sensors
#define SHUT_PIN_0 25  // Sensor 0 (Front)
#define SHUT_PIN_1 26  // Sensor 1 (Left)
#define SHUT_PIN_2 27  // Sensor 2 (Right)

Adafruit_VL6180X sensor;

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);  // Increase I2C speed

    pinMode(SHUT_PIN_0, OUTPUT);
    pinMode(SHUT_PIN_1, OUTPUT);
    pinMode(SHUT_PIN_2, OUTPUT);

    // Ensure all sensors are OFF before starting
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

    // Print distances in one line
    Serial.print(dist0); Serial.print(" ");
    Serial.print(dist1); Serial.print(" ");
    Serial.println(dist2);
}

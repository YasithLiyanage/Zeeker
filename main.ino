// Pin Definitions
#define PWM_MOTOR1 4
#define PWM_MOTOR2 5
#define MOTOR1_IN1 23
#define MOTOR1_IN2 22
#define MOTOR2_IN3 21
#define MOTOR2_IN4 15

#define IR_SENSOR1_PIN 32
#define IR_SENSOR2_PIN 33
#define IR_SENSOR3_PIN 25



void setup() {
    // Set up motor control pins as outputs
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    pinMode(PWM_MOTOR1, OUTPUT);
    pinMode(PWM_MOTOR2, OUTPUT);

    Serial.begin(115200);
}

void loop() {
  // Read the analog values from the IR sensors
  int sensor1Value = analogRead(IR_SENSOR1_PIN);
  int sensor2Value = analogRead(IR_SENSOR2_PIN);
  int sensor3Value = analogRead(IR_SENSOR3_PIN);


  // Output the values to the Serial Monitor
  Serial.print("Sensor 1: ");
  Serial.print(sensor1Value);
  Serial.print("  | Sensor 2: ");
  Serial.print(sensor2Value);
  Serial.print("  | Sensor 3: ");
  Serial.println(sensor3Value);
 


  // Add a delay for readability
  delay(200);
}
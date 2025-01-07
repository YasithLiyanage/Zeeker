// Define the GPIO pins connected to the IR sensors
#define IR_SENSOR1_PIN 32
#define IR_SENSOR2_PIN 33
#define IR_SENSOR3_PIN 25


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Configure GPIO pins as analog inputs
  // For ESP32, analog pins are defined by ADC channels internally.
  // Pins like 5, 17, 16, and 4 are valid for ADC on the ESP32.
  Serial.println("IR Sensor Array Initialized");
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
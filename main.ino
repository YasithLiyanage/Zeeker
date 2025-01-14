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

#define ENCA1 16  // Encoder A channel A pin
#define ENCA2 17  // Encoder A channel B pin
#define ENCB1 18  // Encoder B channel A pin
#define ENCB2 19  // Encoder B channel B pin

// Encoder variables
volatile int posA = 0;  // Position for encoder A
volatile int posB = 0;  // Position for encoder B
volatile int lastEncodedA = 0;  // Last encoded value for encoder A
volatile int lastEncodedB = 0;  // Last encoded value for encoder B

// State Machine Variables
enum RobotState {
    IDLE,
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    CHECK_SENSORS
};

RobotState currentState = IDLE;  // Initial state
bool isMoving = false;           // Indicates if the robot is moving
int targetPosA = 0;              // Target encoder position for motor A
int targetPosB = 0;              // Target encoder position for motor B

// Function Prototypes
void IRAM_ATTR updateEncoderA();
void IRAM_ATTR updateEncoderB();
void moveForwardNonBlocking(int steps);
void turnLeftNonBlocking(int steps);
void turnRightNonBlocking(int steps);
void stopMotors();
void setMotorSpeed(int motor, int speed);
void checkSensors();
void printMotorPositions();
void buzz(int no);

void setup() {
    // Set up motor control pins as outputs
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    pinMode(PWM_MOTOR1, OUTPUT);
    pinMode(PWM_MOTOR2, OUTPUT);

    // Set up encoder pins as inputs
    pinMode(ENCA1, INPUT);
    pinMode(ENCA2, INPUT);
    pinMode(ENCB1, INPUT);
    pinMode(ENCB2, INPUT);

    // Attach interrupt for encoder channels
    attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

    // Start Serial Monitor
    Serial.begin(115200);

    // Initial buzzer signal
    buzz(1);
}

void loop() {
    // State Machine Logic
    switch (currentState) {
        case IDLE:
            Serial.println("Robot is IDLE...");
            printMotorPositions();
            currentState = MOVE_FORWARD; // Start moving forward
            moveForwardNonBlocking(2000); // Start non-blocking movement
            break;

        case MOVE_FORWARD:
            // Check if target positions are reached
            if (posA >= targetPosA && posB >= targetPosB) {
                stopMotors();
                currentState = CHECK_SENSORS; // Transition to sensor check
            }
            printMotorPositions();
            break;

        case TURN_LEFT:
            // Check if target positions are reached
            if (posA <= targetPosA && posB >= targetPosB) {
                stopMotors();
                currentState = CHECK_SENSORS; // Transition to sensor check
            }
            printMotorPositions();
            break;

        case TURN_RIGHT:
            // Check if target positions are reached
            if (posA >= targetPosA && posB <= targetPosB) {
                stopMotors();
                currentState = CHECK_SENSORS; // Transition to sensor check
            }
            printMotorPositions();
            break;

        case CHECK_SENSORS:
            checkSensors(); // Evaluate sensor data
            printMotorPositions();
            // Example logic to decide the next state
            int sensor2Value = analogRead(IR_SENSOR2_PIN);
            if (sensor2Value > 500) {
                currentState = TURN_LEFT;
                turnLeftNonBlocking(1000);
            } else {
                currentState = MOVE_FORWARD;
                moveForwardNonBlocking(2000);
            }
            break;
    }

    delay(10); // Small delay for stability
}

// Function to move forward (non-blocking)
void moveForwardNonBlocking(int steps) {
    targetPosA = posA + steps;
    targetPosB = posB + steps;

    setMotorSpeed(1, 200); // Motor 1 forward
    setMotorSpeed(2, 200); // Motor 2 forward

    isMoving = true;
}

// Function to turn left (non-blocking)
void turnLeftNonBlocking(int steps) {
    targetPosA = posA - steps;
    targetPosB = posB + steps;

    setMotorSpeed(1, -200); // Motor 1 backward
    setMotorSpeed(2, 200);  // Motor 2 forward

    isMoving = true;
}

// Function to turn right (non-blocking)
void turnRightNonBlocking(int steps) {
    targetPosA = posA + steps;
    targetPosB = posB - steps;

    setMotorSpeed(1, 200);  // Motor 1 forward
    setMotorSpeed(2, -200); // Motor 2 backward

    isMoving = true;
}

// Stop all motors
void stopMotors() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    isMoving = false;
    printMotorPositions();
}

// Set motor speed and direction
void setMotorSpeed(int motor, int speed) {
    if (speed > 0) {
        // Forward
        if (motor == 1) {
            digitalWrite(MOTOR1_IN1, HIGH);
            digitalWrite(MOTOR1_IN2, LOW);
            analogWrite(PWM_MOTOR1, speed);
        } else if (motor == 2) {
            digitalWrite(MOTOR2_IN3, HIGH);
            digitalWrite(MOTOR2_IN4, LOW);
            analogWrite(PWM_MOTOR2, speed);
        }
    } else {
        // Reverse
        speed = abs(speed); // Make speed positive
        if (motor == 1) {
            digitalWrite(MOTOR1_IN1, LOW);
            digitalWrite(MOTOR1_IN2, HIGH);
            analogWrite(PWM_MOTOR1, speed);
        } else if (motor == 2) {
            digitalWrite(MOTOR2_IN3, LOW);
            digitalWrite(MOTOR2_IN4, HIGH);
            analogWrite(PWM_MOTOR2, speed);
        }
    }
}

// Encoder A interrupt service routine
void IRAM_ATTR updateEncoderA() {
    int MSB = digitalRead(ENCA1);  // Most Significant Bit
    int LSB = digitalRead(ENCA2);  // Least Significant Bit
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncodedA << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        posA++;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        posA--;
    }

    lastEncodedA = encoded;
}

// Encoder B interrupt service routine
void IRAM_ATTR updateEncoderB() {
    int MSB = digitalRead(ENCB1);
    int LSB = digitalRead(ENCB2);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncodedB << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        posB++;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        posB--;
    }

    lastEncodedB = encoded;
}

// Function to check sensors
void checkSensors() {
    int sensor1Value = analogRead(IR_SENSOR1_PIN);
    int sensor2Value = analogRead(IR_SENSOR2_PIN);
    int sensor3Value = analogRead(IR_SENSOR3_PIN);

    Serial.print("Sensor 1: ");
    Serial.print(sensor1Value);
    Serial.print(" | Sensor 2: ");
    Serial.print(sensor2Value);
    Serial.print(" | Sensor 3: ");
    Serial.println(sensor3Value);
}

// Function to print motor positions
void printMotorPositions() {
    Serial.print("Motor A Position: ");
    Serial.print(posA);
    Serial.print(" | Motor B Position: ");
    Serial.println(posB);
}

// Buzzer feedback function
void buzz(int no) {
    switch (no) {
        case 1:
            tone(2, 1500, 100); delay(200); tone(2, 1000, 100); delay(100);
            break;
        case 2:
            tone(2, 1000, 100); delay(150); tone(2, 1000, 100); delay(150);
            break;
    }
}

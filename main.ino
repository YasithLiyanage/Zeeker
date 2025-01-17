// Pin Definitions
#define PWM_MOTOR1 4
#define PWM_MOTOR2 5
#define MOTOR1_IN1 23
#define MOTOR1_IN2 22
#define MOTOR2_IN3 15
#define MOTOR2_IN4 21

#define ENCA1 16  // Encoder A channel A pin
#define ENCA2 17  // Encoder A channel B pin
#define ENCB1 18  // Encoder B channel A pin
#define ENCB2 19  // Encoder B channel B pin

// Encoder variables
volatile int posA = 0;  // Position for encoder A
volatile int posB = 0;  // Position for encoder B
volatile int lastEncodedA = 0;  // Last encoded value for encoder A
volatile int lastEncodedB = 0;  // Last encoded value for encoder B

// PID constants
#define KP 1.5  // Proportional constant
#define KI 0.0  // Integral constant
#define KD 0.5  // Derivative constant

// PID variables
float errorA = 0, errorB = 0;         // Current error for encoders A and B
float prevErrorA = 0, prevErrorB = 0; // Previous error for encoders A and B
float integralA = 0, integralB = 0;   // Integral term for encoders A and B
float derivativeA = 0, derivativeB = 0; // Derivative term for encoders A and B

// Target speed for the motors (encoder steps per loop cycle)
int targetSpeedA = 0;
int targetSpeedB = 0;

// State Machine Variables
enum RobotState {
    IDLE,
    TURN_LEFT,
    TURN_RIGHT,
    MOVE_FORWARD
};

RobotState currentState = IDLE;  // Initial state
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
void calculatePID();
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
    // Calculate PID for smooth motor control
    calculatePID();

    // State Machine Logic for Testing Movement
    switch (currentState) {
        case IDLE:
            Serial.println("Testing Forward Movement...");
            moveForwardNonBlocking(2000); // Move forward 2000 steps
            currentState = MOVE_FORWARD;  // Transition to moving forward
            break;

        case MOVE_FORWARD:
            if (posA >= targetPosA && posB >= targetPosB) {
                stopMotors();
                delay(1000);
                turnLeftNonBlocking(1000); // Test turning left
                currentState = TURN_LEFT;  // Transition to turning left
            }
            printMotorPositions();
            break;

        case TURN_LEFT:
            if (posA <= targetPosA && posB >= targetPosB) {
                stopMotors();
                delay(1000);
                turnRightNonBlocking(1000); // Test turning right
                currentState = TURN_RIGHT;  // Transition to turning right
            }
            printMotorPositions();
            break;

        case TURN_RIGHT:
            if (posA >= targetPosA && posB <= targetPosB) {
                stopMotors();
                delay(1000);
                currentState = IDLE; // Restart the test
            }
            printMotorPositions();
            break;
    }

    delay(10); // Small delay for stability
}

// Calculate PID values and set motor speeds
void calculatePID() {
    // Calculate PID for motor A
    errorA = targetSpeedA - (posA - prevErrorA);
    integralA += errorA;
    derivativeA = errorA - prevErrorA;
    int outputA = KP * errorA + KI * integralA + KD * derivativeA;
    prevErrorA = posA;

    // Calculate PID for motor B
    errorB = targetSpeedB - (posB - prevErrorB);
    integralB += errorB;
    derivativeB = errorB - prevErrorB;
    int outputB = KP * errorB + KI * integralB + KD * derivativeB;
    prevErrorB = posB;

    // Set motor speeds using PID outputs
    setMotorSpeed(1, constrain(outputA, -255, 255));
    setMotorSpeed(2, constrain(outputB, -255, 255));
}

// Function to move forward (non-blocking)
void moveForwardNonBlocking(int steps) {
    targetPosA = posA + steps;
    targetPosB = posB + steps;

    targetSpeedA = 200; // Target speed for PID
    targetSpeedB = 200;
}

// Function to turn left (non-blocking)
void turnLeftNonBlocking(int steps) {
    targetPosA = posA - steps;
    targetPosB = posB + steps;

    targetSpeedA = -200;
    targetSpeedB = 200;
}

// Function to turn right (non-blocking)
void turnRightNonBlocking(int steps) {
    targetPosA = posA + steps;
    targetPosB = posB - steps;

    targetSpeedA = 200;
    targetSpeedB = -200;
}

// Stop all motors
void stopMotors() {
    targetSpeedA = 0;
    targetSpeedB = 0;
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    printMotorPositions();
}

// Set motor speed and direction
void setMotorSpeed(int motor, int speed) {
    if (speed > 0) {
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
        speed = abs(speed);
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

// Encoder interrupt handlers
void IRAM_ATTR updateEncoderA() {
    int MSB = digitalRead(ENCA1);
    int LSB = digitalRead(ENCA2);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncodedA << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        posA++;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        posA--;
    }

    lastEncodedA = encoded;
}

void IRAM_ATTR updateEncoderB() {
    int MSB = digitalRead(ENCB1);
    int LSB = digitalRead(ENCB2);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncodedB << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        posB--;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        posB++;
    }

    lastEncodedB = encoded;
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

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

// State Machine Variables
enum RobotState {
    IDLE,
    TURN_LEFT,
    TURN_RIGHT
};

RobotState currentState = IDLE;  // Initial state
int targetPosA = 0;              // Target encoder position for motor A
int targetPosB = 0;              // Target encoder position for motor B

// Function Prototypes
void IRAM_ATTR updateEncoderA();
void IRAM_ATTR updateEncoderB();
void turnLeftNonBlocking(int steps);
void turnRightNonBlocking(int steps);
void stopMotors();
void setMotorSpeed(int motor, int speed);
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
    // State Machine Logic for Testing Movement
    switch (currentState) {
        case IDLE:
            Serial.println("Testing Forward Movement...");
            moveForwardNonBlocking(2000); // Move forward 2000 steps (adjust this value for calibration)
            currentState = MOVE_FORWARD;  // Transition to moving forward
            break;

        case MOVE_FORWARD:
            // Check if target positions are reached
            if (posA >= targetPosA && posB >= targetPosB) {
                stopMotors();
                delay(1000); // Pause to observe before next action
                turnLeftNonBlocking(1000); // Test turning left
                currentState = TURN_LEFT;  // Transition to turning left
            }
            printMotorPositions();
            break;

        case TURN_LEFT:
            // Check if target positions are reached
            if (posA <= targetPosA && posB >= targetPosB) {
                stopMotors();
                delay(1000); // Pause to observe
                turnRightNonBlocking(1000); // Test turning right
                currentState = TURN_RIGHT;  // Transition to turning right
            }
            printMotorPositions();
            break;

        case TURN_RIGHT:
            // Check if target positions are reached
            if (posA >= targetPosA && posB <= targetPosB) {
                stopMotors();
                delay(1000); // Pause to observe
                currentState = IDLE; // Restart the test
            }
            printMotorPositions();
            break;
    }

    delay(10); // Small delay for stability
}


// Function to move forward (non-blocking)
void moveForwardNonBlocking(int steps) {
    targetPosA = posA + steps; // Target position for Motor 1
    targetPosB = posB + steps; // Target position for Motor 2

    setMotorSpeed(1, 200); // Motor 1 forward
    setMotorSpeed(2, 200); // Motor 2 forward
}

// Function to turn left (non-blocking)
void turnLeftNonBlocking(int steps) {
    targetPosA = posA - steps;
    targetPosB = posB + steps;

    setMotorSpeed(1, -200); // Motor 1 backward
    setMotorSpeed(2, 200);  // Motor 2 forward
}

// Function to turn right (non-blocking)
void turnRightNonBlocking(int steps) {
    targetPosA = posA + steps;
    targetPosB = posB - steps;

    setMotorSpeed(1, 200);  // Motor 1 forward
    setMotorSpeed(2, -200); // Motor 2 backward
}

// Stop all motors
void stopMotors() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
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

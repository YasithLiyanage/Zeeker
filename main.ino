#define IR_SENSOR_FRONT 33   // Pin for the front IR sensor
#define IR_SENSOR_LEFT 32   // Pin for the left IR sensor
#define IR_SENSOR_RIGHT 25   // Pin for the right IR sensor

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

int targetPosA = 0; // Target position for encoder A
int targetPosB = 0; // Target position for encoder B

// Thresholds for IR sensors
int frontThreshold = 3000; // Adjust this threshold based on your testing
int leftThreshold = 3900;  // Adjust this threshold based on your testing
int rightThreshold = 3900; // Adjust this threshold based on your testing

// Function Prototypes
void IRAM_ATTR updateEncoderA();
void IRAM_ATTR updateEncoderB();
void turnLeftBlocking(int steps);
void turnRightBlocking(int steps);
void moveForwardBlocking(int steps);
void stopMotors();
void setMotorSpeed(int motor, int speed);
void printMotorPositions();
void buzz(int no);

bool isWallFront();
bool isWallLeft();
bool isWallRight();

void setup() {
    // IR sensors setup
    pinMode(IR_SENSOR_FRONT, INPUT);
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    // Motor control pins setup
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    pinMode(PWM_MOTOR1, OUTPUT);
    pinMode(PWM_MOTOR2, OUTPUT);

    // Encoder pins setup
    pinMode(ENCA1, INPUT);
    pinMode(ENCA2, INPUT);
    pinMode(ENCB1, INPUT);
    pinMode(ENCB2, INPUT);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

    // Start serial monitor
    Serial.begin(115200);

    // Initial buzzer signal
    buzz(1);
}

void loop() {
    // Read sensor values and print them to the serial monitor
    int frontValue = analogRead(IR_SENSOR_FRONT);
    int leftValue = analogRead(IR_SENSOR_LEFT);
    int rightValue = analogRead(IR_SENSOR_RIGHT);

    Serial.print("Left : ");
    Serial.print(leftValue);
    Serial.print(" | Front : ");
    Serial.print(frontValue);
    Serial.print(" | Right : ");
    Serial.println(rightValue);

    // Left hand wall-following logic
    if (isWallLeft()) {
        if (isWallFront()) {
            // Turn right if wall is in front and on the left
            Serial.println("Wall detected on front and left. Turning right...");
            turnRightBlocking(670);
        } else {
            // Move forward if no wall in front but wall on left
            Serial.println("Wall detected on left. Moving forward...");
            moveForwardBlocking(1800);
        }
    } else {
        // Turn left if no wall on the left
        Serial.println("No wall on left. Turning left...");
        turnLeftBlocking(670);
    }

    delay(100); // Small delay for stability
}

bool isWallFront() {
    return analogRead(IR_SENSOR_FRONT) < frontThreshold;
}

bool isWallLeft() {
    return analogRead(IR_SENSOR_LEFT) < leftThreshold;
}

bool isWallRight() {
    return analogRead(IR_SENSOR_RIGHT) < rightThreshold;
}

void moveForwardBlocking(int steps) {
    posA = 0;
    posB = 0;
    targetPosA = steps;
    targetPosB = steps;

    setMotorSpeed(1, 100);
    setMotorSpeed(2, 100);

    while (posA < targetPosA || posB < targetPosB) {
        if (isWallFront()) {
            Serial.println("Front wall detected while moving forward. Adjusting...");
            stopMotors();
            return;
        }
        printMotorPositions();
    }

    stopMotors();
}

void turnLeftBlocking(int steps) {
    posA = 0;
    posB = 0;
    targetPosA = -steps;
    targetPosB = steps;

    setMotorSpeed(1, -100);
    setMotorSpeed(2, 100);

    while (posA > targetPosA || posB < targetPosB) {
        if (isWallFront()) {
            Serial.println("Front wall detected while turning left. Continuing turn...");
        }
        printMotorPositions();
    }

    stopMotors();
}

void turnRightBlocking(int steps) {
    posA = 0;
    posB = 0;
    targetPosA = steps;
    targetPosB = -steps;

    setMotorSpeed(1, 100);
    setMotorSpeed(2, -100);

    while (posA < targetPosA || posB > targetPosB) {
        if (isWallFront()) {
            Serial.println("Front wall detected while turning right. Continuing turn...");
        }
        printMotorPositions();
    }

    stopMotors();
}

void stopMotors() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    printMotorPositions();
}

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

void printMotorPositions() {
    Serial.print("Motor A Position: ");
    Serial.print(posA);
    Serial.print(" | Motor B Position: ");
    Serial.println(posB);
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
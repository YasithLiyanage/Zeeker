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
#define ENCB1 18 // Encoder B channel A pin
#define ENCB2 19 // Encoder B channel B pin

volatile int posA = 0;  // Position for encoder A
volatile int posB = 0;  // Position for encoder B
volatile int lastEncodedA = 0;  // Used for storing the last encoded value of encoder A
volatile int lastEncodedB = 0;  // Used for storing the last encoded value of encoder B

// Forward declaration for the interrupt functions
void IRAM_ATTR updateEncoderA();
void IRAM_ATTR updateEncoderB();

bool isTurning = false;  // Indicates if the robot is currently turning
int targetPosA = 0;      // Target position for encoder A
int targetPosB = 0;      // Target position for encoder B
int turnDirection = 0;   // 1 for right, -1 for left, 0 for no turn





void setup() {
    // Set up motor control pins as outputs
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    pinMode(PWM_MOTOR1, OUTPUT);
    pinMode(PWM_MOTOR2, OUTPUT);

    pinMode(ENCA1, INPUT);
    pinMode(ENCA2, INPUT);
    pinMode(ENCB1, INPUT);
    pinMode(ENCB2, INPUT);

    // Attach interrupt for encoder A channels
    attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);  // Track both RISING and FALLING
    attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);  // Track both RISING and FALLING

    // Attach interrupt for encoder B channels
    attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);  // Track both RISING and FALLING
    attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);  // Track both RISING and FALLING

    buzz(1);

    Serial.begin(115200);
}




void loop() {
    // Read the analog values from the IR sensors
    int sensor1Value = analogRead(IR_SENSOR1_PIN);
    int sensor2Value = analogRead(IR_SENSOR2_PIN);
    int sensor3Value = analogRead(IR_SENSOR3_PIN);

    // Output the sensor values to the Serial Monitor
    Serial.print("Sensor 1: ");
    Serial.print(sensor1Value);
    Serial.print("  | Sensor 2: ");
    Serial.print(sensor2Value);
    Serial.print("  | Sensor 3: ");
    Serial.print(sensor3Value);
    Serial.print("  |  ");

    // Output the motor positions
    Serial.print("Motor A Position: ");
    Serial.print(posA);
    Serial.print("   Motor B Position: ");
    Serial.println(posB);

    // Handle motor turning if a turn is in progress
    if (isTurning) {
        if ((turnDirection == 1 && (posA >= targetPosA || posB <= targetPosB)) ||  // Turning right
            (turnDirection == -1 && (posA <= targetPosA || posB >= targetPosB))) { // Turning left
            stopMotors();
            isTurning = false;  // Mark turn as completed
        }
    } else {
        // Start a new turn
        if (turnDirection == 0) {  // If no turn in progress
            turnLeftNonBlocking(1000);  // Start turning left
        } else if (turnDirection == -1) {
            turnRightNonBlocking(1000);  // Start turning right
        }
    }

    delay(100); // Small delay for readability in Serial Monitor
}







void turnRightNonBlocking(int steps) {
    targetPosA = posA + steps;
    targetPosB = posB - steps;

    setMotorSpeed(1, 200);  // Motor 1 forward
    setMotorSpeed(2, -200); // Motor 2 backward

    isTurning = true;
    turnDirection = 1;  // Indicate right turn
}

void turnLeftNonBlocking(int steps) {
    targetPosA = posA - steps;
    targetPosB = posB + steps;

    setMotorSpeed(1, -200); // Motor 1 backward
    setMotorSpeed(2, 200);  // Motor 2 forward

    isTurning = true;
    turnDirection = -1;  // Indicate left turn
}


void stopMotors() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    turnDirection = 0;  // Reset direction
}










void setMotorSpeed(int motor, int speed) {
    if (speed > 0) {
        // Motor moves forward
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
        // Motor moves in reverse
        speed = abs(speed); // Ensure speed is positive for PWM
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






// Define the interrupt service routine for encoder A
void IRAM_ATTR updateEncoderA(){
    // Read the current state of both channels
    int MSB_A = digitalRead(ENCA1);  // Most Significant Bit for encoder A
    int LSB_A = digitalRead(ENCA2);  // Least Significant Bit for encoder A

    int encodedA = (MSB_A << 1) | LSB_A;  // Combine the two bits into a single integer
    int sumA = (lastEncodedA << 2) | encodedA;  // Shift and combine with the last encoded value

    // Determine the direction based on the state transitions for encoder A
    if (sumA == 0b1101 || sumA == 0b0100 || sumA == 0b0010 || sumA == 0b1011){
        posA++;
    }
    else if (sumA == 0b1110 || sumA == 0b0111 || sumA == 0b0001 || sumA == 0b1000){
        posA--;
    }

    lastEncodedA = encodedA;  // Store the current encoded value for the next iteration
}

// Define the interrupt service routine for encoder B
void IRAM_ATTR updateEncoderB(){
    // Read the current state of both channels
    int MSB_B = digitalRead(ENCB1);  //Most Significant Bit for encoder B
    int LSB_B = digitalRead(ENCB2);  // Least Significant Bit for encoder B

    int encodedB = (MSB_B << 1) | LSB_B;  // Combine the two bits into a single integer
    int sumB = (lastEncodedB << 2) | encodedB;  // Shift and combine with the last encoded value

    // Determine the direction based on the state transitions for encoder B
    if (sumB == 0b1101 || sumB == 0b0100 || sumB == 0b0010 || sumB == 0b1011){
        posB++;
    }
    else if (sumB == 0b1110 || sumB == 0b0111 || sumB == 0b0001 || sumB == 0b1000){
        posB--;
    }

    lastEncodedB = encodedB;  // Store the current encoded value for the next iteration
}





// BUZZER FEEDBACK
void buzz(int no) {
    switch(no) {
        case 1 :
            tone(2, 1500, 100);  // 1500 Hz tone for 100 ms
            delay(200);
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            break;

        case 2 :
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(150);
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(150);
            break;

        case 3 :
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            break;

        case 31 :
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            break;

        case 32 :
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            break;

        case 33 :
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            tone(2, 1000, 100);  // 1000 Hz tone for 100 ms
            delay(100);
            break;
    }
}
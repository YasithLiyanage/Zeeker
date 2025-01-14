// Pin Definitions
#define PWM_MOTOR1 4
#define PWM_MOTOR2 5
#define MOTOR1_IN1 23
#define MOTOR1_IN2 22
#define MOTOR2_IN3 21
#define MOTOR2_IN4 15

void setup() {
    // Set up motor control pins as outputs
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    pinMode(PWM_MOTOR1, OUTPUT);
    pinMode(PWM_MOTOR2, OUTPUT);

    Serial.begin(115200);
    buzz(1);
}

void loop() {
    // Both motors forward at full speed
    Serial.println("Motors Forward at Full Speed");
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN3, HIGH);
    digitalWrite(MOTOR2_IN4, LOW);
    analogWrite(PWM_MOTOR1, 155); // Full speed
    analogWrite(PWM_MOTOR2, 155); // Full speed
    delay(1000); // Run for 2 seconds

    // Both motors reverse at full speed
    Serial.println("Motors Reverse at Full Speed");
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN3, LOW);
    digitalWrite(MOTOR2_IN4, HIGH);
    analogWrite(PWM_MOTOR1, 155); // Full speed
    analogWrite(PWM_MOTOR2, 155); // Full speed
    delay(1000); // Run for 2 seconds

    // Stop motors
    Serial.println("Stopping Motors");
    analogWrite(PWM_MOTOR1, 0);
    analogWrite(PWM_MOTOR2, 0);
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN3, LOW);
    digitalWrite(MOTOR2_IN4, LOW);
    delay(2000); // Pause before repeating
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
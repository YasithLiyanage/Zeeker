#define IR_SENSOR_FRONT 33   // Pin for the front IR sensor
#define IR_SENSOR_LEFT 32   // Pin for the left IR sensor
#define IR_SENSOR_RIGHT 25  // Pin for the right IR sensor

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

#define MAZE_ROWS 3
#define MAZE_COLS 5

// Encoder variables
volatile int posA = 0;
volatile int posB = 0;
volatile int lastEncodedA = 0;
volatile int lastEncodedB = 0;

int targetPosA = 0;
int targetPosB = 0;

// Thresholds for IR sensors
int frontThreshold = 3000;
int leftThreshold = 3900;
int rightThreshold = 3900;

int maze[MAZE_ROWS][MAZE_COLS];
int currentRow = 2;
int currentCol = 0;
int goalRow = 1;
int goalCol = 2;

// Function prototypes
void initializeFloodFill();
void updateMazeWalls(int row, int col);
void moveMicromouse();
bool isWallFront();
bool isWallLeft();
bool isWallRight();
void moveForwardBlocking(int steps);
void turnLeftBlocking(int steps);
void turnRightBlocking(int steps);
void stopMotors();
void setMotorSpeed(int motor, int speed);
void printMaze();
void buzz(int no);

void setup() {
    pinMode(IR_SENSOR_FRONT, INPUT);
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

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

    attachInterrupt(digitalPinToInterrupt(ENCA1), [](){ /* encoder logic */ }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB1), [](){ /* encoder logic */ }, CHANGE);

    Serial.begin(115200);

    initializeFloodFill();
    Serial.println("Flood fill initialized.");
    printMaze();

    buzz(1); // Initial buzz to indicate startup
}

void loop() {
    while (!(currentRow == goalRow && currentCol == goalCol)) {
        updateMazeWalls(currentRow, currentCol);
        moveMicromouse();
        printMaze();
        delay(100);
    }
    Serial.println("Goal reached!");
    buzz(2); // Indicate goal reached
    while (1); // Stop the loop
}

void initializeFloodFill() {
    for (int i = 0; i < MAZE_ROWS; i++) {
        for (int j = 0; j < MAZE_COLS; j++) {
            maze[i][j] = abs(goalRow - i) + abs(goalCol - j);
        }
    }
}

void updateMazeWalls(int row, int col) {
    if (isWallFront() && row > 0) maze[row - 1][col] = -1;
    if (isWallLeft() && col > 0) maze[row][col - 1] = -1;
    if (isWallRight() && col < MAZE_COLS - 1) maze[row][col + 1] = -1;
}

void moveMicromouse() {
    int nextRow = currentRow;
    int nextCol = currentCol;
    int minDistance = maze[currentRow][currentCol];

    if (currentRow > 0 && maze[currentRow - 1][currentCol] >= 0 && maze[currentRow - 1][currentCol] < minDistance) {
        nextRow = currentRow - 1;
        nextCol = currentCol;
        minDistance = maze[currentRow - 1][currentCol];
    }
    if (currentRow < MAZE_ROWS - 1 && maze[currentRow + 1][currentCol] >= 0 && maze[currentRow + 1][currentCol] < minDistance) {
        nextRow = currentRow + 1;
        nextCol = currentCol;
        minDistance = maze[currentRow + 1][currentCol];
    }
    if (currentCol > 0 && maze[currentRow][currentCol - 1] >= 0 && maze[currentRow][currentCol - 1] < minDistance) {
        nextRow = currentRow;
        nextCol = currentCol - 1;
        minDistance = maze[currentRow][currentCol - 1];
    }
    if (currentCol < MAZE_COLS - 1 && maze[currentRow][currentCol + 1] >= 0 && maze[currentRow][currentCol + 1] < minDistance) {
        nextRow = currentRow;
        nextCol = currentCol + 1;
        minDistance = maze[currentRow][currentCol + 1];
    }

    if (nextRow < currentRow) turnLeftBlocking(180);
    else if (nextRow > currentRow) turnRightBlocking(180);
    else if (nextCol > currentCol) moveForwardBlocking(180);
    else if (nextCol < currentCol) turnLeftBlocking(180);

    currentRow = nextRow;
    currentCol = nextCol;
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
            stopMotors();
            return;
        }
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

    while (posA > targetPosA || posB < targetPosB);
    stopMotors();
}

void turnRightBlocking(int steps) {
    posA = 0;
    posB = 0;
    targetPosA = steps;
    targetPosB = -steps;

    setMotorSpeed(1, 100);
    setMotorSpeed(2, -100);

    while (posA < targetPosA || posB > targetPosB);
    stopMotors();
}

void stopMotors() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
}

void setMotorSpeed(int motor, int speed) {
    if (motor == 1) {
        digitalWrite(MOTOR1_IN1, speed > 0);
        digitalWrite(MOTOR1_IN2, speed <= 0);
        analogWrite(PWM_MOTOR1, abs(speed));
    } else if (motor == 2) {
        digitalWrite(MOTOR2_IN3, speed > 0);
        digitalWrite(MOTOR2_IN4, speed <= 0);
        analogWrite(PWM_MOTOR2, abs(speed));
    }
}

void printMaze() {
    for (int i = 0; i < MAZE_ROWS; i++) {
        for (int j = 0; j < MAZE_COLS; j++) {
            Serial.print(maze[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();
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

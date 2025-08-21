#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <MPU9250_asukiaaa.h>
#include <math.h>

// ========== Motor pins ==========
#define PWM_LEFT    4
#define PWM_RIGHT   5
#define IN1_LEFT    15
#define IN2_LEFT    21
#define IN1_RIGHT   22
#define IN2_RIGHT   23

// ========== Encoders (quadrature, A & B) ==========
#define ENCA1 16  // Left encoder A
#define ENCA2 17  // Left encoder B
#define ENCB1 18  // Right encoder A
#define ENCB2 19  // Right encoder B

volatile int posA = 0;           // left ticks
volatile int posB = 0;           // right ticks
volatile int lastEncodedA = 0;   // last AB snapshot (left)
volatile int lastEncodedB = 0;   // last AB snapshot (right)

void IRAM_ATTR updateEncoderA() {
  int msb = digitalRead(ENCA1);
  int lsb = digitalRead(ENCA2);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncodedA << 2) | encoded;

  // Gray code decode
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posA++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posA--;

  lastEncodedA = encoded;
}
void IRAM_ATTR updateEncoderB() {
  int msb = digitalRead(ENCB1);
  int lsb = digitalRead(ENCB2);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncodedB << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posB--;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posB++;

  lastEncodedB = encoded;
}

// ----- Buzzer (non-blocking, dedicated LEDC channel) -----
#define BUZZER_PIN 2
#define BUZZ_CH    7      // use a channel not used by your motor PWMs
bool BUZZ_ENABLED = true;

static unsigned long buzz_end_ms = 0;

void buzzerBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZ_CH, 2000 /*Hz*/, 8 /*bits*/);
  ledcAttachPin(BUZZER_PIN, BUZZ_CH);
  ledcWriteTone(BUZZ_CH, 0);     // ensure off
}

// start a single beep without blocking
void buzzStart(int freqHz, int durMs) {
  if (!BUZZ_ENABLED) return;
  ledcWriteTone(BUZZ_CH, freqHz);
  buzz_end_ms = millis() + durMs;
}

// call this every loop() to auto-stop tone
void buzzUpdate() {
  if (buzz_end_ms != 0 && (long)(millis() - buzz_end_ms) >= 0) {
    ledcWriteTone(BUZZ_CH, 0);
    buzz_end_ms = 0;
  }
}

// optional helpers (short/ok and double-beep patterns, non-blocking)
void buzzOK()        { buzzStart(1500, 80); }     // quick chirp
void buzzAttention() { buzzStart(300, 50); }    // slightly 


// ========== I2C / MUX ==========
#define SDA_PIN     25
#define SCL_PIN     26
#define MUX_ADDR    0x70

// MUX channels (adjust to your wiring)
#define CH_LEFT      0
#define CH_ALEFT45   1
#define CH_FRONT     2
#define CH_ARIGHT45  3
#define CH_RIGHT     4

Adafruit_VL6180X tof;
MPU9250_asukiaaa mpu;

// ========== Calibration: mm = a*raw + b ==========
struct Cal { float a, b; };
enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };

Cal CAL[5] = {
  {1.00f, -42.0f},  // LEFT
  {1.00f,  20.0f},  // ALEFT45
  {1.00f, -35.0f},  // FRONT
  {1.00f,   0.0f},  // ARIGHT45
  {1.00f, -28.0f}   // RIGHT
};

// ========== Driving params ==========
#define NO_WALL_MM        100   // > this => treat as "no wall"
#define TARGET_SIDE_MM    50
#define FRONT_BLOCK_MM    32    // used for junction decisions (debounced)
#define FRONT_CLEAR_MM    45
#define FRONT_SLOW_MM     80
#define FRONT_STOP_MM     38    // normal stop threshold once committed
#define EMERGENCY_MM      28    // ALWAYS stop if at/under this, any time

// Commit distance into a new cell before we use FRONT_STOP_MM
// (kept small so we don't tunnel into a short cell)
#define COMMIT_MM         20

// ========== Square-up params ==========
#define A45_TOL_MM        6
#define SQUARE_TURN_PWM   50
#define SQUARE_MAX_MS     1500
#define GAP_TOL_MM        4
#define NUDGE_PWM         60

// ========== WALL MAPPING ==========
#define MAZE_SIZE 4  // 16x16 maze (adjust as needed)

// Wall bits for each cell
#define WALL_NORTH  0x01
#define WALL_EAST   0x02
#define WALL_SOUTH  0x04
#define WALL_WEST   0x08
#define VISITED     0x10


// Global variable to hold the path as a string
String fullPath = "";

// Maze storage
uint8_t maze[MAZE_SIZE][MAZE_SIZE];

// Robot position and orientation
int robotX = 0, robotY = 0;
int robotHeading = 0; // 0=North, 1=East, 2=South, 3=West
int stepCount = 0;

// Direction vectors (FIXED: North should increase Y, East should increase X)
int dx[] = {0, 1, 0, -1}; // North, East, South, West  
int dy[] = {1, 0, -1, 0}; // North=+Y, East=+X, South=-Y, West=-X
const char* dirNames[] = {"North", "East", "South", "West"};

// ========== IMU state ==========
static float gyroBiasZ = 0.0f;

// ========== IMU state ==========
//static float gyroBiasZ = 0.0f;

// ======================================================
// ================  MUX + ToF helpers  =================
// ======================================================
void mux(uint8_t ch){
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}
bool tofBegin(uint8_t ch){ mux(ch); delay(3); return tof.begin(); }

float readMM(SensorId sid, uint8_t ch){
  mux(ch); delay(2);
  uint8_t r = tof.readRange();
  if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
  float mm = CAL[sid].a * r + CAL[sid].b;
  if (mm < 0) mm = 0;
  if (mm > NO_WALL_MM) return -1.0f;
  return mm;
}

// Initialize maze (add boundary walls)
void initMaze() {
  // Clear all walls
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      maze[x][y] = 0;
    }
  }
  
  // Add boundary walls
  for (int i = 0; i < MAZE_SIZE; i++) {
    maze[i][0] |= WALL_SOUTH;           // South boundary
    maze[i][MAZE_SIZE-1] |= WALL_NORTH; // North boundary
    maze[0][i] |= WALL_WEST;            // West boundary
    maze[MAZE_SIZE-1][i] |= WALL_EAST;  // East boundary
  }
  
  Serial.println("Maze initialized with boundary walls");
}

// Add wall to maze
void addWall(int x, int y, uint8_t wallBit) {
  if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
    maze[x][y] |= wallBit;
    
    // Also add corresponding wall to adjacent cell
    int adjX = x, adjY = y;
    uint8_t adjWallBit = 0;
    
    switch (wallBit) {
      case WALL_NORTH:
        adjY++; adjWallBit = WALL_SOUTH;
        break;
      case WALL_EAST:
        adjX++; adjWallBit = WALL_WEST;
        break;
      case WALL_SOUTH:
        adjY--; adjWallBit = WALL_NORTH;
        break;
      case WALL_WEST:
        adjX--; adjWallBit = WALL_EAST;
        break;
    }
    
    if (adjX >= 0 && adjX < MAZE_SIZE && adjY >= 0 && adjY < MAZE_SIZE) {
      maze[adjX][adjY] |= adjWallBit;
    }
  }
}

// Scan and record walls at current position
void scanWalls() {
  Serial.printf("\n[SCAN] Position (%d,%d) facing %s\n", robotX, robotY, dirNames[robotHeading]);
  
  // Mark current cell as visited
  maze[robotX][robotY] |= VISITED;
  
  // Read sensors
  float L = readMM(SID_LEFT,  CH_LEFT);
  float F = readMM(SID_FRONT, CH_FRONT);
  float R = readMM(SID_RIGHT, CH_RIGHT);
  
  Serial.printf("[SCAN] Sensor readings: L=%.1fmm F=%.1fmm R=%.1fmm\n", L, F, R);
  
  // Determine wall directions relative to robot's current heading
  int leftDir = (robotHeading + 3) % 4;   // Turn left
  int frontDir = robotHeading;            // Straight
  int rightDir = (robotHeading + 1) % 4;  // Turn right
  
  // Convert directions to wall bits
  uint8_t wallBits[] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
  
  // Check and record walls
  if (L >= 0 && L < NO_WALL_MM) {
    addWall(robotX, robotY, wallBits[leftDir]);
    Serial.printf("[SCAN] Wall detected on LEFT (%s)\n", dirNames[leftDir]);
  }
  
  if (F >= 0 && F < NO_WALL_MM) {
    addWall(robotX, robotY, wallBits[frontDir]);
    Serial.printf("[SCAN] Wall detected on FRONT (%s)\n", dirNames[frontDir]);
  }
  
  if (R >= 0 && R < NO_WALL_MM) {
    addWall(robotX, robotY, wallBits[rightDir]);
    Serial.printf("[SCAN] Wall detected on RIGHT (%s)\n", dirNames[rightDir]);
  }
}

// Update robot position after moving forward
void updatePosition() {
  int oldX = robotX, oldY = robotY;

  robotX += dx[robotHeading];
  robotY += dy[robotHeading];

  // ✅ if didn’t actually move, skip
  if (robotX == oldX && robotY == oldY) {
    Serial.println("[WARN] updatePosition called but no movement. Skipping.");
    return;
  }

  stepCount++;

  // clamp to maze bounds
  if (robotX < 0) robotX = 0;
  if (robotX >= MAZE_SIZE) robotX = MAZE_SIZE - 1;
  if (robotY < 0) robotY = 0;
  if (robotY >= MAZE_SIZE) robotY = MAZE_SIZE - 1;

  // append step to path string
  char stepChar;
  if (robotX > oldX) stepChar = 'E';
  else if (robotX < oldX) stepChar = 'W';
  else if (robotY > oldY) stepChar = 'N';
  else if (robotY < oldY) stepChar = 'S';
  else stepChar = '?';

  fullPath += stepChar;
  fullPath += ' ';

  Serial.printf("[MOVE] Step %d: (%d,%d) -> (%d,%d) [%c]\n",
                stepCount, oldX, oldY, robotX, robotY, stepChar);

  Serial.printf("[PATH] %s\n", fullPath.c_str());
}



// Update robot heading after turning
void updateHeading(int turnDegrees) {
  if (turnDegrees == 90) {        // Left turn
    robotHeading = (robotHeading + 3) % 4;
  } else if (turnDegrees == -90) { // Right turn
    robotHeading = (robotHeading + 1) % 4;
  } else if (turnDegrees == 180) { // U-turn
    robotHeading = (robotHeading + 2) % 4;
  }
  
  Serial.printf("[TURN] Turned %d degrees, now facing %s\n", 
                turnDegrees, dirNames[robotHeading]);
}

// Print current maze map
void printMaze() {
  Serial.println("\n========== CURRENT MAZE MAP ==========");
  Serial.printf("Robot at (%d,%d) facing %s | Steps: %d\n", 
                robotX, robotY, dirNames[robotHeading], stepCount);
  Serial.println("Legend: R=Robot, *=Visited, .=Unknown, +=Wall");
  
  // Print top border
  Serial.print("  ");
  for (int x = 0; x < MAZE_SIZE; x++) {
    Serial.printf("%2d", x);
  }
  Serial.println();
  
  // Print maze from top to bottom (North to South)
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    Serial.printf("%2d", y);
    
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (x == robotX && y == robotY) {
        Serial.print(" R"); // Robot position
      } else if (maze[x][y] & VISITED) {
        Serial.print(" *"); // Visited cell
      } else {
        Serial.print(" ."); // Unknown cell
      }
    }
    Serial.println();
  }
  
  // Print detailed wall information for visited cells
  Serial.println("\n--- DETAILED WALL MAP ---");
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (maze[x][y] & VISITED) {
        Serial.printf("Cell (%d,%d): ", x, y);
        if (maze[x][y] & WALL_NORTH) Serial.print("N ");
        if (maze[x][y] & WALL_EAST)  Serial.print("E ");
        if (maze[x][y] & WALL_SOUTH) Serial.print("S ");
        if (maze[x][y] & WALL_WEST)  Serial.print("W ");
        Serial.println();
      }
    }
  }
  Serial.println("=====================================\n");
}

// ======================================================
/**                     Motors                          */
// ======================================================
void motorL(int pwm){
  pwm = constrain(pwm, -255, 255);
  bool fwd = (pwm >= 0);
  digitalWrite(IN1_LEFT,  fwd ? HIGH : LOW);
  digitalWrite(IN2_LEFT,  fwd ? LOW  : HIGH);
  analogWrite(PWM_LEFT, abs(pwm));
}
void motorR(int pwm){
  pwm = constrain(pwm, -255, 255);
  bool fwd = (pwm >= 0);
  digitalWrite(IN1_RIGHT, fwd ? HIGH : LOW);
  digitalWrite(IN2_RIGHT, fwd ? LOW  : HIGH);
  analogWrite(PWM_RIGHT, abs(pwm));
}
void motorsStop(){ motorL(0); motorR(0); }

// ======================================================
// ============  Wall-follow (your PD)  =================
// ======================================================
void driveCentered(int basePWM, float err){
  static float err_prev = 0, err_sum = 0;
  static unsigned long last = millis();
  unsigned long now = millis();
  float dt = (now - last) / 1000.0f; if (dt <= 0) dt = 0.001f;
  last = now;

  const float Kp = 0.8f, Ki = 0.0f, Kd = 0.1f;
  err_sum += err * dt;
  float derr = (err - err_prev) / dt;
  float turn = Kp*err + Ki*err_sum + Kd*derr;
  err_prev = err;

  int l = constrain((int)(basePWM + turn), 0, 255);
  int r = constrain((int)(basePWM - turn), 0, 255);
  motorL(l); motorR(r);
}

// ======================================================
// ================  Square-up routine  =================
// ======================================================
void squareUp(){
  Serial.println(">>> Square-up START");
  unsigned long t0 = millis();

  while (millis() - t0 < SQUARE_MAX_MS){
    float al = readMM(SID_ALEFT45,  CH_ALEFT45);
    float ar = readMM(SID_ARIGHT45, CH_ARIGHT45);
    if (al < 0 || ar < 0) break;

    float diff = ar - al;  // positive => rotate left
    if (fabs(diff) <= A45_TOL_MM) break;

    if (diff > 0){  // left is farther -> rotate left
      motorL(-SQUARE_TURN_PWM); motorR(SQUARE_TURN_PWM);
    } else {        // right farther -> rotate right
      motorL(SQUARE_TURN_PWM);  motorR(-SQUARE_TURN_PWM);
    }

    buzzUpdate();  // let buzzer auto-stop while aligning

    delay(20);
  }
  motorsStop();

  // Set final front gap
  float f = readMM(SID_FRONT, CH_FRONT);
  if (f > 0 && fabs(f - FRONT_STOP_MM) > GAP_TOL_MM){
    int dir = (f > FRONT_STOP_MM) ? +1 : -1; // too far -> forward; too close -> back
    motorL(dir * NUDGE_PWM); motorR(dir * NUDGE_PWM);
    delay(150);
    motorsStop();
  }
  Serial.println(">>> Square-up DONE");
}

// ======================================================
// ====================  IMU turns  =====================
// ======================================================
void imuBegin(){
  mpu.setWire(&Wire);
  mpu.beginGyro();
  // bias (keep robot still)
  unsigned long t0 = millis();
  double sum=0; long n=0;
  while (millis()-t0 < 1200){
    mpu.gyroUpdate();
    sum += mpu.gyroZ();
    n++; delay(2);
  }
  gyroBiasZ = (n>0) ? (float)(sum/n) : 0.0f;
  Serial.printf("IMU bias Z=%.3f deg/s\n", gyroBiasZ);
}

// +angleDeg = left/CCW; -angleDeg = right/CW
void turnIMU(float angleDeg, int basePWM=90, int timeout_ms=1500){
  if (angleDeg == 0) return;
  int dir = (angleDeg > 0) ? +1 : -1;
  float target = fabsf(angleDeg);

  float yaw = 0.0f;
  unsigned long last = millis();
  unsigned long t0 = last;

  const float slowBand = 18.0f; // start slowing when within this many degrees
  const int   minPWM   = 70;

  while (true){
    if ((int)(millis()-t0) > timeout_ms) break;

    mpu.gyroUpdate();
    float gz = mpu.gyroZ() - gyroBiasZ; // deg/s
    unsigned long now = millis();
    float dt = (now - last)/1000.0f; if (dt <= 0) dt = 0.001f;
    last = now;

    yaw += fabsf(gz)*dt;
    if (yaw >= target) break;

    float remaining = target - yaw;
    float scale = (remaining < slowBand) ? fmaxf(remaining/slowBand, 0.2f) : 1.0f;
    int pwm = (int)fmaxf((basePWM*scale), (float)minPWM);

    // in-place spin
    motorL((dir>0)? -pwm : +pwm);
    motorR((dir>0)? +pwm : -pwm);

    buzzUpdate();  // let buzzer auto-stop while aligning

    delay(2);
  }
  motorsStop();
  delay(60);
}

// ======================================================
// ============== Optional: one-cell helper =============
// (uses posA/posB; sign-agnostic; NOT called by loop)
// ======================================================
float TICKS_PER_MM_L = 13.56f;   // set after calibration
float TICKS_PER_MM_R = 13.45f;   // set after calibration
int   CELL_MM        = 180;

int ONE_CELL_TICKS_L(){ return (int)roundf(TICKS_PER_MM_L * CELL_MM); }
int ONE_CELL_TICKS_R(){ return (int)roundf(TICKS_PER_MM_R * CELL_MM); }

bool driveOneCell() {
  const int   BASE_PWM   = 90;
  const float BAL_GAIN   = 0.5f;
  const int   MIN_PWM    = 70, MAX_PWM = 110;

  int startA = posA, startB = posB;

  while ((abs(posA - startA) < ONE_CELL_TICKS_L()) &&
         (abs(posB - startB) < ONE_CELL_TICKS_R())) {

    // --- encoder progress ---
    int   dA = abs(posA - startA), dB = abs(posB - startB);
    float mmL = dA / TICKS_PER_MM_L;
    float mmR = dB / TICKS_PER_MM_R;
    float mmProg = fminf(mmL, mmR);

    // --- check front sensor ---
    float F = readMM(SID_FRONT, CH_FRONT);

    if (F >= 0 && F <= EMERGENCY_MM) {
      motorsStop();
      Serial.println("[driveOneCell] EMERGENCY stop.");
      return false;
    }

    if (mmProg > COMMIT_MM && F >= 0 && F < FRONT_STOP_MM) {
      motorsStop();
      Serial.println("[driveOneCell] Early stop: wall ahead.");
      return false;
    }

    // --- side centering ---
    float L = readMM(SID_LEFT, CH_LEFT);
    float R = readMM(SID_RIGHT, CH_RIGHT);

    float err = 0.0f;
    if (L >= 0 && R >= 0) err = L - R;
    else if (L >= 0)      err = L - TARGET_SIDE_MM;
    else if (R >= 0)      err = -(R - TARGET_SIDE_MM);

    // --- taper near end ---
    int remainL = ONE_CELL_TICKS_L() - dA;
    int remainR = ONE_CELL_TICKS_R() - dB;
    float remainMM = fminf(remainL / TICKS_PER_MM_L, remainR / TICKS_PER_MM_R);

    int base = BASE_PWM;
    if (remainMM < 40.0f) {
      float scale = fmaxf(remainMM / 40.0f, 0.4f);
      base = (int)fmaxf(BASE_PWM * scale, (float)MIN_PWM);
    }

    // --- slow down if wall is near ---
    if (F >= 0 && F < FRONT_SLOW_MM) {
      base = (int)fmaxf((float)base * 0.7f, (float)MIN_PWM);
    }

    // --- tick balancing ---
    int tickErr = (posA - startA) - (posB - startB);
    base = (int)roundf(base - BAL_GAIN * tickErr);
    base = constrain(base, MIN_PWM, MAX_PWM);

    driveCentered(base, err);
    buzzUpdate();
    delay(10);
  }

  motorsStop();

  // ✅ verify travel before success
  int dA = abs(posA - startA);
  int dB = abs(posB - startB);
  float fracA = (float)dA / ONE_CELL_TICKS_L();
  float fracB = (float)dB / ONE_CELL_TICKS_R();

  if (fracA < 0.8f || fracB < 0.8f) {
    Serial.printf("[driveOneCell] Incomplete (%.2f, %.2f). FAIL.\n", fracA, fracB);
    return false;
  }

  Serial.printf("[driveOneCell] Done OK: dA=%d dB=%d\n", dA, dB);
  return true;
}


// ======================================================
// ====================  Setup  =========================
// ======================================================
void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // motors & buzzer
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT,OUTPUT);
  pinMode(IN2_RIGHT,OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  motorsStop();

  // ToF sensors
  tofBegin(CH_LEFT);
  tofBegin(CH_ALEFT45);
  tofBegin(CH_FRONT);
  tofBegin(CH_ARIGHT45);
  tofBegin(CH_RIGHT);

  // IMU (ensure robot is still)
  imuBegin();

  // Encoders: set INPUT (or INPUT_PULLUP if your encoders are open-collector)
  pinMode(ENCA1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCB2, INPUT);
  // attach both lines for each wheel to the same ISR (CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

  // Initialize maze mapping
  initMaze();
  
  // Set starting position (typically bottom-left corner)
  robotX = 0;
  robotY = 0;
  robotHeading = 0; // Facing North initially
  stepCount = 0;
  
  // Scan initial position
  delay(1000); // Let sensors stabilize
  scanWalls();
  printMaze();

  // little boot chirp
  buzzerBegin();
  buzzOK();   // quick chirp at boot (non-blocking)

  Serial.println("Zeeker: centered-drive + square-up + IMU turns + QUAD encoders + buzzer + WALL MAPPING.");
}

// ======================================================
// ====================  Main loop  =====================
// cell-to-cell navigation using left-hand rule with mapping
// ======================================================
void loop() {
  buzzUpdate();

  static bool busy = false;
  if (busy) return;
  busy = true;

  Serial.println("\n[Cell-to-cell] Starting new step...");
  Serial.printf("[DEBUG] At (%d,%d) facing %s\n", robotX, robotY, dirNames[robotHeading]);

  scanWalls();

  float F = readMM(SID_FRONT, CH_FRONT);

  if (F >= 0 && F < FRONT_STOP_MM) {
    motorsStop();
    buzzAttention();
    squareUp();
    delay(100);

    float L = readMM(SID_LEFT, CH_LEFT);
    float R = readMM(SID_RIGHT, CH_RIGHT);

    bool leftOpen  = (L < 0);
    bool frontOpen = (F < 0);
    bool rightOpen = (R < 0);

    int turnDeg;
    if (leftOpen)        turnDeg = +90;
    else if (frontOpen)  turnDeg = 0;
    else if (rightOpen)  turnDeg = -90;
    else                 turnDeg = 180;

    Serial.printf("[Decision] L=%.1f F=%.1f R=%.1f -> %d\n", L, F, R, turnDeg);

    if (turnDeg ==  90) { turnIMU(+90, 80, 1600); updateHeading(+90); }
    if (turnDeg == -90) { turnIMU(-90, 80, 1600); updateHeading(-90); }
    if (turnDeg == 180) {
      turnIMU(180, 85, 1800);
      updateHeading(180);
      Serial.println("[DEBUG] Dead-end: backtracking one cell");
      if (driveOneCell()) updatePosition();
      printMaze();
      busy = false;
      return;
    }
  }

  // forward one cell
  if (driveOneCell()) {
    updatePosition();
    buzzOK();
  } else {
    Serial.println("[WARN] Forward move failed.");
  }

  printMaze();
  busy = false;
}



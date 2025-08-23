// #include <Wire.h>
// #include "Adafruit_VL6180X.h"
// #include <MPU9250_asukiaaa.h>
// #include <math.h>

// #define IMU_Z_SIGN  -1   // <-- change to +1 if IMU is mounted differently
// void motorsStop();   // forward declaration
// void backtrackToStart();  // forward declaration

// // top of file (globals)
// static unsigned long lastStuckBeep = 0;

// #define EARLY_WALL_TOL_MM  40   // acceptable if we stop this close to 1-cell


// // ========== Motor pins ==========
// #define PWM_LEFT    4
// #define PWM_RIGHT   5
// #define IN1_LEFT    15
// #define IN2_LEFT    21
// #define IN1_RIGHT   22
// #define IN2_RIGHT   23

// // ========== Encoders (quadrature, A & B) ==========
// #define ENCA1 16  // Left encoder A
// #define ENCA2 17  // Left encoder B
// #define ENCB1 18  // Right encoder A
// #define ENCB2 19  // Right encoder B

// volatile int posA = 0;           // left ticks
// volatile int posB = 0;           // right ticks
// volatile int lastEncodedA = 0;   // last AB snapshot (left)
// volatile int lastEncodedB = 0;   // last AB snapshot (right)

// void IRAM_ATTR updateEncoderA() {
//   int msb = digitalRead(ENCA1);
//   int lsb = digitalRead(ENCA2);
//   int encoded = (msb << 1) | lsb;
//   int sum = (lastEncodedA << 2) | encoded;

//   // Gray code decode
//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posA++;
//   else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posA--;

//   lastEncodedA = encoded;
// }
// void IRAM_ATTR updateEncoderB() {
//   int msb = digitalRead(ENCB1);
//   int lsb = digitalRead(ENCB2);
//   int encoded = (msb << 1) | lsb;
//   int sum = (lastEncodedB << 2) | encoded;

//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) posB--;
//   else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) posB++;

//   lastEncodedB = encoded;
// }

// // ----- Buzzer (non-blocking, dedicated LEDC channel) -----
// #define BUZZER_PIN 2
// #define BUZZ_CH    7      // use a channel not used by your motor PWMs
// bool BUZZ_ENABLED = true;

// static unsigned long buzz_end_ms = 0;

// void buzzerBegin() {
//   pinMode(BUZZER_PIN, OUTPUT);
//   ledcSetup(BUZZ_CH, 2000 /*Hz*/, 8 /*bits*/);
//   ledcAttachPin(BUZZER_PIN, BUZZ_CH);
//   ledcWriteTone(BUZZ_CH, 0);     // ensure off
// }

// // start a single beep without blocking
// void buzzStart(int freqHz, int durMs) {
//   if (!BUZZ_ENABLED) return;
//   ledcWriteTone(BUZZ_CH, freqHz);
//   buzz_end_ms = millis() + durMs;
// }

// // call this every loop() to auto-stop tone
// void buzzUpdate() {
//   if (buzz_end_ms != 0 && (long)(millis() - buzz_end_ms) >= 0) {
//     ledcWriteTone(BUZZ_CH, 0);
//     buzz_end_ms = 0;
//   }
// }

// // optional helpers (short/ok and double-beep patterns, non-blocking)
// void buzzOK()        { buzzStart(1500, 80); }     // quick chirp
// void buzzAttention() { buzzStart(300, 50); }    // slightly 

// // ========== I2C / MUX ==========
// #define SDA_PIN     25
// #define SCL_PIN     26
// #define MUX_ADDR    0x70

// // MUX channels (adjust to your wiring)
// #define CH_LEFT      0
// #define CH_ALEFT45   1
// #define CH_FRONT     2
// #define CH_ARIGHT45  3
// #define CH_RIGHT     4

// Adafruit_VL6180X tof;
// MPU9250_asukiaaa mpu;

// // ========== Calibration: mm = a*raw + b ==========
// struct Cal { float a, b; };
// enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };

// Cal CAL[5] = {
//   {1.00f, -42.0f},  // LEFT
//   {1.00f,  20.0f},  // ALEFT45
//   {1.00f, -35.0f},  // FRONT
//   {1.00f, -10.0f},  // ARIGHT45
//   {1.00f, -28.0f}   // RIGHT
// };

// // ========== Driving params ==========
// #define NO_WALL_MM        100   // > this => treat as "no wall"
// #define TARGET_SIDE_MM    50
// #define FRONT_BLOCK_MM    32    // used for junction decisions (debounced)
// #define FRONT_CLEAR_MM    45
// #define FRONT_SLOW_MM     80
// #define FRONT_STOP_MM     40    // normal stop threshold once committed
// #define EMERGENCY_MM      28    // ALWAYS stop if at/under this, any time

// // --- thresholds for square-up centering ---
// #define FRONT_CENTER_MM    32    // true "cell center" stop
// #define GAP_TOL_MM         3     // tolerance when nudging

// // Commit distance into a new cell before we use FRONT_STOP_MM
// // (kept small so we don't tunnel into a short cell)
// #define COMMIT_MM         20

// // ========== Square-up params ==========
// #define A45_TOL_MM        6
// #define SQUARE_TURN_PWM   90
// #define SQUARE_MAX_MS     3000
// #define GAP_TOL_MM        5
// #define NUDGE_PWM         85

// // ========== WALL MAPPING ==========
// #define MAZE_SIZE 6  // Keep your original 4x4 size

// // Wall bits for each cell
// #define WALL_NORTH  0x01
// #define WALL_EAST   0x02
// #define WALL_SOUTH  0x04
// #define WALL_WEST   0x08
// #define VISITED     0x10

// // ========== FLOOD FILL ==========
// #define FLOOD_MAX 255


// // Global variable to hold the path as a string
// String fullPath = "";

// // Maze storage
// uint8_t maze[MAZE_SIZE][MAZE_SIZE];
// uint8_t floodMap[MAZE_SIZE][MAZE_SIZE];

// // Robot position and orientation
// int robotX = 0, robotY = 0;
// int robotHeading = 0; // 0=North, 1=East, 2=South, 3=West
// int stepCount = 0;

// // Algorithm state
// enum AlgoState { EXPLORING, RETURNING, SPEED_RUN };
// AlgoState currentState = EXPLORING;
// bool targetReached = false;

// // Direction vectors (FIXED: North should increase Y, East should increase X)
// int dx[] = {0, 1, 0, -1}; // North, East, South, West  
// int dy[] = {1, 0, -1, 0}; // North=+Y, East=+X, South=-Y, West=-X
// const char* dirNames[] = {"North", "East", "South", "West"};

// // ========== IMU state ==========
// static float gyroBiasZ = 0.0f;

// // ======================================================
// // ================  MUX + ToF helpers  =================
// // ======================================================
// void mux(uint8_t ch){
//   Wire.beginTransmission(MUX_ADDR);
//   Wire.write(1 << ch);
//   Wire.endTransmission();
//   delayMicroseconds(200);
// }
// bool tofBegin(uint8_t ch){ mux(ch); delay(3); return tof.begin(); }

// float readMM(SensorId sid, uint8_t ch){
//   mux(ch); delay(2);
//   uint8_t r = tof.readRange();
//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
//   float mm = CAL[sid].a * r + CAL[sid].b;
//   if (mm < 0) mm = 0;
//   if (mm > NO_WALL_MM) return -1.0f;
//   return mm;
// }

// // Initialize maze (add boundary walls)
// void initMaze() {
//   // Clear all walls
//   for (int x = 0; x < MAZE_SIZE; x++) {
//     for (int y = 0; y < MAZE_SIZE; y++) {
//       maze[x][y] = 0;
//       floodMap[x][y] = FLOOD_MAX;
//     }
//   }
  
//   // Add boundary walls
//   for (int i = 0; i < MAZE_SIZE; i++) {
//     maze[i][0] |= WALL_SOUTH;           // South boundary
//     maze[i][MAZE_SIZE-1] |= WALL_NORTH; // North boundary
//     maze[0][i] |= WALL_WEST;            // West boundary
//     maze[MAZE_SIZE-1][i] |= WALL_EAST;  // East boundary
//   }
  
//   Serial.println("Maze initialized with boundary walls");
// }

// // ========== FLOOD FILL FUNCTIONS ==========
// // Automatically compute center cells for any even MAZE_SIZE
// #define CENTER1 (MAZE_SIZE/2 - 1)
// #define CENTER2 (MAZE_SIZE/2)

// bool isTarget(int x, int y) {
//   return (x == CENTER1 && y == CENTER1) ||
//          (x == CENTER1 && y == CENTER2) ||
//          (x == CENTER2 && y == CENTER1) ||
//          (x == CENTER2 && y == CENTER2);
// }


// bool canMove(int fromX, int fromY, int dir) {
//   if (fromX < 0 || fromX >= MAZE_SIZE || fromY < 0 || fromY >= MAZE_SIZE) {
//     return false;
//   }
  
//   uint8_t wallBit;
//   switch (dir) {
//     case 0: wallBit = WALL_NORTH; break;
//     case 1: wallBit = WALL_EAST; break;
//     case 2: wallBit = WALL_SOUTH; break;
//     case 3: wallBit = WALL_WEST; break;
//     default: return false;
//   }
  
//   return !(maze[fromX][fromY] & wallBit);
// }

// void floodFill() {
//   // Reset flood map
//   for (int x = 0; x < MAZE_SIZE; x++) {
//     for (int y = 0; y < MAZE_SIZE; y++) {
//       floodMap[x][y] = FLOOD_MAX;
//     }
//   }
  
//   // Simple queue for BFS
//   struct Point { int x, y; };
//   Point queue[MAZE_SIZE * MAZE_SIZE];
//   int front = 0, rear = 0;
  
//   // Add target cells with distance 0
//   if (currentState == RETURNING) {
//     floodMap[0][0] = 0;
//     queue[rear++] = {0, 0};
// } else {
//   // Seed all 4 center cells
//   floodMap[CENTER1][CENTER1] = 0; queue[rear++] = {CENTER1, CENTER1};
//   floodMap[CENTER1][CENTER2] = 0; queue[rear++] = {CENTER1, CENTER2};
//   floodMap[CENTER2][CENTER1] = 0; queue[rear++] = {CENTER2, CENTER1};
//   floodMap[CENTER2][CENTER2] = 0; queue[rear++] = {CENTER2, CENTER2};
// }

  
//   // BFS
//   while (front < rear) {
//     Point current = queue[front++];
//     int currentDist = floodMap[current.x][current.y];
    
//     for (int dir = 0; dir < 4; dir++) {
//       int newX = current.x + dx[dir];
//       int newY = current.y + dy[dir];
      
//       if (newX < 0 || newX >= MAZE_SIZE || newY < 0 || newY >= MAZE_SIZE) continue;
//       if (!canMove(current.x, current.y, dir)) continue;
//       if (floodMap[newX][newY] > currentDist + 1) {
//         floodMap[newX][newY] = currentDist + 1;
//         queue[rear++] = {newX, newY};
//       }
//     }
//   }
// }

// // ========== NEW: DEDICATED FLOOD FILL DECISION FUNCTION ==========
// int makeFloodFillDecision() {
//   Serial.println("\n[FLOOD FILL] Making navigation decision...");
  
//   // Run flood fill algorithm
//   floodFill();
  
//   // Find the best direction (lowest flood value)
//   int bestDirection = -1;
//   int lowestValue = FLOOD_MAX + 1;
  
//   Serial.printf("[FLOOD FILL] Current position (%d,%d) has flood value: %d\n", 
//                 robotX, robotY, floodMap[robotX][robotY]);
  
//   // Check all 4 directions
//   for (int dir = 0; dir < 4; dir++) {
//     int newX = robotX + dx[dir];
//     int newY = robotY + dy[dir];
    
//     // Check bounds
//     if (newX < 0 || newX >= MAZE_SIZE || newY < 0 || newY >= MAZE_SIZE) {
//       Serial.printf("[FLOOD FILL] Direction %s: OUT OF BOUNDS\n", dirNames[dir]);
//       continue;
//     }
    
//     // Check for walls
//     if (!canMove(robotX, robotY, dir)) {
//       Serial.printf("[FLOOD FILL] Direction %s: BLOCKED BY WALL\n", dirNames[dir]);
//       continue;
//     }
    
//     int floodValue = floodMap[newX][newY];
//     Serial.printf("[FLOOD FILL] Direction %s -> Cell (%d,%d) has flood value: %d\n", 
//                   dirNames[dir], newX, newY, floodValue);
    
//     if (floodValue < lowestValue) {
//       lowestValue = floodValue;
//       bestDirection = dir;
//     }
//   }
  
//   if (bestDirection == -1) {
//     Serial.println("[FLOOD FILL ERROR] No valid direction found!");
//     return -1;
//   }
  
//   Serial.printf("[FLOOD FILL] Best direction: %s (flood value: %d)\n", 
//                 dirNames[bestDirection], lowestValue);
  
//   return bestDirection;
// }

// // Calculate turn needed to face a specific direction
// int calculateTurn(int targetDirection) {
//   if (targetDirection == -1) return 0;

//   int dirDiff = (targetDirection - robotHeading + 4) % 4; 
//   // normalize to [0..3]

//   switch (dirDiff) {
//     case 0: return 0;     // already facing target
//     case 1: return -90;   // need to turn right
//     case 2: return 180;   // need U-turn
//     case 3: return 90;    // need to turn left
//   }
//   return 0;
// }


// // Add wall to maze
// void addWall(int x, int y, uint8_t wallBit) {
//   if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE) {
//     maze[x][y] |= wallBit;
    
//     // Also add corresponding wall to adjacent cell
//     int adjX = x, adjY = y;
//     uint8_t adjWallBit = 0;
    
//     switch (wallBit) {
//       case WALL_NORTH:
//         adjY++; adjWallBit = WALL_SOUTH;
//         break;
//       case WALL_EAST:
//         adjX++; adjWallBit = WALL_WEST;
//         break;
//       case WALL_SOUTH:
//         adjY--; adjWallBit = WALL_NORTH;
//         break;
//       case WALL_WEST:
//         adjX--; adjWallBit = WALL_EAST;
//         break;
//     }
    
//     if (adjX >= 0 && adjX < MAZE_SIZE && adjY >= 0 && adjY < MAZE_SIZE) {
//       maze[adjX][adjY] |= adjWallBit;
//     }
//   }
// }

// // ================= Wall detection helper =================
// bool isWall(float mm, float oppMm) {
//   if (mm < 0) return false;           // invalid reading
  
//   if (mm < 70) return true;           // close wall
  
//   if (mm > 80 && mm < 110) {          // mid (~100mm) reading
//     if (oppMm > 0 && oppMm < 70) {    // opposite side is close
//       return true;
//     }
//   }
  
//   return false;
// }


// // Scan and record walls at current position
// void scanWalls() {
//   Serial.printf("\n[SCAN] Position (%d,%d) facing %s\n", robotX, robotY, dirNames[robotHeading]);
  
//   // Mark current cell as visited
//   maze[robotX][robotY] |= VISITED;
  
//   // Read sensors
//   float L = readMM(SID_LEFT,  CH_LEFT);
//   float F = readMM(SID_FRONT, CH_FRONT);
//   float R = readMM(SID_RIGHT, CH_RIGHT);
  
//   Serial.printf("[SCAN] Sensor readings: L=%.1fmm F=%.1fmm R=%.1fmm\n", L, F, R);
  
//   // Determine wall directions relative to robot's current heading
//   int leftDir = (robotHeading + 3) % 4;   // Turn left
//   int frontDir = robotHeading;            // Straight
//   int rightDir = (robotHeading + 1) % 4;  // Turn right
  
//   // Convert directions to wall bits
//   uint8_t wallBits[] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
  
//   // Check and record walls
// if (isWall(L, R)) {
//   addWall(robotX, robotY, wallBits[leftDir]);
//   Serial.printf("[SCAN] Wall detected on LEFT (%s)\n", dirNames[leftDir]);
// }

// if (isWall(F, -1)) {   // front has no “opposite”
//   addWall(robotX, robotY, wallBits[frontDir]);
//   Serial.printf("[SCAN] Wall detected on FRONT (%s)\n", dirNames[frontDir]);
// }

// if (isWall(R, L)) {
//   addWall(robotX, robotY, wallBits[rightDir]);
//   Serial.printf("[SCAN] Wall detected on RIGHT (%s)\n", dirNames[rightDir]);
// }

// }

// // Update robot position after moving forward
// void updatePosition() {
//   int oldX = robotX, oldY = robotY;

//   robotX += dx[robotHeading];
//   robotY += dy[robotHeading];

//   // ✅ if didn't actually move, skip
//   if (robotX == oldX && robotY == oldY) {
//     Serial.println("[WARN] updatePosition called but no movement. Skipping.");
//     return;
//   }

//   stepCount++;

//   // clamp to maze bounds
//   if (robotX < 0) robotX = 0;
//   if (robotX >= MAZE_SIZE) robotX = MAZE_SIZE - 1;
//   if (robotY < 0) robotY = 0;
//   if (robotY >= MAZE_SIZE) robotY = MAZE_SIZE - 1;

//   // append step to path string
//   char stepChar;
//   if (robotX > oldX) stepChar = 'E';
//   else if (robotX < oldX) stepChar = 'W';
//   else if (robotY > oldY) stepChar = 'N';
//   else if (robotY < oldY) stepChar = 'S';
//   else stepChar = '?';

//   fullPath += stepChar;
//   fullPath += ' ';

//   Serial.printf("[MOVE] Step %d: (%d,%d) -> (%d,%d) [%c]\n",
//                 stepCount, oldX, oldY, robotX, robotY, stepChar);

//   Serial.printf("[PATH] %s\n", fullPath.c_str());
  
//   // Check if target reached
// if (!targetReached && isTarget(robotX, robotY)) {
//   targetReached = true;
//   Serial.println("*** TARGET REACHED! STARTING BACKTRACK ***");
//   buzzStart(2000, 500);
//   motorsStop();

//   delay(1000);  // small pause
//   backtrackToStart();   // ✅ go back to (0,0)
// }


  
//   // Check if back at start
//   if (currentState == RETURNING && robotX == 0 && robotY == 0 && targetReached) {
//     currentState = SPEED_RUN;
//     Serial.println("[STATE] Ready for SPEED RUN!");
//     buzzStart(1000, 200);
//     delay(300);
//     buzzStart(1000, 200);
//   }
// }

// // Update robot heading after turning
// void updateHeading(int turnDegrees) {
//   if (turnDegrees == 90) {        // Left turn
//     robotHeading = (robotHeading + 3) % 4;
//   } else if (turnDegrees == -90) { // Right turn
//     robotHeading = (robotHeading + 1) % 4;
//   } else if (turnDegrees == 180) { // U-turn
//     robotHeading = (robotHeading + 2) % 4;
//   }
  
//   Serial.printf("[TURN] Turned %d degrees, now facing %s\n", 
//                 turnDegrees, dirNames[robotHeading]);
// }

// // Print current maze map
// void printMaze() {
//   Serial.println("\n========== CURRENT MAZE MAP ==========");
//   Serial.printf("Robot at (%d,%d) facing %s | Steps: %d\n", 
//                 robotX, robotY, dirNames[robotHeading], stepCount);
//   Serial.printf("Algorithm: FLOOD FILL | State: %s\n",
//                 (currentState == EXPLORING) ? "EXPLORING" : 
//                 (currentState == RETURNING) ? "RETURNING" : "SPEED_RUN");
//   Serial.println("Legend: R=Robot, *=Visited, .=Unknown, T=Target");
  
//   // Print top border
//   Serial.print("  ");
//   for (int x = 0; x < MAZE_SIZE; x++) {
//     Serial.printf("%2d", x);
//   }
//   Serial.println();
  
//   // Print maze from top to bottom (North to South)
//   for (int y = MAZE_SIZE - 1; y >= 0; y--) {
//     Serial.printf("%2d", y);
    
//     for (int x = 0; x < MAZE_SIZE; x++) {
//       if (x == robotX && y == robotY) {
//         Serial.print(" R"); // Robot position
//       } else if (isTarget(x, y)) {
//         Serial.print(" T"); // Target
//       } else if (maze[x][y] & VISITED) {
//         Serial.print(" *"); // Visited cell
//       } else {
//         Serial.print(" ."); // Unknown cell
//       }
//     }
//     Serial.println();
//   }
  
//   // Print flood fill values
//   Serial.println("\n--- FLOOD FILL VALUES ---");
//   for (int y = MAZE_SIZE - 1; y >= 0; y--) {
//     for (int x = 0; x < MAZE_SIZE; x++) {
//       if (floodMap[x][y] == FLOOD_MAX) {
//         Serial.print("### ");
//       } else {
//         Serial.printf("%3d ", floodMap[x][y]);
//       }
//     }
//     Serial.println();
//   }
  
//   // Print detailed wall information for visited cells
//   Serial.println("\n--- DETAILED WALL MAP ---");
//   for (int y = MAZE_SIZE - 1; y >= 0; y--) {
//     for (int x = 0; x < MAZE_SIZE; x++) {
//       if (maze[x][y] & VISITED) {
//         Serial.printf("Cell (%d,%d): ", x, y);
//         if (maze[x][y] & WALL_NORTH) Serial.print("N ");
//         if (maze[x][y] & WALL_EAST)  Serial.print("E ");
//         if (maze[x][y] & WALL_SOUTH) Serial.print("S ");
//         if (maze[x][y] & WALL_WEST)  Serial.print("W ");
//         Serial.println();
//       }
//     }
//   }
//   Serial.println("=====================================\n");
// }

// // ======================================================
// /**                     Motors                          */
// // ======================================================
// void motorL(int pwm){
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   digitalWrite(IN1_LEFT,  fwd ? HIGH : LOW);
//   digitalWrite(IN2_LEFT,  fwd ? LOW  : HIGH);
//   analogWrite(PWM_LEFT, abs(pwm));
// }
// void motorR(int pwm){
//   pwm = constrain(pwm, -255, 255);
//   bool fwd = (pwm >= 0);
//   digitalWrite(IN1_RIGHT, fwd ? HIGH : LOW);
//   digitalWrite(IN2_RIGHT, fwd ? LOW  : HIGH);
//   analogWrite(PWM_RIGHT, abs(pwm));
// }
// void motorsStop(){ motorL(0); motorR(0); }

// // ======================================================
// // ============  Wall-follow (your PD)  =================
// // ======================================================
// // ---- GLOBAL STATE ----
// float desiredHeading = 0;  // updated whenever you start a new cell move
// bool hasLeftWall = false, hasRightWall = false;

// // === Centered drive with wall/IMU fallback ===
// // forward declarations
// float imuGetYaw();
// float angleDiff(float target, float current);
// void driveCentered(int basePWM, float err, bool resetPID = false);

// // === Centered drive with wall/IMU fallback ===
// void driveOneCell(int basePWM) {
//   // read sensors
//   float distL = readMM(SID_LEFT, CH_LEFT);
//   float distR = readMM(SID_RIGHT, CH_RIGHT);
//   hasLeftWall  = (distL > 0 && distL < NO_WALL_MM);
//   hasRightWall = (distR > 0 && distR < NO_WALL_MM);

//   float err = 0;

//   if (hasLeftWall && hasRightWall) {
//     err = (distL - distR);
//   }
//   else if (hasLeftWall) {
//     const float targetDistL = 80;
//     err = (targetDistL - distL);
//   }
//   else if (hasRightWall) {
//     const float targetDistR = 80;
//     err = (distR - targetDistR);
//   }
//   else {
//     // 🚨 No walls → IMU heading lock
//     float currentHeading = imuGetYaw();
//     float headingErr = angleDiff(desiredHeading, currentHeading);
//     err = headingErr * 2.0f;
//   }

// driveCentered(basePWM, err, false);
// }


// // === Low-level PID drive with optional reset parameter ===
// void driveCentered(int basePWM, float err, bool resetPID ) {
//   static float err_prev = 0, err_sum = 0;
//   static unsigned long last = millis();
  
//   // Reset PID state if requested
//   if (resetPID) {
//     err_prev = 0;
//     err_sum = 0;
//     last = millis();
//     Serial.println("[PID] State reset");
//   }
  
//   unsigned long now = millis();
//   float dt = (now - last) / 1000.0f; 
//   if (dt <= 0) dt = 0.001f;
//   last = now;

//   const float Kp = 0.8f, Ki = 0.0f, Kd = 0.1f;
//   err_sum += err * dt;
//   float derr = (err - err_prev) / dt;
//   float turn = Kp*err + Ki*err_sum + Kd*derr;
//   err_prev = err;

//   int l = constrain((int)(basePWM + turn), 0, 255);
//   int r = constrain((int)(basePWM - turn), 0, 255);
//   motorL(l); 
//   motorR(r);
// }

// // === Helper for heading difference ===
// float angleDiff(float target, float current) {
//   float d = target - current;
//   while (d > 180) d -= 360;
//   while (d < -180) d += 360;
//   return d;
// }


// // ======================================================
// // ================  Square-up routine  =================
// // ======================================================
// void squareUp() {
//   Serial.println(">>> Square-up START");
//   unsigned long t0 = millis();

//   while (millis() - t0 < SQUARE_MAX_MS) {
//     // --- Read sensors ---
//     float L   = readMM(SID_LEFT,    CH_LEFT);
//     float R   = readMM(SID_RIGHT,   CH_RIGHT);
//     float F   = readMM(SID_FRONT,   CH_FRONT);
//     float AL  = readMM(SID_ALEFT45, CH_ALEFT45);
//     float AR  = readMM(SID_ARIGHT45,CH_ARIGHT45);

//     Serial.print("[SQUARE] L="); Serial.print(L);
//     Serial.print(" R="); Serial.print(R);
//     Serial.print(" F="); Serial.print(F);
//     Serial.print(" AL="); Serial.print(AL);
//     Serial.print(" AR="); Serial.println(AR);

//     // --- Step 1: Orientation (heading correction) ---
//     float headingErr = 0.0f;
//     if (AL > 0 && AR > 0) {
//       headingErr = AR - AL; // prefer 45° sensors
//     } else if (L > 0 && R > 0) {
//       headingErr = (R - L); // fallback: side sensors
//     }

//     if (fabs(headingErr) > A45_TOL_MM) {
//       int pwm = constrain((int)(SQUARE_TURN_PWM * fabs(headingErr) / 30.0f),
//                           35, SQUARE_TURN_PWM);
//       if (headingErr > 0) { motorL(-pwm); motorR(pwm); }
//       else                { motorL(pwm);  motorR(-pwm); }
//       delay(50); motorsStop();
//       Serial.print("[SQUARE] Corrected headingErr="); 
//       Serial.println(headingErr);
//     }

//     // --- Step 2: Distance to front wall (center at ~32 mm) ---
//     if (F > 0) {
//       float frontErr = F - FRONT_CENTER_MM; // target is 32 mm, not 40
//       if (fabs(frontErr) > GAP_TOL_MM) {
//         int dir = (frontErr > 0) ? +1 : -1;
//         int pwm = constrain((int)(NUDGE_PWM * fabs(frontErr) / 25.0f),
//                             35, NUDGE_PWM);
//         motorL(dir * pwm); motorR(dir * pwm);
//         delay(80); motorsStop();
//         Serial.print("[SQUARE] Nudged front by "); 
//         Serial.print(frontErr); Serial.println(" mm");
//       }
//     }

//     // --- Step 3: Centering between side walls ---
//     if (L > 0 && R > 0) {
//       float centerErr = L - R;
//       if (fabs(centerErr) > GAP_TOL_MM) {
//         int dir = (centerErr > 0) ? -1 : +1; // move toward further wall
//         int pwm = constrain((int)(NUDGE_PWM * fabs(centerErr) / 25.0f),
//                             35, NUDGE_PWM);
//         motorL(dir * pwm); motorR(dir * pwm);
//         delay(80); motorsStop();
//         Serial.print("[SQUARE] Adjusted side balance by ");
//         Serial.print(centerErr); Serial.println(" mm");
//       }
//     }

//     // --- Check exit conditions ---
//     bool headingOK = fabs(headingErr) < A45_TOL_MM;
//     bool frontOK   = (F <= 0) || fabs(F - FRONT_CENTER_MM) < GAP_TOL_MM;
//     bool centerOK  = !(L > 0 && R > 0 && fabs(L - R) > GAP_TOL_MM);

//     if (headingOK && frontOK && centerOK) break;

//     buzzUpdate();
//     delay(40); // pause before next adjustment
//   }

//   motorsStop();
//   Serial.println(">>> Square-up DONE");
// }





// // ======================================================
// // ====================  IMU turns  =====================
// // ======================================================
// int GZ_SIGN = +1; // CCW positive; will auto-set in imuBegin()

// void imuBegin(){
//   mpu.setWire(&Wire);
//   mpu.beginGyro();

//   // bias (keep robot still!)
//   unsigned long t0 = millis();
//   double sum = 0; 
//   long n = 0;
//   while (millis() - t0 < 1200){
//     mpu.gyroUpdate();
//     sum += mpu.gyroZ();
//     n++; 
//     delay(2);
//   }
//   gyroBiasZ = (n > 0) ? (float)(sum / n) : 0.0f;
//   Serial.printf("IMU bias Z = %.3f deg/s\n", gyroBiasZ);

//   // fixed sign (no twitch, manual setting)
//   GZ_SIGN = +1;  // +1 = CCW positive, -1 = CW positive
//   Serial.printf("IMU Z sign forced = %d (expect +1 for CCW)\n", GZ_SIGN);
// }



// // +angleDeg = left/CCW; -angleDeg = right/CW
// void turnIMU(float angleDeg, int basePWM=90, int timeout_ms=1800) {
//   if (angleDeg == 0) return;

//   const float slowBandDeg = 25.0f;   // start tapering speed inside this band
//   const float finishBand  = 3.0f;    // consider done when remaining < this
//   const float minSpinGz   = 8.0f;    // deg/s; if below for too long, bump PWM
//   const int   minPWM      = 60;
//   const int   maxPWM      = 140;

//   int dir = (angleDeg > 0) ? +1 : -1;         // +1=CCW, -1=CW
//   float target = fabsf(angleDeg);

//   float yaw_prog = 0.0f;                      // progress in commanded sense
//   unsigned long last = millis();
//   unsigned long t0   = last;
//   unsigned long lowSpinSince = last;

//   while (true) {
//     if ((int)(millis() - t0) > timeout_ms) {
//       Serial.println("[TURN] Timeout!");
//       break;
//     }

//     mpu.gyroUpdate();
//     float gz = (mpu.gyroZ() - gyroBiasZ) * IMU_Z_SIGN;
//     unsigned long now = millis();
//     float dt = (now - last) / 1000.0f; if (dt <= 0) dt = 0.001f;
//     last = now;

//     // accumulate progress in the *commanded* direction
//     yaw_prog += (dir * gz) * dt;                // positive when moving toward target
//     float remaining = target - yaw_prog;

//     // finish condition (close enough and not spinning fast)
//     if (remaining <= finishBand && fabsf(gz) < 25.0f) break;
//     if (remaining <= 0) break;

//     // adaptive speed profile
//     float scale = (remaining < slowBandDeg) ? fmaxf(remaining/slowBandDeg, 0.22f) : 1.0f;
//     int pwm = (int)fmaxf(fminf(basePWM * scale, (float)maxPWM), (float)minPWM);

//     // help if we’re barely rotating (stiction / floor friction)
//     if (fabsf(gz) < minSpinGz) {
//       if ((int)(now - lowSpinSince) > 120) {
//         pwm = fmin(pwm + 15, maxPWM);
//         lowSpinSince = now;
//       }
//     } else {
//       lowSpinSince = now;
//     }

//     motorL((dir > 0) ? -pwm : +pwm);
//     motorR((dir > 0) ? +pwm : -pwm);

//     // OPTIONAL: debug every ~50ms
//     // static unsigned long dbg=0; if(now-dbg>50){ dbg=now;
//     //   Serial.printf("[TURN] gz=%.1f yaw=%.1f rem=%.1f pwm=%d\n", gz, yaw_prog, remaining, pwm);
//     // }

//     buzzUpdate();
//     delay(2);
//   }

//   motorsStop();

//   // micro-brake to kill residual momentum
//   int bp = 40;
//   motorL((dir > 0) ? +bp : -bp);
//   motorR((dir > 0) ? -bp : +bp);
//   delay(30);
//   motorsStop();
//   delay(60);
// }




// // ======================================================
// // ============== Optional: one-cell helper =============
// // (uses posA/posB; sign-agnostic; NOT called by loop)
// // ======================================================
// float TICKS_PER_MM_L = 13.56f;   // set after calibration
// float TICKS_PER_MM_R = 13.45f;   // set after calibration
// int   CELL_MM        = 192;

// int ONE_CELL_TICKS_L(){ return (int)roundf(TICKS_PER_MM_L * CELL_MM); }
// int ONE_CELL_TICKS_R(){ return (int)roundf(TICKS_PER_MM_R * CELL_MM); }

// // ======================================================
// // ============= Encoder & Drive Helpers ================
// // ======================================================

// // reset encoder tick counters
// void resetEncoders() {
//   noInterrupts();
//   posA = 0;
//   posB = 0;
//   interrupts();
// }

// // drive straight without wall centering (used in open space)
// void driveStraight(int pwm) {
//   pwm = constrain(pwm, -255, 255);
//   motorL(pwm);
//   motorR(pwm);
// }


// bool driveOneCell() {
//   // --- constants ---
//   const int   BASE_PWM    = 80;
//   const int   MIN_PWM     = 70;
//   const int   MAX_PWM     = 90;
//   const float BAL_GAIN    = 0.5f;
//   const float ACCEL_MM    = 40.0f;   // distance to ramp up
//   const float DECEL_MM    = 50.0f;   // distance to ramp down
//   const float EARLY_STOP  = 25.0f;   // emergency stop distance
//   const float FRONT_SLOW  = 80.0f;   // slow down if wall ahead

//   resetEncoders();
//   int startA = posA, startB = posB;
//   bool stoppedEarly = false;
  
//   // ✅ Track mode transitions
//   bool wasInCorridor = false;
//   bool wasInOpenSpace = false;

//   while (true) {
//     // --- encoder progress ---
//     int   dA = abs(posA - startA), dB = abs(posB - startB);
//     float mmL = dA / TICKS_PER_MM_L;
//     float mmR = dB / TICKS_PER_MM_R;
//     float mmProg = fminf(mmL, mmR);
//     float fracA = (float)dA / ONE_CELL_TICKS_L();
//     float fracB = (float)dB / ONE_CELL_TICKS_R();
//     float prog  = (fracA + fracB) / 2.0f;

//     // --- check front sensor ---
//     float F = readMM(SID_FRONT, CH_FRONT);

//     // --- DYNAMIC wall detection (updated every loop!) ---
//     float L = readMM(SID_LEFT, CH_LEFT);
//     float R = readMM(SID_RIGHT, CH_RIGHT);
//     bool hasLeftWall = (L >= 0 && L < NO_WALL_MM);
//     bool hasRightWall = (R >= 0 && R < NO_WALL_MM);
//     bool noSideWalls = (!hasLeftWall && !hasRightWall);
    
//     // ✅ Detect mode transitions and reset PID state
//     bool inCorridor = !noSideWalls;
//     bool inOpenSpace = noSideWalls;
    
//     if (inCorridor && wasInOpenSpace) {
//       Serial.println("[TRANSITION] Open space -> Corridor: Resetting PID state");
//       driveCentered(BASE_PWM, 0.0f, true); // Reset PID state
//     }
    
//     if (inOpenSpace && wasInCorridor) {
//       Serial.println("[TRANSITION] Corridor -> Open space: Switching to encoder mode");
//       driveCentered(BASE_PWM, 0.0f, true); // Reset PID state
//     }
    
//     wasInCorridor = inCorridor;
//     wasInOpenSpace = inOpenSpace;

//     // 🚨 EMERGENCY STOPS (apply to both cases)
//     if (F >= 0 && F < EARLY_STOP) {
//       motorsStop();
//       Serial.printf("[driveOneCell] 🚨 EMERGENCY stop at %.0f%% cell, F=%.1f\n", prog * 100, F);
//       stoppedEarly = true;
//       break;
//     }
    
//     if (prog > 0.2f && F >= 0 && F < FRONT_STOP_MM) {
//       motorsStop();
//       Serial.printf("[driveOneCell] ⚠️ Early stop, wall ahead. prog=%.0f%%, F=%.1f\n", prog * 100, F);
//       stoppedEarly = true;
//       break;
//     }

//     // 🚨 SUCCESS CONDITION (apply to both cases)
//     if (prog >= 1.0f) {
//       motorsStop();
//       Serial.println("[driveOneCell] ✅ Cell completed successfully.");
//       return true;
//     }

//     // --- Calculate base speed with accel/decel ---
//     float remainL = ONE_CELL_TICKS_L() - dA;
//     float remainR = ONE_CELL_TICKS_R() - dB;
//     float remainMM = fminf(remainL / TICKS_PER_MM_L, remainR / TICKS_PER_MM_R);

//     float base = BASE_PWM;
//     if (mmProg < ACCEL_MM) {
//       float accelScale = fmaxf(mmProg / ACCEL_MM, 0.4f);
//       base = MIN_PWM + (BASE_PWM - MIN_PWM) * accelScale;
//     }
//     if (remainMM < DECEL_MM) {
//       float decelScale = fmaxf(remainMM / DECEL_MM, 0.4f);
//       base = fminf(base, MIN_PWM + (BASE_PWM - MIN_PWM) * decelScale);
//     }
//     if (F >= 0 && F < FRONT_SLOW) {
//       base = fmaxf(base * 0.6f, (float)MIN_PWM);
//     }

//     // 🚨 Case 1: OPEN SPACE (no side walls) - Pure encoder balancing
//     if (noSideWalls) {
//       Serial.printf("[driveOneCell] OPEN SPACE: L=%.1f R=%.1f prog=%.0f%% (ticks: %d,%d)\n", 
//                     L, R, prog * 100, dA, dB);
      
//       // ✅ GENTLE encoder balancing only - REDUCED gain further
//       int tickErr = (posA - startA) - (posB - startB);
//       float encoderCorrection = tickErr * 0.08f; // REDUCED from 0.15f to 0.08f
      
//       int leftPWM = constrain((int)(base + encoderCorrection), MIN_PWM, MAX_PWM);
//       int rightPWM = constrain((int)(base - encoderCorrection), MIN_PWM, MAX_PWM);
      
//       Serial.printf("[OPEN] tickErr=%d, correction=%.2f, PWM L=%d R=%d\n", 
//                     tickErr, encoderCorrection, leftPWM, rightPWM);
      
//       // Direct motor control - NO driveCentered()
//       motorL(leftPWM);
//       motorR(rightPWM);
      
//       buzzUpdate();
//       delay(10);
//       continue; // Skip the corridor logic below
//     }

//     // 🚨 Case 2: CORRIDOR (with walls) - Wall centering
//     Serial.printf("[driveOneCell] CORRIDOR: L=%.1f R=%.1f prog=%.0f%% (ticks: %d,%d)\n", 
//                   L, R, prog * 100, dA, dB);
    
//     float err = 0.0f;
//     if (hasLeftWall && hasRightWall) {
//       err = L - R;  // Center between both walls
//       Serial.printf("[CORRIDOR] Both walls: err=%.1f\n", err);
//     } else if (hasLeftWall) {
//       err = L - TARGET_SIDE_MM;  // Maintain distance from left wall
//       Serial.printf("[CORRIDOR] Left wall only: err=%.1f\n", err);
//     } else if (hasRightWall) {
//       err = -(R - TARGET_SIDE_MM);  // Maintain distance from right wall  
//       Serial.printf("[CORRIDOR] Right wall only: err=%.1f\n", err);
//     }

//     // Apply encoder tick balancing to base speed
//     int tickErr = (posA - startA) - (posB - startB);
//     base = base - BAL_GAIN * tickErr;
//     base = constrain(base, MIN_PWM, MAX_PWM);

//     // Use wall-centering PID for corridors
//     driveCentered((int)base, err, false);
    
//     buzzUpdate();
//     delay(10);
//   }

//   // --- Handle early stops ---
//   motorsStop();
//   int dA = abs(posA - startA);
//   int dB = abs(posB - startB);
//   float prog = ((float)dA / ONE_CELL_TICKS_L() + (float)dB / ONE_CELL_TICKS_R()) / 2.0f;

//   if (stoppedEarly) {
//     if (prog >= 0.90f) {
//       Serial.printf("[driveOneCell] ✅ Early stop but %.0f%% covered → ACCEPTED\n", prog * 100);
//       return true;
//     } else {
//       Serial.printf("[driveOneCell] ❌ Stopped early at %.0f%%\n", prog * 100);
//       return false;
//     }
//   }

//   return false; // fallback
// }

// void backtrackToStart() {
//   Serial.println("[BACKTRACK] Returning to start via recorded path...");

//   // reverse iterate through the path string
//   for (int i = fullPath.length() - 2; i >= 0; i -= 2) { // -2 skips trailing space
//     char step = fullPath[i];
//     char backStep;

//     // compute opposite move
//     if (step == 'N') backStep = 'S';
//     else if (step == 'S') backStep = 'N';
//     else if (step == 'E') backStep = 'W';
//     else if (step == 'W') backStep = 'E';
//     else backStep = '?';

//     Serial.printf("[BACKTRACK] Step %c -> %c\n", step, backStep);

//     // turn robot to face backStep
//     int dir;
//     if (backStep == 'N') dir = 0;
//     if (backStep == 'E') dir = 1;
//     if (backStep == 'S') dir = 2;
//     if (backStep == 'W') dir = 3;

//     int turnDeg = calculateTurn(dir);
//     if (turnDeg != 0) turnIMU(turnDeg, 80, 1800);
//     updateHeading(turnDeg);

//     // move one cell back
//     if (driveOneCell()) {
//       updatePosition();
//       buzzOK();
//     } else {
//       Serial.println("[BACKTRACK] ❌ Failed to move back one cell!");
//       break;
//     }
//   }

//   Serial.println("[BACKTRACK] Arrived back at (0,0)!");
// }


// // ======================================================
// // ====================  Setup  =========================
// // ======================================================
// void setup() {
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   // === Motors & buzzer ===
//   pinMode(IN1_LEFT, OUTPUT);
//   pinMode(IN2_LEFT, OUTPUT);
//   pinMode(IN1_RIGHT, OUTPUT);
//   pinMode(IN2_RIGHT, OUTPUT);
//   pinMode(BUZZER_PIN, OUTPUT);

//   // Immediately stop motors and give a short settle time
//   motorsStop();
//   delay(500);   // absorb any boot glitch on strapping pins

//   // === ToF sensors ===
//   tofBegin(CH_LEFT);
//   tofBegin(CH_ALEFT45);
//   tofBegin(CH_FRONT);
//   tofBegin(CH_ARIGHT45);
//   tofBegin(CH_RIGHT);

//   // === IMU (ensure robot is still) ===
//   imuBegin();

//   // === Encoders ===
//   pinMode(ENCA1, INPUT);
//   pinMode(ENCA2, INPUT);
//   pinMode(ENCB1, INPUT);
//   pinMode(ENCB2, INPUT);
//   attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

//   // === Maze mapping ===
//   initMaze();

//   // Set starting position (typically bottom-left corner)
//   robotX = 0;
//   robotY = 0;
//   robotHeading = 0;   // Facing North initially
//   stepCount = 0;
//   currentState = EXPLORING;
//   targetReached = false;

//   // Initial scan after sensors settle
//   delay(1000);
//   scanWalls();
//   printMaze();

//   // Boot chirp
//   buzzerBegin();
//   buzzOK();

//     Serial.printf("[init] Maze size = %d x %d\n", MAZE_SIZE, MAZE_SIZE);

//   if (MAZE_SIZE % 2 == 0) {
//     int c1 = CENTER1;
//     int c2 = CENTER2;
//     Serial.printf("[init] Even maze, center cells are: (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",
//                   c1, c1, c1, c2, c2, c1, c2, c2);
//   } else {
//     int mid = MAZE_SIZE / 2;
//     Serial.printf("[init] Odd maze, single center cell is: (%d,%d)\n", mid, mid);
//   }


//   Serial.println("Zeeker: Starting with PURE FLOOD FILL algorithm!");
// }


// // ======================================================
// // ====================  Main loop  =====================
// // ======================================================

// void loop() {
//   buzzUpdate();
//   static bool busy = false;
//   if (busy) return;
//   busy = true;

//   Serial.println("\n[FLOOD FILL] Starting new step...");
//   Serial.printf("[DEBUG] At (%d,%d) facing %s\n", robotX, robotY, dirNames[robotHeading]);

//   // --- Scan walls at current position (BEFORE TURN) ---
//   Serial.println("[SCAN] Sensor readings BEFORE turn:");
//   scanWalls();

//   // --- Dead-end reflex ---
//   float Lnow = readMM(SID_LEFT, CH_LEFT);
//   float Fnow = readMM(SID_FRONT, CH_FRONT);
//   float Rnow = readMM(SID_RIGHT, CH_RIGHT);

//   bool deadEndNow = (Lnow >= 0 && Lnow < NO_WALL_MM) &&
//                     (Fnow >= 0 && Fnow < NO_WALL_MM) &&
//                     (Rnow >= 0 && Rnow < NO_WALL_MM);

//   static unsigned long lastStuckBeep = 0;
//   if (deadEndNow) {
//     Serial.println("[REFLEX] Dead-end detected by sensors -> U-turn");
//     turnIMU(180, 85, 2000);
//     updateHeading(180);
//     squareUp();

//     // --- Debug AFTER U-turn ---
//     Serial.println("[SCAN] Sensor readings AFTER U-turn:");
//     scanWalls();

//     if (millis() - lastStuckBeep > 800) {
//       buzzAttention();
//       lastStuckBeep = millis();
//     }
//     busy = false;
//     return;
//   }

//   // --- Use flood fill to decide next move ---
//   int bestDirection = makeFloodFillDecision();
//   if (bestDirection == -1) {
//     Serial.println("[ERROR] No valid direction found! Stopping robot.");
//     motorsStop();
//     buzzAttention();
//     busy = false;
//     return;
//   }

//   // --- Calculate required turn ---
//   int turnDegrees = calculateTurn(bestDirection);
//   Serial.printf("[NAVIGATION] Need to go %s, currently facing %s -> Turn: %d°\n",
//                 dirNames[bestDirection], dirNames[robotHeading], turnDegrees);

//   // --- Execute turn if needed ---
//   if (turnDegrees == 90) {
//     Serial.println("[TURN] Executing LEFT turn (90°)");
//     turnIMU(+90, 80, 1600);
//     squareUp();
//     updateHeading(+90);

//   } else if (turnDegrees == -90) {
//     Serial.println("[TURN] Executing RIGHT turn (-90°)");
//     turnIMU(-90, 80, 1600);
//     squareUp();
//     updateHeading(-90);

//   } else if (turnDegrees == 180) {
//     Serial.println("[TURN] Executing U-turn (180°)");
//     turnIMU(180, 85, 1800);
//     squareUp();
//     updateHeading(180);

//   } else {
//     Serial.println("[TURN] No turn needed, going straight");
//   }

//   // --- Debug AFTER TURN ---
//   Serial.println("[SCAN] Sensor readings AFTER turn:");
//   scanWalls();

//   // --- Move forward one cell ---
//   Serial.println("[MOVEMENT] Moving forward one cell...");
//   if (driveOneCell()) {
//     float frontCheck = readMM(SID_FRONT, CH_FRONT);
//     if (frontCheck > 0 && frontCheck < NO_WALL_MM) {
//       squareUp();
//     }

//     updatePosition();
//     buzzOK();
//     Serial.println("[MOVEMENT] Forward movement successful");
//   } else {
//     Serial.println("[WARN] Forward move failed.");
//     buzzAttention();
//   }

//   // --- Print current state ---
//   printMaze();

//   delay(1000);
//   busy = false;
// }


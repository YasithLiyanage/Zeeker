// #include <Wire.h>
// #include "Adafruit_VL6180X.h"
// #include <MPU9250_asukiaaa.h>
// #include <math.h>

// // ========== Motor pins ==========
// #define PWM_LEFT    4
// #define PWM_RIGHT   5
// #define IN1_LEFT    15
// #define IN2_LEFT    21
// #define IN1_RIGHT   22
// #define IN2_RIGHT   23

// // ========== Encoders ==========
// #define ENCA1 16  
// #define ENCA2 17  
// #define ENCB1 18  
// #define ENCB2 19  

// volatile int posA = 0;           
// volatile int posB = 0;           
// volatile int lastEncodedA = 0;   
// volatile int lastEncodedB = 0;   

// // one-shot: relax front-stop for next cell after a U-turn
// bool relaxFrontStopNext = false;

// // ========== Buzzer ==========
// #define BUZZER_PIN 2
// #define BUZZ_CH    7
// bool BUZZ_ENABLED = true;
// static unsigned long buzz_end_ms = 0;

// void buzzerBegin() {
//   pinMode(BUZZER_PIN, OUTPUT);
//   ledcSetup(BUZZ_CH, 2000, 8);
//   ledcAttachPin(BUZZER_PIN, BUZZ_CH);
//   ledcWriteTone(BUZZ_CH, 0);
// }
// void buzzStart(int freqHz, int durMs) {
//   if (!BUZZ_ENABLED) return;
//   ledcWriteTone(BUZZ_CH, freqHz);
//   buzz_end_ms = millis() + durMs;
// }
// void buzzUpdate() {
//   if (buzz_end_ms != 0 && (long)(millis() - buzz_end_ms) >= 0) {
//     ledcWriteTone(BUZZ_CH, 0);
//     buzz_end_ms = 0;
//   }
// }
// void buzzOK()        { buzzStart(1500, 80); }
// void buzzAttention() { buzzStart(300, 50); }

// // ========== I2C / MUX ==========
// #define SDA_PIN     25
// #define SCL_PIN     26
// #define MUX_ADDR    0x70
// #define CH_LEFT      0
// #define CH_ALEFT45   1
// #define CH_FRONT     2
// #define CH_ARIGHT45  3
// #define CH_RIGHT     4

// Adafruit_VL6180X tof;
// MPU9250_asukiaaa mpu;

// // ========== Calibration ==========
// struct Cal { float a, b; };
// enum SensorId { SID_LEFT=0, SID_ALEFT45=1, SID_FRONT=2, SID_ARIGHT45=3, SID_RIGHT=4 };
// Cal CAL[5] = {
//   {1.00f, -42.0f}, {1.00f, 20.0f}, {1.00f, -35.0f}, {1.00f, 0.0f}, {1.00f, -28.0f}
// };

// // ========== Driving params ==========
// #define NO_WALL_MM        100
// #define TARGET_SIDE_MM    50
// #define FRONT_BLOCK_MM    32
// #define FRONT_CLEAR_MM    45
// #define FRONT_SLOW_MM     80
// #define FRONT_STOP_MM     38
// #define EMERGENCY_MM      28
// #define COMMIT_MM         20

// // ========== Square-up params ==========
// #define A45_TOL_MM        6
// #define SQUARE_TURN_PWM   50
// #define SQUARE_MAX_MS     1500
// #define GAP_TOL_MM        4
// #define NUDGE_PWM         60

// // ======= Maze grid & pose =======
// #define MAZE_W 16
// #define MAZE_H 16
// enum Heading { N=0, E=1, S=2, W=3 };

// struct Cell { uint8_t walls[4] = {0,0,0,0}; uint8_t known[4] = {0,0,0,0}; };
// Cell grid[MAZE_W][MAZE_H];

// struct Pose { int x=0; int y=0; Heading h=N; } pose;
// const char* HSTR = "NESW";

// // ---- Encoder ISRs ----
// void IRAM_ATTR updateEncoderA() {
//   int msb = digitalRead(ENCA1), lsb = digitalRead(ENCA2);
//   int encoded = (msb << 1) | lsb;
//   int sum = (lastEncodedA << 2) | encoded;
//   if (sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) posA++;
//   else if (sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) posA--;
//   lastEncodedA = encoded;
// }
// void IRAM_ATTR updateEncoderB() {
//   int msb = digitalRead(ENCB1), lsb = digitalRead(ENCB2);
//   int encoded = (msb << 1) | lsb;
//   int sum = (lastEncodedB << 2) | encoded;
//   if (sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) posB--;
//   else if (sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) posB++;
//   lastEncodedB = encoded;
// }

// // ---- Helpers for pose/grid ----
// inline void dirDelta(int d,int &dx,int &dy){ switch(d){case N:dx=0;dy=1;break;case E:dx=1;dy=0;break;case S:dx=0;dy=-1;break;case W:dx=-1;dy=0;break;}}
// inline int localToAbs(int local, Heading h){
//   static const int LUT[4][3]={{N,W,E},{E,N,S},{S,E,W},{W,S,N}};
//   return LUT[(int)h][local];
// }
// void setWall(int x,int y,int absDir,bool isWall,bool isKnown=true){
//   if(x<0||x>=MAZE_W||y<0||y>=MAZE_H) return;
//   grid[x][y].walls[absDir]=isWall; if(isKnown) grid[x][y].known[absDir]=1;
//   int dx,dy; dirDelta(absDir,dx,dy);
//   int nx=x+dx, ny=y+dy;
//   if(nx<0||nx>=MAZE_W||ny<0||ny>=MAZE_H) return;
//   int back=(absDir+2)%4;
//   grid[nx][ny].walls[back]=isWall; if(isKnown) grid[nx][ny].known[back]=1;
// }
// void applyTurnToHeading(int turnDeg){ int k=((turnDeg/90)%4+4)%4; pose.h=(Heading)(((int)pose.h - k + 4)%4); }
// void stepForwardUpdatePose(){ switch(pose.h){case N:pose.y++;break;case E:pose.x++;break;case S:pose.y--;break;case W:pose.x--;break;} if(pose.x<0)pose.x=0;if(pose.x>=MAZE_W)pose.x=MAZE_W-1;if(pose.y<0)pose.y=0;if(pose.y>=MAZE_H)pose.y=MAZE_H-1;}

// // ---- ToF helpers ----
// void mux(uint8_t ch){ Wire.beginTransmission(MUX_ADDR); Wire.write(1<<ch); Wire.endTransmission(); delayMicroseconds(200);}
// bool tofBegin(uint8_t ch){ mux(ch); delay(3); return tof.begin(); }
// float readMM(SensorId sid,uint8_t ch){ mux(ch); delay(2); uint8_t r=tof.readRange(); if(tof.readRangeStatus()!=VL6180X_ERROR_NONE)return -1.0f; float mm=CAL[sid].a*r+CAL[sid].b; if(mm<0)mm=0; if(mm>NO_WALL_MM)return -1.0f; return mm;}
// inline bool senseIsWall(float mm){ return(mm>=0&&mm<60.0f); }
// float readMM_median3(SensorId sid,uint8_t ch){ float a=readMM(sid,ch),b=readMM(sid,ch),c=readMM(sid,ch); float v[3]={a,b,c}; for(int i=0;i<3;i++)for(int j=i+1;j<3;j++)if(v[j]<v[i]){float t=v[i];v[i]=v[j];v[j]=t;} return v[1]; }

// void mapWallsAtCurrentCellFromSensors(){
//   float F=readMM_median3(SID_FRONT,CH_FRONT),L=readMM_median3(SID_LEFT,CH_LEFT),R=readMM_median3(SID_RIGHT,CH_RIGHT);
//   bool wF=senseIsWall(F),wL=senseIsWall(L),wR=senseIsWall(R);
//   int dF=localToAbs(0,pose.h),dL=localToAbs(1,pose.h),dR=localToAbs(2,pose.h);
//   setWall(pose.x,pose.y,dF,wF,true); setWall(pose.x,pose.y,dL,wL,true); setWall(pose.x,pose.y,dR,wR,true);
//   Serial.printf("[map] Cell (%d,%d) H=%c | abs(F=%c L=%c R=%c)\n",pose.x,pose.y,HSTR[pose.h],wF?'#':'.',wL?'#':'.',wR?'#':'.');
// }
// void seedOuterWalls(){ for(int x=0;x<MAZE_W;x++){ setWall(x,0,S,true,true); setWall(x,MAZE_H-1,N,true,true);} for(int y=0;y<MAZE_H;y++){ setWall(0,y,W,true,true); setWall(MAZE_W-1,y,E,true,true);} }

// // ---- Motors ----
// void motorL(int pwm){ pwm=constrain(pwm,-255,255); bool fwd=(pwm>=0); digitalWrite(IN1_LEFT,fwd?HIGH:LOW); digitalWrite(IN2_LEFT,fwd?LOW:HIGH); analogWrite(PWM_LEFT,abs(pwm));}
// void motorR(int pwm){ pwm=constrain(pwm,-255,255); bool fwd=(pwm>=0); digitalWrite(IN1_RIGHT,fwd?HIGH:LOW); digitalWrite(IN2_RIGHT,fwd?LOW:HIGH); analogWrite(PWM_RIGHT,abs(pwm));}
// void motorsStop(){ motorL(0); motorR(0); }

// // ---- Drive & Square-up ----
// void driveCentered(int basePWM,float err){ static float err_prev=0,err_sum=0; static unsigned long last=millis(); unsigned long now=millis(); float dt=(now-last)/1000.0f;if(dt<=0)dt=0.001f; last=now; const float Kp=0.8f,Ki=0,Kd=0.1f; err_sum+=err*dt; float derr=(err-err_prev)/dt; float turn=Kp*err+Ki*err_sum+Kd*derr; err_prev=err; int l=constrain((int)(basePWM+turn),0,255); int r=constrain((int)(basePWM-turn),0,255); motorL(l); motorR(r);}
// void squareUp(){ Serial.println(">>> Square-up START"); unsigned long t0=millis(); while(millis()-t0<SQUARE_MAX_MS){ float al=readMM(SID_ALEFT45,CH_ALEFT45),ar=readMM(SID_ARIGHT45,CH_ARIGHT45); if(al<0||ar<0)break; float diff=ar-al; if(fabs(diff)<=A45_TOL_MM)break; if(diff>0){ motorL(-SQUARE_TURN_PWM); motorR(SQUARE_TURN_PWM);} else{ motorL(SQUARE_TURN_PWM); motorR(-SQUARE_TURN_PWM);} delay(20);} motorsStop(); float f=readMM(SID_FRONT,CH_FRONT); if(f>0&&fabs(f-FRONT_STOP_MM)>GAP_TOL_MM){ int dir=(f>FRONT_STOP_MM)?+1:-1; motorL(dir*NUDGE_PWM); motorR(dir*NUDGE_PWM); delay(150); motorsStop();} Serial.println(">>> Square-up DONE"); }

// // ---- IMU ----
// static float gyroBiasZ=0.0f;
// void imuBegin(){ mpu.setWire(&Wire); mpu.beginGyro(); unsigned long t0=millis(); double sum=0; long n=0; while(millis()-t0<1200){ mpu.gyroUpdate(); sum+=mpu.gyroZ(); n++; delay(2);} gyroBiasZ=(n>0)?(float)(sum/n):0.0f; Serial.printf("IMU bias Z=%.3f deg/s\n",gyroBiasZ);}
// void turnIMU(float angleDeg,int basePWM=90,int timeout_ms=1500){ if(angleDeg==0)return; int dir=(angleDeg>0)?+1:-1; float target=fabsf(angleDeg); float yaw=0; unsigned long last=millis(),t0=last; const float slowBand=18.0f; const int minPWM=70; while(true){ if((int)(millis()-t0)>timeout_ms)break; mpu.gyroUpdate(); float gz=mpu.gyroZ()-gyroBiasZ; unsigned long now=millis(); float dt=(now-last)/1000.0f;if(dt<=0)dt=0.001f; last=now; yaw+=fabsf(gz)*dt; if(yaw>=target)break; float remaining=target-yaw; float scale=(remaining<slowBand)?fmaxf(remaining/slowBand,0.2f):1.0f; int pwm=(int)fmaxf((basePWM*scale),(float)minPWM); motorL((dir>0)?-pwm:+pwm); motorR((dir>0)?+pwm:-pwm); buzzUpdate(); delay(2);} motorsStop(); delay(60); }

// // ---- One-cell drive ----
// float TICKS_PER_MM_L=13.56f, TICKS_PER_MM_R=13.45f; int CELL_MM=180;
// int ONE_CELL_TICKS_L(){ return (int)roundf(TICKS_PER_MM_L*CELL_MM);}
// int ONE_CELL_TICKS_R(){ return (int)roundf(TICKS_PER_MM_R*CELL_MM);}
// bool driveOneCell(bool relaxFrontStop=false){
//   const int BASE_PWM=90, MIN_PWM=70, MAX_PWM=110; const float BAL_GAIN=0.5f;
//   int startA=posA,startB=posB;
//   while((abs(posA-startA)<ONE_CELL_TICKS_L())||(abs(posB-startB)<ONE_CELL_TICKS_R())){
//     int dA=abs(posA-startA),dB=abs(posB-startB); float mmL=dA/TICKS_PER_MM_L,mmR=dB/TICKS_PER_MM_R; float mmProg=fminf(mmL,mmR);
//     float F=readMM(SID_FRONT,CH_FRONT);
//     if(F>=0&&F<=EMERGENCY_MM){ motorsStop(); Serial.println("[driveOneCell] EMERGENCY stop"); return false; }
//     const float COMMIT_MM_RELAX=70.0f; float commitGate=relaxFrontStop?COMMIT_MM_RELAX:COMMIT_MM;
//     if(mmProg>commitGate&&F>=0&&F<FRONT_STOP_MM){ motorsStop(); Serial.println("[driveOneCell] Early stop: front wall"); return false; }
//     float L=readMM(SID_LEFT,CH_LEFT),R=readMM(SID_RIGHT,CH_RIGHT); float err=0; bool leftOK=(L>=0),rightOK=(R>=0);
//     if(leftOK&&rightOK)err=L-R; else if(leftOK&&!rightOK)err=L-TARGET_SIDE_MM; else if(!leftOK&&rightOK)err=-(R-TARGET_SIDE_MM);
//     int remainL=ONE_CELL_TICKS_L()-dA,remainR=ONE_CELL_TICKS_R()-dB; float remainMM=fminf(remainL/TICKS_PER_MM_L,remainR/TICKS_PER_MM_R);
//     int base=BASE_PWM; if(remainMM<40){ float scale=fmaxf(remainMM/40.0f,0.4f); base=(int)fmaxf(BASE_PWM*scale,(float)MIN_PWM);}
//     if(F>=0&&F<FRONT_SLOW_MM) base=(int)fmaxf((float)base*0.7f,(float)MIN_PWM);
//     int tickErr=(posA-startA)-(posB-startB); base=(int)roundf(base-BAL_GAIN*tickErr); base=constrain(base,MIN_PWM,MAX_PWM);
//     driveCentered(base,err); buzzUpdate(); delay(10);
//   }
//   motorsStop(); Serial.printf("[driveOneCell] Done: posA=%d posB=%d\n",posA,posB); return true;
// }

// // ---- Finalize cell step ----
// void finalizeCellStep(bool advanced){
//   squareUp(); delay(60);
//   if(advanced) stepForwardUpdatePose();
//   mapWallsAtCurrentCellFromSensors();
//   buzzOK();
// }

// // ---- Setup ----
// void setup(){
//   Serial.begin(115200);
//   Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000);
//   pinMode(IN1_LEFT,OUTPUT); pinMode(IN2_LEFT,OUTPUT); pinMode(IN1_RIGHT,OUTPUT); pinMode(IN2_RIGHT,OUTPUT);
//   motorsStop();
//   tofBegin(CH_LEFT); tofBegin(CH_ALEFT45); tofBegin(CH_FRONT); tofBegin(CH_ARIGHT45); tofBegin(CH_RIGHT);
//   imuBegin();
//   pinMode(ENCA1,INPUT); pinMode(ENCA2,INPUT); pinMode(ENCB1,INPUT); pinMode(ENCB2,INPUT);
//   attachInterrupt(digitalPinToInterrupt(ENCA1),updateEncoderA,CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCA2),updateEncoderA,CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCB1),updateEncoderB,CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENCB2),updateEncoderB,CHANGE);
//   buzzerBegin(); buzzOK();
//   Serial.println("Zeeker: centered-drive + square-up + IMU turns + QUAD encoders + buzzer.");
//   seedOuterWalls();
// }

// // ---- Loop ----
// void loop(){
//   buzzUpdate();
//   static bool busy=false; if(busy)return; busy=true;
//   Serial.println("\n[Cell-to-cell] Step...");
//   auto isOpen=[](float mm){ return(mm<0); };
//   float F0=readMM_median3(SID_FRONT,CH_FRONT);
//   if(F0>=0&&F0<FRONT_STOP_MM){
//     motorsStop(); squareUp(); delay(60);
//     float L=readMM_median3(SID_LEFT,CH_LEFT),F=readMM_median3(SID_FRONT,CH_FRONT),R=readMM_median3(SID_RIGHT,CH_RIGHT);
//     int turnDeg; if(isOpen(L))turnDeg=+90; else if(isOpen(F))turnDeg=0; else if(isOpen(R))turnDeg=-90; else turnDeg=180;
//     Serial.printf("[Decision] L=%.1f F=%.1f R=%.1f -> turn %d\n",L,F,R,turnDeg);
//     if(turnDeg!=0){ if(turnDeg==90)turnIMU(+90,80,1600); if(turnDeg==-90)turnIMU(-90,80,1600); if(turnDeg==180)turnIMU(180,85,1800);
//       applyTurnToHeading(turnDeg); delay(60); squareUp();
//     }
//   }
//   bool ok=driveOneCell(relaxFrontStopNext); relaxFrontStopNext=false;
//   Serial.printf("[Cell-to-cell] driveOneCell done, ok=%d\n",ok?1:0);
//   finalizeCellStep(ok);   // <---- always settle + map + beep
//   busy=false;
// }

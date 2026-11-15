// Completion-priority candidate, 2026-11-13.
// Empty cruise170 / approach125 / recovery120; decelerate at55% nominal.
// Motor balance ratio and measured timed-turn settings are preserved.
// Mission2 loaded cruise130; loaded approach100 / recovery110 preserved.
// Host sensor-event tests passed normal scenarios; physical stability is UNVERIFIED.
// Use the included report for model assumptions and failed stress scenarios.
/*
 * Mission 2 field map 2026-11-13.
 * Start external T5 -> H5, facing NORTH (START_AT_EXTERNAL_T=true).
 * Set false ONLY if already centered on H5 itself.
 * Pickup1 external T beyond A6 -> drop1 external T beyond H2.
 * Pickup2 external T beyond A4 -> drop2 external T beyond H6.
 * D4-D6 fixed; one obstacle D1-D3, one D7-D8.
 * Empty calibration from attached Mission1; loaded calibration retained.
 * RFID [removed for publication]. Reset stops/restarts the sketch.
 * T center times are initial estimates, NOT measured position feedback.
 * Existing pickup maneuver retained: empty U-turn before lifting.
 * Check fork remains under pallet after this U-turn.
 */
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

const uint8_t PIN_L_PWM=5, PIN_R_PWM=6, PIN_L_DIR=7, PIN_R_DIR=8;
const uint8_t PIN_IR_L=A6, PIN_IR_R=A7, PIN_FRONT=A0;
const uint8_t PIN_BUZZER=3, PIN_LIFT=9;
MFRC522 rfid(2,4);
Servo lifter;

// ===== 빈 차: 사용자가 성공 확인한 종목1 파일 기준 =====
const int EMPTY_PWM=170, EMPTY_APPROACH_PWM=125, EMPTY_RECOVERY_PWM=100;
const int EMPTY_KP=40, EMPTY_KD=35, EMPTY_CORRECTION_MAX=75;
const int EMPTY_IR_WHITE=470, EMPTY_IR_BLACK=960, EMPTY_IR_THRESHOLD=720;
const int EMPTY_ALIGN_PWM=112;
const unsigned int EMPTY_ALIGN_MS=40, EMPTY_SETTLE_MS=70;
const int EMPTY_START_PWM=100, EMPTY_RAMP_STEP=0; // 0: 종목1처럼 바로 목표 출력
const int EMPTY_LEFT_RATIO_NUM=182, EMPTY_LEFT_RATIO_DEN=207;
const int TURN_PWM_EMPTY=127;
const unsigned int LEFT90_EMPTY=340, RIGHT90_EMPTY=350, U180_EMPTY=660;

// ===== 적재: 사용자 보정값 =====
const int LOADED_PWM=100, LOADED_APPROACH_PWM=90, LOADED_RECOVERY_PWM=100;
const int LOADED_KP=40, LOADED_KD=55, LOADED_CORRECTION_MAX=25;
const int LOADED_IR_WHITE=470, LOADED_IR_BLACK=960, LOADED_IR_THRESHOLD=720;
const int LOADED_ALIGN_PWM=100;
const unsigned int LOADED_ALIGN_MS=40, LOADED_SETTLE_MS=200;
const unsigned int LOADED_PRETURN_EXTRA_SETTLE_MS=200; // 적재 회전 직전만 추가 안정화 대기
const int LOADED_START_PWM=80, LOADED_RAMP_STEP=2;
const int LOADED_LEFT_RATIO_NUM=135, LOADED_LEFT_RATIO_DEN=140;
const int LEFT90_PWM_LOADED=100, RIGHT90_PWM_LOADED=100, U180_PWM_LOADED=100;
const unsigned int LEFT90_LOADED=455, RIGHT90_LOADED=455, U180_LOADED=890;

// 적재 회전 후 IR 미세 보정: 종목1의 PWM70 / 12ms를 초기값으로 사용.
const bool LOADED_TRIM_ENABLED=true;
const int LOADED_TRIM_PWM=70, LOADED_TRIM_ERROR=280;
const uint8_t LOADED_TRIM_MAX_STEPS=8;
const unsigned int LOADED_TRIM_MS=12, LOADED_TRIM_SETTLE_MS=28;

// ===== 팔레트 아래 진입/이탈: 통상 주행과 별도 =====
const int EMPTY_DOCK_PWM=100, LOADED_DOCK_PWM=100;
const unsigned int PICKUP_ALIGN_MS=30; // T 중심 보정 초기값: 실측 필요
const unsigned int DROP_ALIGN_MS=30;
const unsigned long DOCK_TIMEOUT_MS=4500;
const bool START_AT_EXTERNAL_T=true;
const int START_EDGE_PWM=145;
const unsigned int START_T_ALIGN_MS=40; // H5 센서 검출 후 중심 보정
const uint8_t LIFT_DOWN=55, LIFT_UP=170;
const unsigned long LIFT_STEP_MS=20, LIFT_SETTLE_MS=500;

// ===== 공통 =====
const int MIN_PWM=50;
const unsigned long RAMP_INTERVAL_MS=10;
const unsigned long EDGE_MS120=1130, EDGE_TIMEOUT_MS=4500;
const unsigned int MIN_NODE_GAP_MS=190;
const int FRONT_THRESHOLD=90;
const bool FRONT_NEAR_IS_LOW=false;
const uint8_t FRONT_SAMPLES=15, FRONT_VOTES=11;
const unsigned long MISSION_TIMEOUT_MS=180000;
const uint8_t TEST_SCAN_ONLY=0;

// 행: A=0 ... H=7, 열: 1...8. 방향: 북0 동1 남2 서3.
int row=7,col=5,heading=0,openCol=0,previousError=0;
int leftOpenCol=0,rightOpenCol=0;
int liftAngle=LIFT_DOWN;
bool running=false,aborted=false,carrying=false;
unsigned long missionStart=0;
int travelPower=0;
unsigned long travelRampAt=0;

bool axisAligned=true;
unsigned int settleMs(){return carrying?LOADED_SETTLE_MS:EMPTY_SETTLE_MS;}
int alignPWM(){return carrying?LOADED_ALIGN_PWM:EMPTY_ALIGN_PWM;}
unsigned int alignMs(){return carrying?LOADED_ALIGN_MS:EMPTY_ALIGN_MS;}

void rawMotor(int l,int r){
  l=constrain(l,-255,255);r=constrain(r,-255,255);
  digitalWrite(PIN_L_DIR,l>=0?HIGH:LOW);
  digitalWrite(PIN_R_DIR,r>=0?LOW:HIGH);
  analogWrite(PIN_L_PWM,abs(l));analogWrite(PIN_R_PWM,abs(r));
}
void stopMotors(){rawMotor(0,0);travelPower=0;previousError=0;}
void forward(int l,int r){
  int num=carrying?LOADED_LEFT_RATIO_NUM:EMPTY_LEFT_RATIO_NUM;
  int den=carrying?LOADED_LEFT_RATIO_DEN:EMPTY_LEFT_RATIO_DEN;
  rawMotor((long)constrain(l,0,255)*num/den,constrain(r,0,255));
}
bool fail(const __FlashStringHelper *why){
  stopMotors();aborted=true;Serial.print(F("FAULT: "));Serial.println(why);return false;
}
bool safe(){
  if(aborted)return false;
  if(running && millis()-missionStart>=MISSION_TIMEOUT_MS)return fail(F("TIME LIMIT"));
  return true;
}
bool waitSafe(unsigned long ms){
  unsigned long t=millis();while(millis()-t<ms){if(!safe())return false;delay(2);}return true;
}
int blackness(int v){
  int white=carrying?LOADED_IR_WHITE:EMPTY_IR_WHITE;
  int black=carrying?LOADED_IR_BLACK:EMPTY_IR_BLACK;
  return constrain((int)((long)(v-white)*1000/(black-white)),0,1000);
}
void lineControl(int power, int l, int r) {
  int e = blackness(l) - blackness(r);

  // 적재 상태에서만 보정 강도와 최대 보정량을 낮춤
  int kp = carrying ? LOADED_KP : EMPTY_KP;
  int kd = carrying ? LOADED_KD : EMPTY_KD;
  int maxCorrection = carrying ? LOADED_CORRECTION_MAX : EMPTY_CORRECTION_MAX;

  long corr =
      ((long)kp * e + (long)kd * (e - previousError)) / 1000;

  previousError = e;
  corr = constrain(corr, -(long)maxCorrection, (long)maxCorrection);

  forward(
    constrain(power - (int)corr, MIN_PWM, 255),
    constrain(power + (int)corr, MIN_PWM, 255)
  );
}
bool alignAndStop(unsigned int duration){
  if(!axisAligned){
    forward(alignPWM(),alignPWM());
    if(!waitSafe(duration))return false;
    axisAligned=true;
  }
  stopMotors();return waitSafe(settleMs());
}
// 제한적인 회전 후 보정. 종료는 절대각 정렬 성공을 의미하지 않는다.
bool trimLoadedHeading(){
  if(!carrying || !LOADED_TRIM_ENABLED)return true;
  int lastSign=0;
  for(uint8_t j=0;j<LOADED_TRIM_MAX_STEPS;++j){
    if(!safe())return false;
    int l=analogRead(PIN_IR_L),r=analogRead(PIN_IR_R);
    bool lb=l>=LOADED_IR_THRESHOLD,rb=r>=LOADED_IR_THRESHOLD;
    // 모두 흰색: 선 위치 불명. 모두 검정: 교차선 가능. 추측 회전하지 않음.
    if(lb==rb)break;
    int e=blackness(l)-blackness(r);
    if(abs(e)<LOADED_TRIM_ERROR)break;
    int sign=e>0?1:-1;
    // 반대쪽으로 넘어갔다면 왕복 보정하지 않고 종료.
    if(lastSign && sign!=lastSign)break;
    lastSign=sign;
    rawMotor(-sign*LOADED_TRIM_PWM,sign*LOADED_TRIM_PWM);
    if(!waitSafe(LOADED_TRIM_MS)){stopMotors();return false;}
    stopMotors();
    if(!waitSafe(LOADED_TRIM_SETTLE_MS))return false;
  }
  previousError=0;
  return true;
}
bool turnTo(int target){
  int delta=(target-heading+4)%4;if(!delta)return true;
  if(!alignAndStop(alignMs()))return false;
  if(carrying && !waitSafe(LOADED_PRETURN_EXTRA_SETTLE_MS))return false;
  int pwm=TURN_PWM_EMPTY;
  unsigned int ms;
  bool left=(delta==3 || delta==2);
  if(delta==2){
    if(carrying)pwm=U180_PWM_LOADED;
    ms=carrying?U180_LOADED:U180_EMPTY;
  }else if(left){
    if(carrying)pwm=LEFT90_PWM_LOADED;
    ms=carrying?LEFT90_LOADED:LEFT90_EMPTY;
  }else{
    if(carrying)pwm=RIGHT90_PWM_LOADED;
    ms=carrying?RIGHT90_LOADED:RIGHT90_EMPTY;
  }
  rawMotor(left?-pwm:pwm,left?pwm:-pwm);
  if(!waitSafe(ms)){stopMotors();return false;}
  stopMotors();if(!waitSafe(settleMs()))return false;
  // 적재 좌/우90도 및 유턴의 기본 회전/대기 후, 출발 전에만 실행.
  if(!trimLoadedHeading())return false;
  heading=target;previousError=0;return true;
}

// 빈 차 일반 주행: 종목1 (첨부9)의 센서 판단, PD 초기화, 감속/타임아웃을 적용.
// 팔레트 진입(dockMode)은 기존 제어 유지. 적재 일반 주행도 동일한 탐색 정책 적용.
bool seekNextEmptyM1(){
  unsigned long t=millis();
  unsigned long nominal=EDGE_MS120*120UL/EMPTY_PWM;
  unsigned long slowAt=nominal*55UL/100UL;
  unsigned long timeout=nominal*175UL/100UL+160UL;
  bool armed=false;
  uint8_t clearCount=0,hits=0;
  previousError=0;
  while(millis()-t<timeout){
    if(!safe())return false;
    int l=analogRead(PIN_IR_L),r=analogRead(PIN_IR_R);
    bool lb=l>=EMPTY_IR_THRESHOLD,rb=r>=EMPTY_IR_THRESHOLD;
    if(!(lb&&rb)){
      if(clearCount<3)++clearCount;
      if(clearCount>=3)armed=true;
    }else clearCount=0;
    unsigned long elapsed=millis()-t;
    if(armed && elapsed>=MIN_NODE_GAP_MS && lb&&rb){
      if(++hits>=2){axisAligned=false;return true;}
    }else hits=0;
    int power=elapsed>=slowAt?EMPTY_APPROACH_PWM:EMPTY_PWM;
    if(lb!=rb && power>EMPTY_RECOVERY_PWM)power=EMPTY_RECOVERY_PWM;
    travelPower=power;
    if(lb&&rb)forward(power,power);else lineControl(power,l,r);
    delay(2);
  }
  stopMotors();
  Serial.print(F("TIMEOUT AT row="));Serial.print(row);
  Serial.print(F(" col="));Serial.print(col);
  Serial.print(F(" IR_L="));Serial.print(analogRead(PIN_IR_L));
  Serial.print(F(" IR_R="));Serial.println(analogRead(PIN_IR_R));
  return fail(F("INTERSECTION TIMEOUT"));
}

// 적재 일반 주행: 종목1 탐색 정책 + 기존 적재 출력/가속 설정. 팔레트 진입은 유지.
bool seekNext(bool dockMode,bool stopAtTarget){
#ifdef M2_ROUTE_TEST
  axisAligned=false;return safe();
#else
  if(!carrying && !dockMode)return seekNextEmptyM1();
  int cruise=dockMode?(carrying?LOADED_DOCK_PWM:EMPTY_DOCK_PWM):(carrying?LOADED_PWM:EMPTY_PWM);
  int approach=carrying?LOADED_APPROACH_PWM:EMPTY_APPROACH_PWM;
  int recovery=carrying?LOADED_RECOVERY_PWM:EMPTY_RECOVERY_PWM;
  int threshold=carrying?LOADED_IR_THRESHOLD:EMPTY_IR_THRESHOLD;
  int rampStep=carrying?LOADED_RAMP_STEP:EMPTY_RAMP_STEP;
  if(travelPower==0){
    travelPower=carrying?LOADED_START_PWM:EMPTY_START_PWM;
    travelRampAt=millis();previousError=0;
  }
  unsigned long t=millis();
  if(!dockMode)previousError=0; // 일반 주행은 교차점마다 초기화
  unsigned long nominal=EDGE_MS120*120UL/cruise;
  unsigned long slowAt=nominal*55UL/100UL;
  unsigned long timeout=dockMode?DOCK_TIMEOUT_MS:(nominal*175UL/100UL+160UL);
  bool armed=false;uint8_t clearCount=0,hits=0;
  while(millis()-t<timeout){
    if(!safe())return false;
    int l=analogRead(PIN_IR_L),r=analogRead(PIN_IR_R);
    bool lb=l>=threshold,rb=r>=threshold;
    if(!(lb&&rb)){if(clearCount<3)++clearCount;if(clearCount>=3)armed=true;}else clearCount=0;
    if(armed && millis()-t>=(dockMode?20UL:MIN_NODE_GAP_MS) && lb&&rb){
      if(++hits>=2){axisAligned=false;return true;}
    }else hits=0;
    int target=(!dockMode && millis()-t>=slowAt)?approach:cruise;
    if(lb!=rb && target>recovery)target=recovery;
    // 팔레트 진입 시 멈추지 않고 저속 상한으로 전환.
    if(dockMode && travelPower>cruise)travelPower=cruise;
    // 가속은 기존 램프 유지, 감속(선 이탈 복구 등)은 즉시 적용.
    if(rampStep==0 || target<travelPower){
      travelPower=target;
    }else if(millis()-travelRampAt>=RAMP_INTERVAL_MS){
      travelRampAt=millis();
      if(travelPower<target)travelPower=min(travelPower+rampStep,target);
    }
    if(lb&&rb)forward(travelPower,travelPower);else lineControl(travelPower,l,r);
    delay(2);
  }
  stopMotors();
  Serial.print(F("TIMEOUT AT row="));Serial.print(row);
  Serial.print(F(" col="));Serial.print(col);
  Serial.print(F(" IR_L="));Serial.print(analogRead(PIN_IR_L));
  Serial.print(F(" IR_R="));Serial.println(analogRead(PIN_IR_R));
  return fail(F("INTERSECTION TIMEOUT"));
#endif
}
// Full 8x8 grid; every entered D node must be a selected clear corridor.
bool edgeAllowed(int nr,int nc){
  if(nr<0 || nr>7 || nc<1 || nc>8)return false;
  if(abs(nr-row)+abs(nc-col)!=1)return false;
  if(nr==3 || row==3){
    if(nc!=col || col!=openCol)return false;
    if(col>=4 && col<=6)return false;
  }
  return true;
}
bool goTo(int tr,int tc,bool stopAtEnd,bool dockMode){
  if(tr!=row && tc!=col)return fail(F("DIAGONAL ROUTE"));
  if(tr==row && tc==col)return true;
  int dir=tr<row?0:(tr>row?2:(tc>col?1:3));
  if(!turnTo(dir))return false;
  while(row!=tr || col!=tc){
    int nr=row+(dir==2)-(dir==0),nc=col+(dir==1)-(dir==3);
    if(!edgeAllowed(nr,nc))return fail(F("ILLEGAL EDGE"));
    if(!seekNext(dockMode,stopAtEnd && nr==tr && nc==tc))return false;
    row=nr;col=nc;
    // 주행 중 시리얼 출력 생략: 교차점 통과 직후 제어를 이어감.
#ifdef M2_ROUTE_TEST
    testNode(row,col);
#endif
  }
  if(stopAtEnd)return alignAndStop(alignMs());
  return true;
}
bool go(int tr,int tc){return goTo(tr,tc,true,false);}
// -1: aborted, 0: open, 1: blocked, 2: unstable -> stop (never assume open).
int probeFront(){
#ifdef M2_ROUTE_TEST
  return testProbe(col);
#else
  stopMotors();if(!waitSafe(300))return -1;
  uint8_t votes=0;
  for(uint8_t i=0;i<FRONT_SAMPLES;++i){
    if(!safe())return -1;
    int v=analogRead(PIN_FRONT);
    Serial.print(F("FRONT "));Serial.println(v);
    if(FRONT_NEAR_IS_LOW?v<FRONT_THRESHOLD:v>FRONT_THRESHOLD)++votes;
    if(!waitSafe(20))return -1;
  }
  if(votes>=FRONT_VOTES)return 1;
  if(votes<=FRONT_SAMPLES-FRONT_VOTES)return 0;
  return 2;
#endif
}
bool scanLeft(){
  // H5 -> E5 -> E1. Skip F1; probe D1 only after reaching E1.
  if(!go(4,5)||!go(4,1))return false;
  for(int candidate=1;candidate<=3;++candidate){
    if(col!=candidate && !go(4,candidate))return false;
    if(!turnTo(0))return false;
    int result=probeFront();
    if(result<0)return false;
    if(result==2)return fail(F("FRONT UNSTABLE"));
    if(result==0){leftOpenCol=openCol=candidate;return true;}
    // D1-D3: blocked -> move east along E and scan the next column.
  }
  return fail(F("LEFT ALL BLOCKED"));
}
bool scanRight(){
  // After unloading1: H2 -> H7 -> E7, directly north along column7.
  if(!go(7,7)||!go(4,7)||!turnTo(0))return false;
  int result=probeFront();
  if(result<0)return false;
  if(result==2)return fail(F("FRONT UNSTABLE"));
  if(result==1){
    // Exactly ONE obstacle in D7/D8: infer D8 clear, no second scan.
    if(!go(4,8)||!turnTo(0))return false;
    rightOpenCol=openCol=8;
  }else rightOpenCol=openCol=7;
  return true;
}
bool liftTo(int target){
  stopMotors();
  while(liftAngle!=target){
    if(!safe())return false;
    liftAngle+=target>liftAngle?1:-1;lifter.write(liftAngle);
    if(!waitSafe(LIFT_STEP_MS))return false;
  }
  return waitSafe(LIFT_SETTLE_MS); // Keep servo attached to hold the load.
}
// Outside A/H: retain the boundary coordinates until the same boundary is reacquired.
bool advanceToPallet(){
  // A/H를 벗어난 후 양쪽 IR이 다음 검은 가로선을 읽어야 도착으로 인정.
  if(!seekNext(true,true))return false;
  int dockPWM=carrying?LOADED_DOCK_PWM:EMPTY_DOCK_PWM;
  forward(dockPWM,dockPWM);
  if(!waitSafe(carrying?DROP_ALIGN_MS:PICKUP_ALIGN_MS))return false;
  axisAligned=true;stopMotors();
  Serial.println(F("PALLET CROSS DETECTED / ALIGNED"));
  return waitSafe(settleMs());
}
bool returnToBoundary(bool turnFirst){
  // 상차는 리프트 상승 전에 이미 유턴함. 하역 후에는 여기서 유턴.
  if(turnFirst && !turnTo((heading+2)%4))return false;
  // First intersection on return is A/H, NOT B/G. Do not increment row here.
  if(!seekNext(true,false))return false;
  // A/H는 복귀 중간 노드. 정렬/정지 없이 B/G까지 이어서 이동.
  return true;
}
bool pickup(int c){
  if(row!=1||col!=c||carrying)return fail(F("PICKUP STATE"));
  if(liftAngle!=LIFT_DOWN && !liftTo(LIFT_DOWN))return false;
  if(!goTo(0,c,false,true))return false;
  if(!advanceToPallet())return false;
  // 아직 빈 차/리프트55 상태: 빈 차 PWM 및 U180_EMPTY로 먼저 유턴.
  if(!turnTo((heading+2)%4))return false;
  // 유턴 후에도 리프트가 팔레트 아래에 위치한다는 기구 조건이 필요.
  if(!liftTo(LIFT_UP))return false;
  carrying=true;
  Serial.println(F("PICKUP COMMAND COMPLETE"));
#ifdef M2_ROUTE_TEST
  testWork(true,c);
#endif
  // Re-detect A boundary with sensors, then count the next intersection as B.
  return returnToBoundary(false) && go(1,c);
}
bool deliver(int c){
  if(row!=6||col!=c||!carrying)return fail(F("DELIVERY STATE"));
  if(!goTo(7,c,false,true))return false;
  if(!advanceToPallet())return false;
  if(!liftTo(LIFT_DOWN))return false;
  carrying=false;
  Serial.println(F("UNLOAD COMMAND COMPLETE"));
#ifdef M2_ROUTE_TEST
  testWork(false,c);
#endif
  // Return to H, center there for the horizontal H2 -> H7 transfer.
  return returnToBoundary(true) && alignAndStop(alignMs());
}
bool enterStartH5(){
  if(!START_AT_EXTERNAL_T)return true;
#ifdef M2_ROUTE_TEST
  axisAligned=false;
  testNode(7,5);
  return alignAndStop(START_T_ALIGN_MS);
#else
  // Short T5 -> H5: no normal 190ms lockout (Mission1 start policy).
  unsigned long t=millis();
  bool armed=false;uint8_t clearCount=0,hits=0;
  previousError=0;
  while(millis()-t<1000UL){
    if(!safe())return false;
    int l=analogRead(PIN_IR_L),r=analogRead(PIN_IR_R);
    bool lb=l>=EMPTY_IR_THRESHOLD,rb=r>=EMPTY_IR_THRESHOLD;
    if(!(lb&&rb)){if(clearCount<3)++clearCount;if(clearCount>=3)armed=true;}
    else clearCount=0;
    if(armed && lb&&rb){
      if(++hits>=2){axisAligned=false;return alignAndStop(START_T_ALIGN_MS);}
    }else hits=0;
    int power=(lb!=rb)?EMPTY_RECOVERY_PWM:START_EDGE_PWM;
    if(lb&&rb)forward(power,power);else lineControl(power,l,r);
    delay(2);
  }
  return fail(F("START H5 TIMEOUT"));
#endif
}
bool mission2(){
  row=7;col=5;heading=0;openCol=0;leftOpenCol=0;rightOpenCol=0;
  carrying=false;aborted=false;axisAligned=true;stopMotors();
  running=true;missionStart=millis();
  if(!liftTo(LIFT_DOWN)||!enterStartH5()||!scanLeft())return false;
  if(TEST_SCAN_ONLY)return true;
  // E(selected) -> D -> C -> B; B6 -> A6 -> external pickup T1.
  if(!go(1,leftOpenCol)||!go(1,6)||!pickup(6))return false;
  // Return through the SAME selected left corridor; G2 -> H2 -> drop T1.
  if(!go(1,leftOpenCol)||!go(6,leftOpenCol)||!go(6,2)||!deliver(2))return false;
  if(!scanRight())return false;
  // E7 (or E8) -> B -> B4 -> A4 -> external pickup T2.
  if(!go(1,rightOpenCol)||!go(1,4)||!pickup(4))return false;
  // Same right corridor on return; G6 -> H6 -> drop T2.
  if(!go(1,rightOpenCol)||!go(6,rightOpenCol)||!go(6,6))return false;
  // Final unload: stop at the external T; do not leave after scoring.
  if(!goTo(7,6,false,true)||!advanceToPallet()||!liftTo(LIFT_DOWN))return false;
  carrying=false;
#ifdef M2_ROUTE_TEST
  testWork(false,6);
#endif
  Serial.println(F("UNLOAD2 COMPLETE / STOP AT T"));
  stopMotors();return true;
}
bool cardSeen(){
  if(!rfid.PICC_IsNewCardPresent()||!rfid.PICC_ReadCardSerial())return false;
  const bool RFID_UID_CONFIGURED = false; // Set true only after entering your own UID.
  const byte uid[]={0x00, 0x00, 0x00, 0x00}; // Placeholder, not an authorized card.
  bool ok=RFID_UID_CONFIGURED && rfid.uid.size==4;
  for(byte i=0;ok && i<4;++i)if(rfid.uid.uidByte[i]!=uid[i])ok=false;
  rfid.PICC_HaltA();rfid.PCD_StopCrypto1();return ok;
}
void setup(){
  pinMode(PIN_L_PWM,OUTPUT);pinMode(PIN_R_PWM,OUTPUT);
  pinMode(PIN_L_DIR,OUTPUT);pinMode(PIN_R_DIR,OUTPUT);
  pinMode(PIN_BUZZER,OUTPUT);
  stopMotors();Serial.begin(9600);
  lifter.attach(PIN_LIFT);
  liftAngle=LIFT_DOWN;
  lifter.write(LIFT_DOWN); // attach 후 DOWN 명령: 대기 및 탐색 중 리프트를 내린다.
  delay(LIFT_SETTLE_MS);
  SPI.begin();rfid.PCD_Init();
  Serial.println(F("M2 READY T5 -> H5 NORTH / LIFTER DOWN / RFID START"));
  Serial.println(F("FRONT_THRESHOLD: adjust near/far ADC separation"));
}
void loop(){
  if(!cardSeen()){delay(40);return;}
  tone(PIN_BUZZER,880,120);delay(200);noTone(PIN_BUZZER);
  bool ok=mission2();stopMotors();running=false;
  Serial.println(ok?F("M2 FINISHED - RESET"):F("M2 FAILED - RESET"));
  while(true)delay(50);
}

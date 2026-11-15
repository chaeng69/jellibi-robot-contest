// 2026-11-13 | 종목 1: 외부 T2 출발 -> 격자 주행 -> 외부 T6 최종 정지
// RFID UID is configured locally; no card is authorized by default.
// 시작: H2 아래 T자(T2), 로봇은 H2 방향을 향해 놓음.
// 종료: H6 아래 T자(T6), T6 검출 시 즉시 정지하고 2.3초 대기.
// 교차점 0=T2(이미 점유), 1=H2, ... 19=H6, 20=T6.
// 주의: T2->H2와 H6->T6는 일반 한 칸보다 짧으므로 별도 조건 사용.
// 현장 모터값: L182/R207, 회전 PWM127, L350ms/R360ms/U690ms.
// 직진 1000ms는 calibration 테스트 시간으로, 셀 주행시간이 아님.
// 스케치의 시간/센서 기준은 현장 주행으로 추가 검증 필요.
#ifndef HOST_SIM
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#endif

#define PIN_L_PWM 5
#define PIN_R_PWM 6
#define PIN_L_DIR 7
#define PIN_R_DIR 8
#define PIN_IR_L A6
#define PIN_IR_R A7
#define PIN_BUTTON A3
#define PIN_BUZZER 3
#define PIN_RFID_SS 2
#define PIN_RFID_RST 4
// After reading the card, allow a brief time to move your hand away.
#define RFID_START_DELAY_MS 200

#ifndef HOST_SIM
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
#endif

// ----- 현장 보정 변경 구역 -----
#ifndef SIM_CRUISE
#define SIM_CRUISE 207
#endif
#ifndef SIM_APPROACH
#define SIM_APPROACH 145
#endif
#ifndef SIM_KP
#define SIM_KP 40
#endif
#ifndef SIM_KD
#define SIM_KD 5
#endif
#ifndef SIM_SLOW_PCT
#define SIM_SLOW_PCT 70
#endif
#ifndef SIM_LAUNCH_MS
#define SIM_LAUNCH_MS 0
#endif
#ifndef SIM_LAUNCH_PWM
#define SIM_LAUNCH_PWM 100
#endif
#ifndef SIM_TRIM
#define SIM_TRIM 0
#endif
#ifndef SIM_TRIM_PWM
#define SIM_TRIM_PWM 70
#endif
#ifndef SIM_TRIM_MS
#define SIM_TRIM_MS 12
#endif
#ifndef SIM_RECOVERY_PWM
#define SIM_RECOVERY_PWM 140
#endif
#ifndef SIM_ALIGN_MS
#define SIM_ALIGN_MS 40
#endif
#ifndef SIM_EDGE_MS120
#define SIM_EDGE_MS120 1130  // 가정값. 센서 검출에 쓰는 거리값은 아님. 감속/안전 시간에만 사용
#endif

#define CRUISE_PWM SIM_CRUISE
#define APPROACH_PWM SIM_APPROACH
#define LT_KP SIM_KP
#define LT_KD SIM_KD
#define SLOW_START_PERCENT SIM_SLOW_PCT
#define ALIGN_MS SIM_ALIGN_MS
#define TRIM_ENABLED SIM_TRIM
#define TRIM_PWM SIM_TRIM_PWM
#define TRIM_MS SIM_TRIM_MS
#define RECOVERY_PWM SIM_RECOVERY_PWM
#define LAUNCH_MS SIM_LAUNCH_MS
#define LAUNCH_PWM SIM_LAUNCH_PWM
#define EDGE_MS120 SIM_EDGE_MS120

#define IR_WHITE 470
#define IR_BLACK 960
#define IR_BLACK_THRESHOLD 720
#define MOTOR_L_RATIO_NUM 182L
#define MOTOR_L_RATIO_DEN 207L
#define TURN_PWM 127
#define LEFT90_MS 350
#define RIGHT90_MS 360
#define LEFT180_MS 690
#define ALIGN_PWM 112
#define HOLD_MS 2300
// T2->H2, H6->T6: 짧은 외부 T자 연결 구간에만 적용.
#define START_EDGE_PWM 145
#define START_NODE_GAP_MS 0
#define START_EDGE_TIMEOUT_MS 1000UL
#define END_EDGE_PWM START_EDGE_PWM
#define END_NODE_GAP_MS 0
#define END_EDGE_TIMEOUT_MS 1000UL
#define MIN_NODE_GAP_MS 190 // 중복 교차점 계수 방지 (거리/속력 실측 없이도 안전한 재장전 시간의 시작값)
#define MAX_EDGE_TIME_PERCENT 175
#define INTERSECTION_CONFIRM 2
#define IR_CORR_MAX 75
#define MIN_PWM 50
#define NODE_SETTLE_MS 70
#define DEBUG_SERIAL 1
// 노드 인덱스: T2=0 / H2=1 / G2=2 / F2=3 / H6=19 / T6=20
// 전체 실행: 20. 앞에서 멈추려면 해당 번호로 변경.
#define TEST_STOP_AT_NODE 20

// T2 -> H2 -> ... -> H6 -> T6. 시작점 T2에서는 탐지를 수행하지 않음.
// 노드에서 해야 할 동작: S=직진, L=좌90, R=우90, U=반시계180,
// H=2.3초 정지 후 우90, F=마지막 2.3초 정지 후 완료.
// 해당 노드에 도착한 뒤의 행동임.
const char ROUTE[][3] = {
  {'T','2','S'}, // 0: 출발 T자, 이미 위치하고 있으므로 센서 탐지 생략
  {'H','2','S'}, {'G','2','S'}, {'F','2','R'}, {'F','3','S'}, {'F','4','S'},
  {'F','5','L'}, {'E','5','S'}, {'D','5','S'}, {'C','5','H'}, {'C','6','S'},
  {'C','7','L'}, {'B','7','U'}, {'C','7','S'}, {'D','7','S'}, {'E','7','S'},
  {'F','7','R'}, {'F','6','L'}, {'G','6','S'}, {'H','6','S'},
  {'T','6','F'}  // 20: H6 아래 외부 T자에서 최종 정지
};
#define N_ROUTE (sizeof(ROUTE) / sizeof(ROUTE[0]))

bool aborted=false;
bool running=false;
bool justTurned=false;
int previousError=0;

void rawMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(PIN_L_DIR, left >= 0 ? HIGH : LOW);
  digitalWrite(PIN_R_DIR, right >= 0 ? LOW : HIGH);
  analogWrite(PIN_L_PWM, abs(left));
  analogWrite(PIN_R_PWM, abs(right));
}
void driveForward(int left, int right) {
  // 전진만 182/207 비율 적용. 회전은 양쪽 모터 PWM127로 수행.
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  rawMotor((int)((long)left * MOTOR_L_RATIO_NUM / MOTOR_L_RATIO_DEN), right);
}
void stopMotors() { rawMotor(0,0); }

bool buttonDown() { return digitalRead(PIN_BUTTON) == LOW; }
bool checkAbort() {
  if (running && buttonDown()) {
    aborted=true;
    stopMotors();
  }
  return aborted;
}
bool waitSafe(unsigned long ms) {
  unsigned long start=millis();
  while(millis()-start < ms) {
    if(checkAbort()) return false;
    delay(5);
  }
  return true;
}

int blackness(int val) {
  long b=(long)(val-IR_WHITE)*1000L/(IR_BLACK-IR_WHITE);
  return constrain((int)b, 0, 1000);
}
void lineControl(int power,int irL,int irR) {
  const int err=blackness(irL)-blackness(irR);
  long correct=((long)LT_KP*err+(long)LT_KD*(err-previousError))/1000L;
  previousError=err;
  correct=constrain(correct, -(long)IR_CORR_MAX, (long)IR_CORR_MAX);
  driveForward(constrain(power-(int)correct, MIN_PWM, 255),
               constrain(power+(int)correct, MIN_PWM, 255));
}

// 반환 false: 검은 교차선 미검출/긴급정지. 시간은 '보조 감속과 타임아웃'으로만 사용함.
// 정상 검출 없이 200mm를 왔다고 간주하지 않음(이전 코드의 위험한 추정 완료 문제 수정).
bool seekNextIntersection(uint8_t nextIdx) {
  unsigned long t0=millis();
  unsigned long nominal=(unsigned long)EDGE_MS120*120UL/CRUISE_PWM;
  unsigned long slowAt=nominal*SLOW_START_PERCENT/100UL;
  const bool startEdge=(nextIdx==1); // T2 -> H2
  const bool endEdge=(nextIdx==N_ROUTE-1); // H6 -> T6
  unsigned long minAt=startEdge ? START_NODE_GAP_MS : (endEdge ? END_NODE_GAP_MS : MIN_NODE_GAP_MS);
  unsigned long timeout=startEdge ? START_EDGE_TIMEOUT_MS : (endEdge ? END_EDGE_TIMEOUT_MS : nominal*MAX_EDGE_TIME_PERCENT/100UL+160UL);
  bool armed=false;
  uint8_t clearCount=0, hitCount=0;
  previousError=0;
  while(millis()-t0 < timeout) {
    if(checkAbort()) return false;
    int il=analogRead(PIN_IR_L), ir=analogRead(PIN_IR_R);
    bool lBlack=il>=IR_BLACK_THRESHOLD, rBlack=ir>=IR_BLACK_THRESHOLD;
    // 흰색 하나만 나와도 재장전 가능. 연속 3회로 노이즈 방지.
    if(!(lBlack && rBlack)) { if(clearCount<3) ++clearCount; if(clearCount>=3) armed=true; }
    else clearCount=0;
    unsigned long elapsed=millis()-t0;
    if(armed && elapsed>=minAt && lBlack && rBlack) {
      if(++hitCount>=INTERSECTION_CONFIRM) {
#ifdef HOST_SIM
        simNodeDetected(nextIdx);
#endif
        justTurned=false;
        return true;
      }
    } else hitCount=0;
    int power=startEdge ? START_EDGE_PWM : (endEdge ? END_EDGE_PWM :
             ((justTurned && elapsed<LAUNCH_MS) ? LAUNCH_PWM : (elapsed>=slowAt ? APPROACH_PWM : CRUISE_PWM)));
    // 급격한 이탈 감지 시 일시 감속: 두 센서 중 한쪽만 검게 읽힐 때.
    if(lBlack != rBlack && power>RECOVERY_PWM) power=RECOVERY_PWM;
    if(lBlack && rBlack) driveForward(power,power);
    else lineControl(power,il,ir);
    delay(2);
  }
  stopMotors();
  return false;
}

bool alignWheelAxis(uint8_t nodeIdx) {
  driveForward(ALIGN_PWM,ALIGN_PWM);
  if(!waitSafe(ALIGN_MS)) return false;
  stopMotors();
  if(!waitSafe(NODE_SETTLE_MS)) return false;
#ifdef HOST_SIM
  simAligned(nodeIdx);
#endif
  return true;
}

// 회전 후 두 바닥 IR 센서 중 한쪽만 선을 크게 읽으면 작은 각도로 보정.
// 센서 두 개가 모두 흰색인 상황은 완전한 정렬 증거가 아니므로 보정은 제한적으로만 수행.
bool trimHeading() {
#if TRIM_ENABLED
  for(uint8_t j=0;j<8;++j) {
    if(checkAbort()) return false;
    int left=blackness(analogRead(PIN_IR_L));
    int right=blackness(analogRead(PIN_IR_R));
    int e=left-right;
    if(abs(e)<280) break;
    // 왼쪽이 검정 -> 반시계, 오른쪽이 검정 -> 시계
    rawMotor(e>0?-TRIM_PWM:TRIM_PWM, e>0?TRIM_PWM:-TRIM_PWM);
    if(!waitSafe(TRIM_MS)) return false;
    stopMotors();
    if(!waitSafe(28)) return false;
  }
#endif
  return true;
}

bool timedSpin(char action) {
  if(action=='S'||action=='F') return true;
  bool left=(action=='L'||action=='U');
  int duration=action=='L'?LEFT90_MS:(action=='R'?RIGHT90_MS:LEFT180_MS);
  // 좌: 왼쪽 후진/오른쪽 전진. 우: 왼쪽 전진/오른쪽 후진.
  rawMotor(left?-TURN_PWM:TURN_PWM, left?TURN_PWM:-TURN_PWM);
  if(!waitSafe(duration)) {stopMotors();return false;}
  stopMotors();
  if(!waitSafe(NODE_SETTLE_MS)) return false;
  if(!trimHeading()) return false;
  previousError=0;
  justTurned=true;
  return true;
}

void printNode(uint8_t i) {
#if DEBUG_SERIAL
  Serial.print(F("NODE ")); Serial.print((int)i);Serial.print(F(" "));
  Serial.print(ROUTE[i][0]);Serial.print(ROUTE[i][1]);
  Serial.print(F(" ACTION "));Serial.println(ROUTE[i][2]);
#endif
}

bool runMission1() {
  aborted=false;
  running=true;
  previousError=0;
  unsigned long startTime=millis();
#if DEBUG_SERIAL
  Serial.println(F("M1 START / FULL 8x8 GRID / STRICT SENSOR DETECTION"));
#endif
  // 로봇은 T2에 놓고 H2 방향으로 바라보게 함. 첫 감지 대상은 H2.
  // H6 아래에 있는 T6도 독립적으로 감지하여 정지한다.
  justTurned=false;
  if(checkAbort()) { running=false; return false; }
  printNode(0);
  for(uint8_t i=1;i<N_ROUTE;++i) {
    if(!seekNextIntersection(i)) {
#if DEBUG_SERIAL
      Serial.print(F("FAIL: intersection timeout at "));Serial.print(ROUTE[i][0]);Serial.println(ROUTE[i][1]);
#endif
      stopMotors();running=false;return false;
    }
    printNode(i);
    char action=ROUTE[i][2];
    if(action=='F') {
      // 바깥쪽 T자에는 여유거리가 없으므로 40ms 추가 전진 정렬을 하지 않음.
      // 센서가 T자를 확인하는 즉시 모터를 멈추고 2.3초 대기한다.
      stopMotors();
      if(!waitSafe(HOLD_MS)) {stopMotors();running=false;return false;}
      running=false;
#if DEBUG_SERIAL
      Serial.print(F("M1 DONE ms="));Serial.println(millis()-startTime);
#endif
      return true;
    }
    if(action!='S') {
      if(!alignWheelAxis(i)) {stopMotors();return false;}
      if(action=='H') {
        if(!waitSafe(HOLD_MS)) {stopMotors();return false;}
        action='R';
      }
      if(!timedSpin(action)){stopMotors();return false;}
#ifdef HOST_SIM
      simTurnFinished(i);
#endif
    }
    if (i >= TEST_STOP_AT_NODE && i < N_ROUTE - 1) {
      stopMotors(); running=false;
      Serial.print(F("PARTIAL TEST STOP at node ")); Serial.println((int)i);
      return true;
    }
  }
  stopMotors();running=false;return false;
}

// Only the locally configured 4-byte RFID card starts the mission.
// In HOST_SIM the card event is provided by the desktop simulator only.
bool rfidStartSeen() {
#ifdef HOST_SIM
  return simRFIDStartEvent();
#else
  if (!rfid.PICC_IsNewCardPresent()) return false;
  if (!rfid.PICC_ReadCardSerial()) return false;

// Card identifiers are not printed in the public version.

  // Configure your own 4-byte card locally; do not publish its UID.
  const bool RFID_UID_CONFIGURED = false; // Set true only after entering your own UID.
  const byte allowedUid[] = {0x00, 0x00, 0x00, 0x00}; // Placeholder, not an authorized card.
  bool allowed = RFID_UID_CONFIGURED && (rfid.uid.size == sizeof(allowedUid));
  if (allowed) {
    for (byte i = 0; i < sizeof(allowedUid); ++i) {
      if (rfid.uid.uidByte[i] != allowedUid[i]) {
        allowed = false;
        break;
      }
    }
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  if (!allowed) {
    Serial.println(F("UNAUTHORIZED CARD"));
  }
  return allowed;
#endif
}

void setup(){
  pinMode(PIN_L_PWM,OUTPUT);pinMode(PIN_R_PWM,OUTPUT);
  pinMode(PIN_L_DIR,OUTPUT);pinMode(PIN_R_DIR,OUTPUT);
  pinMode(PIN_BUTTON,INPUT_PULLUP);pinMode(PIN_BUZZER,OUTPUT);
  stopMotors();
  Serial.begin(9600);
#ifndef HOST_SIM
  SPI.begin();
  rfid.PCD_Init();
  delay(4);
#endif
  Serial.println(F("M1 READY: hold RFID card to start; A3 to EMERGENCY STOP."));
}

void loop(){
  stopMotors();
  if(!rfidStartSeen()) { delay(40); return; }

  Serial.println(F("RFID DETECTED. M1 STARTING"));
  tone(PIN_BUZZER, 880, 120);
  delay(RFID_START_DELAY_MS);
  noTone(PIN_BUZZER);
  if (buttonDown()) {
    stopMotors();
    Serial.println(F("CANCELLED before START by A3"));
#ifdef HOST_SIM
    return;
#else
    while(true) delay(50);
#endif
  }

  bool success=runMission1();
  running=false;
  stopMotors();
  if(success) Serial.println(F("FINISHED: reset to restart"));
  else Serial.println(F("STOPPED/FAILED: reset after checking position"));
#ifndef HOST_SIM
  while(true) delay(50); // Prevent a second start until manual reset.
#endif
}
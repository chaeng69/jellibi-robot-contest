/* Standalone motor/lift calibration for Arduino Nano.
 * Serial: 9600 baud, Newline or Both NL & CR.
 * No line following, intersection detection, RFID or automatic repeat.
 * PWM and milliseconds are commands, NOT measured speed/angle.
 * e = empty/down55, b = loaded/up170; manually position pallet before b.
 */
#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <ctype.h>

const uint8_t L_PWM=5, R_PWM=6, L_DIR=7, R_DIR=8, LIFT_PIN=9;
const int LIFT_DOWN=55, LIFT_UP=170;
const unsigned long LIFT_STEP_MS=20, LIFT_SETTLE_MS=500, MOTOR_SETTLE_MS=300;

// Initial trial values only. Replace with values measured using the NEW motors.
struct Profile {
  int leftPWM, rightPWM;
  unsigned int straightMs;
  int turnPWM;
  unsigned int leftMs, rightMs, uturnMs;
};
Profile emptyProfile={120,120,1000,120,340,340,660};
Profile loadedProfile={100,100,1000,100,465,470,890};

Servo lift;
bool loadedMode=false, liftReady=false;
uint8_t phase=0; // 0 idle, 1 drive, 2 lift movement, 3 settling
int liftAngle=LIFT_DOWN, liftTarget=LIFT_DOWN;
unsigned long phaseAt=0, durationMs=0, liftAt=0;
char inputLine[32];
uint8_t inputLength=0;
bool discardLine=false;

void motor(int left,int right){
  left=constrain(left,-255,255);right=constrain(right,-255,255);
  digitalWrite(L_DIR,left>=0?HIGH:LOW);
  digitalWrite(R_DIR,right>=0?LOW:HIGH);
  analogWrite(L_PWM,abs(left));analogWrite(R_PWM,abs(right));
}
void stopDrive(){motor(0,0);}
void showProfile(bool loaded){
  Profile &p=loaded?loadedProfile:emptyProfile;
  Serial.println(loaded?F("[LOADED]"):F("[EMPTY]"));
  Serial.print(F("L="));Serial.print(p.leftPWM);
  Serial.print(F(" R="));Serial.print(p.rightPWM);
  Serial.print(F(" T="));Serial.println(p.straightMs);
  Serial.print(F("P="));Serial.print(p.turnPWM);
  Serial.print(F(" A="));Serial.print(p.leftMs);
  Serial.print(F(" D="));Serial.print(p.rightMs);
  Serial.print(F(" U="));Serial.println(p.uturnMs);
}
void showAll(){
  showProfile(false);showProfile(true);
  Serial.print(F("Selected: "));Serial.println(loadedMode?F("LOADED"):F("EMPTY"));
  Serial.print(F("Lift command angle: "));Serial.println(liftAngle);
}
void help(){
  Serial.println(F("e=empty/down55, b=loaded/up170"));
  Serial.println(F("f=straight, a=left90, d=right90, u=left U-turn"));
  Serial.println(F("s=STOP immediately, ?=settings, h=help"));
  Serial.println(F("Set active profile: L 120 / R 120 / T 1000"));
  Serial.println(F("P 100=turn PWM; A 460 / D 460 / U 900=turn ms"));
  Serial.println(F("Settings are RAM only. Copy ? output before reset."));
}
void emergencyStop(){
  stopDrive();
  if(phase==2)liftReady=false; // position command stopped partway
  phase=0;
  inputLength=0;
  Serial.println(F("STOP. Servo holds last command; e/b to finish interrupted lift."));
}
void selectMode(bool loaded){
  stopDrive();loadedMode=loaded;liftTarget=loaded?LIFT_UP:LIFT_DOWN;
  liftReady=false;phase=2;liftAt=millis();
  Serial.println(loaded?F("LIFT UP command 170"):F("LIFT DOWN command 55"));
}
void startDrive(char command){
  if(!liftReady){Serial.println(F("Select e or b and wait READY first."));return;}
  Profile &p=loadedMode?loadedProfile:emptyProfile;
  int left=0,right=0;unsigned int ms=0;
  if(command=='f'){left=p.leftPWM;right=p.rightPWM;ms=p.straightMs;}
  if(command=='a'){left=-p.turnPWM;right=p.turnPWM;ms=p.leftMs;}
  if(command=='d'){left=p.turnPWM;right=-p.turnPWM;ms=p.rightMs;}
  if(command=='u'){left=-p.turnPWM;right=p.turnPWM;ms=p.uturnMs;}
  Serial.print(F("RUN "));Serial.print(command);
  Serial.print(F(" L="));Serial.print(left);Serial.print(F(" R="));Serial.print(right);
  Serial.print(F(" ms="));Serial.println(ms);
  durationMs=ms;phaseAt=millis();phase=1;motor(left,right);
}
void commandLine(char *line){
  while(isspace((unsigned char)*line))++line;
  if(!*line)return;
  char c=*line++;
  while(isspace((unsigned char)*line))++line;
  if(phase!=0){Serial.println(F("BUSY: only s accepted. Wait READY."));return;}
  if(!*line){
    if(c=='e'){selectMode(false);return;}
    if(c=='b'){selectMode(true);return;}
    if(c=='f'||c=='a'||c=='d'||c=='u'){startDrive(c);return;}
    if(c=='?'){showAll();return;}
    if(c=='h'){help();return;}
  }
  if(c!='L'&&c!='R'&&c!='T'&&c!='P'&&c!='A'&&c!='D'&&c!='U'){
    Serial.println(F("Unknown command; h for help."));return;
  }
  char *end;long value=strtol(line,&end,10);
  bool hasNumber=(end!=line);
  while(isspace((unsigned char)*end))++end;
  bool pwm=(c=='L'||c=='R'||c=='P');
  if(!hasNumber||*end||value<(pwm?0:10)||value>(pwm?255:5000)){
    Serial.println(F("Invalid value: PWM 0..255, time 10..5000ms."));return;
  }
  Profile &p=loadedMode?loadedProfile:emptyProfile;
  if(c=='L')p.leftPWM=value;
  if(c=='R')p.rightPWM=value;
  if(c=='T')p.straightMs=value;
  if(c=='P')p.turnPWM=value;
  if(c=='A')p.leftMs=value;
  if(c=='D')p.rightMs=value;
  if(c=='U')p.uturnMs=value;
  showProfile(loadedMode);
}
void updateAction(){
  unsigned long now=millis();
  if(phase==1 && now-phaseAt>=durationMs){
    stopDrive();phase=3;phaseAt=now;durationMs=MOTOR_SETTLE_MS;
    Serial.println(F("MOTOR STOP"));
  }else if(phase==2){
    if(liftAngle==liftTarget){
      liftReady=true;phase=3;phaseAt=now;durationMs=LIFT_SETTLE_MS;
      Serial.println(F("Lift command complete (not position feedback)."));
    }else if(now-liftAt>=LIFT_STEP_MS){
      liftAt=now;liftAngle+=(liftTarget>liftAngle)?1:-1;lift.write(liftAngle);
    }
  }else if(phase==3 && now-phaseAt>=durationMs){
    phase=0;Serial.println(F("READY"));
  }
}
void setup(){
  pinMode(L_PWM,OUTPUT);pinMode(R_PWM,OUTPUT);
  pinMode(L_DIR,OUTPUT);pinMode(R_DIR,OUTPUT);stopDrive();
  Serial.begin(9600);lift.attach(LIFT_PIN);lift.write(LIFT_DOWN);
  phase=3;phaseAt=millis();durationMs=LIFT_SETTLE_MS;liftReady=true;
  Serial.println(F("CALIBRATION ONLY: no auto motion. Lift starts at55."));
  help();showAll();
}
void loop(){
  updateAction();
  while(Serial.available()){
    char c=Serial.read();
    if(c=='s'||c=='S'){
      emergencyStop();discardLine=true;continue;
    }
    if(c=='\n'||c=='\r'){
      if(!discardLine && inputLength){inputLine[inputLength]='\0';commandLine(inputLine);}
      inputLength=0;discardLine=false;
    }else if(!discardLine){
      if(inputLength<sizeof(inputLine)-1)inputLine[inputLength++]=c;
      else{inputLength=0;discardLine=true;Serial.println(F("Line too long."));}
    }
    updateAction();
  }
}

// Exterminate - RC drive + weapon, with Pi fallback.
// Same wiring as face_drive.ino: TB6612 PWMA D5 AIN1 D7 AIN2 D8, PWMB D6 BIN1 D9 BIN2 D10, STBY D4.
// HotRC F-06A: CH1 steer A1, CH2 throttle A2, CH3 weapon button A3. Weapon ESC on D3.
// RC has priority. When the remote is off, the Pi's "M l r" / "S" serial commands still work.
#include <Servo.h>

const uint8_t PIN_STBY=4, PIN_PWMA=5, PIN_AIN1=7, PIN_AIN2=8, PIN_PWMB=6, PIN_BIN1=9, PIN_BIN2=10;
const uint8_t PIN_ESC=3;
const bool INVERT_LEFT=false, INVERT_RIGHT=false, INVERT_STEER=false, INVERT_THR=false;
const int  DEADBAND=12;          // percent of stick
const int  WEAPON_PCT=30;        // spin speed when CH3 on (raise once happy)
const unsigned long RC_LOST_MS=200, PI_TIMEOUT_MS=300;

Servo esc;
volatile uint16_t rcUs[3]={0,0,0};
volatile unsigned long rcRise[3], rcLast[3];
volatile uint8_t lastPinc;

ISR(PCINT1_vect){            // A1..A3 = PC1..PC3
  uint8_t now=PINC; uint8_t ch=now^lastPinc; lastPinc=now;
  unsigned long t=micros();
  for(uint8_t i=0;i<3;i++){ uint8_t m=1<<(i+1);
    if(ch&m){ if(now&m) rcRise[i]=t; else { uint16_t w=t-rcRise[i]; if(w>800&&w<2200){rcUs[i]=w; rcLast[i]=millis();} } } }
}

void chan(uint8_t pwm,uint8_t a,uint8_t b,int s,bool inv){
  if(inv)s=-s;
  digitalWrite(a,s>0); digitalWrite(b,s<0); analogWrite(pwm,abs(s));
}
void setMotors(int l,int r){
  l=constrain(l,-255,255); r=constrain(r,-255,255);
  chan(PIN_PWMA,PIN_AIN1,PIN_AIN2,l,INVERT_LEFT);
  chan(PIN_PWMB,PIN_BIN1,PIN_BIN2,r,INVERT_RIGHT);
  digitalWrite(PIN_STBY,(l||r)?HIGH:LOW);
}
int stickPct(uint16_t us){ int p=((int)us-1500)/5; if(abs(p)<DEADBAND) return 0; return constrain(p,-100,100); }

char buf[32]; uint8_t len=0; int piL=0,piR=0; unsigned long piLast=0;
void pollSerial(){
  while(Serial.available()){ char c=Serial.read(); if(c=='\r')continue;
    if(c=='\n'){ buf[len]=0; len=0; char*p=buf; while(*p==' ')p++;
      if(*p=='M'||*p=='m'){ char*e; long l=strtol(p+1,&e,10); long r=strtol(e,&e,10); piL=l; piR=r; piLast=millis(); Serial.println(F("OK")); }
      else if(*p=='S'||*p=='s'){ piL=piR=0; piLast=millis(); Serial.println(F("OK")); }
      else if(*p=='P'||*p=='p'){ piLast=millis(); Serial.println(F("PONG")); }
    } else if(len<31) buf[len++]=c; }
}

int escUs=1500;  // bidirectional ESC: 1500 = stop
void setup(){
  pinMode(PIN_STBY,OUTPUT); digitalWrite(PIN_STBY,LOW);
  uint8_t outs[]={PIN_PWMA,PIN_PWMB,PIN_AIN1,PIN_AIN2,PIN_BIN1,PIN_BIN2};
  for(uint8_t p:outs){pinMode(p,OUTPUT);digitalWrite(p,LOW);}
  esc.attach(PIN_ESC); esc.writeMicroseconds(1500);
  pinMode(A1,INPUT); pinMode(A2,INPUT); pinMode(A3,INPUT);
  lastPinc=PINC; PCMSK1|=0b00001110; PCICR|=bit(PCIE1);
  Serial.begin(115200); Serial.println(F("READY RC"));
}

unsigned long lastStat=0;
void loop(){
  pollSerial();
  unsigned long now=millis();
  uint16_t s,t,w; unsigned long l1,l2,l3;
  noInterrupts(); s=rcUs[0]; t=rcUs[1]; w=rcUs[2]; l1=rcLast[0]; l2=rcLast[1]; l3=rcLast[2]; interrupts();
  bool rcOk = s && t && now-l1<RC_LOST_MS && now-l2<RC_LOST_MS;
  int L=0,R=0;
  int st=rcOk?stickPct(s):0, th=rcOk?stickPct(t):0;
  if(INVERT_STEER) st=-st; if(INVERT_THR) th=-th;
  if(st||th){            // remote sticks moved: remote wins
    L=constrain(th+st,-100,100)*255/100; R=constrain(th-st,-100,100)*255/100;
  } else if(now-piLast<PI_TIMEOUT_MS){ L=piL; R=piR; }   // sticks centered: Pi drives
  setMotors(L,R);
  // Weapon: only with live RC and CH3 high; ramps up, cuts instantly.
  bool weaponOn = rcOk && w>1600 && now-l3<RC_LOST_MS;
  int target = weaponOn ? 1500+WEAPON_PCT*5 : 1500;
  if(target<escUs) escUs=target; else if(escUs<target) escUs+=1;
  esc.writeMicroseconds(escUs);
  if(now-lastStat>500){ lastStat=now;
    Serial.print(F("RC=")); Serial.print(rcOk); Serial.print(F(" CH1=")); Serial.print(s); Serial.print(F(" CH2=")); Serial.print(t);
    Serial.print(F(" CH3=")); Serial.print(w); Serial.print(F(" L=")); Serial.print(L); Serial.print(F(" R=")); Serial.print(R); Serial.print(F(" ESC=")); Serial.println(escUs); }
  delay(2);
}

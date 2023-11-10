#include <util/atomic.h>
#include <chrono>
#include <iostream>

struct Encoder {
  volatile int pos; //ISR stable counter
  int EA;           //hall affect pin A
  int EB;           //hall affect pin B
  };

class Motor {
  public:
    Encoder encoder;

    double prevTime; // Last time update was called
    double curTime;

    int desPos;
    int curPos;     //stable position variable -> updated from volitile encoder pos
    int dirPin;
    int pwmPin;
    int posHigh;
    bool pwmInvert;

    double pwmVal;

    double kp;
    double ki;
    double kd;
    double PIDlimit;
    int PIDrate;

    double Ihist = 0.0;
    double error = 0.0;
    double prevErr = 0.0;

    double policy;  //PID output
    int mCommand;   //pwm mapped PID output

  void computeError() {
    //Compute error
    error = desPos - curPos;
  }

  double proportion() {
    //compute propportional term
    double P = kp * error;
    return P;
  }

  double integral(double dt) {
    //compute integral term
    if (-PIDlimit < policy < PIDlimit) {
      Ihist += error * dt;
    }
    double I = constrain(ki * Ihist, -900, 900);
    return I;
  }

  double derivative(double dt) {
    //compute derivative term
    double dx = (error - prevErr)/dt;
    double D = kd * dx;
    return D;
  }

  void remapPID(double PIDout, double PIDmin, double PIDmax) {
    int pwmCommand = (int)(255.0 * ((policy) / (PIDmax)));
    mCommand = constrain(abs(pwmCommand), 0, 255);
    if (curPos < desPos) {
      if (pwmInvert) {
        mCommand = 255 - mCommand;
      }
    } else {
      if (!pwmInvert) {
        mCommand = 255 - mCommand;
      }
    }

    /*Serial.print("pos: ");
    Serial.print(curPos);
    Serial.print("  policy: ");
    Serial.print(policy);
    Serial.print("  pwmCmd: ");
    Serial.print(pwmCommand);
    Serial.print("  mCommand: ");
    Serial.print(mCommand);*/
  }

  void PID_controller() {
    //update feedback policy term

    curTime = millis();                   //get current time
    double dt = (curTime - prevTime)/1000;  //compute change in time since last call

    computeError(); 

    //compute policy terms
    double P = proportion();    //compute proportinal term
    double I = integral(dt);    //compute Integral term
    if ((abs(error)*2 + abs(prevErr)*2) < 1) {
      Ihist = 0;
    }
    double D = derivative(dt);  //compute derivative

    //update params
    prevErr = error;
    prevTime = curTime;

    //update policy
    policy = constrain(P + I + D, -PIDlimit, PIDlimit);
    remapPID(policy, -PIDlimit, PIDlimit);

    /*Serial.print(" P: ");
    Serial.print(P);
    Serial.print(" I: ");
    Serial.print(I);
    Serial.print(" D: ");
    Serial.println(D);*/

  }

  void command(int pos) {
    desPos = pos;
    PID_controller();
    if (policy < 0) {
      if (posHigh) {
        digitalWrite(dirPin, LOW);
      } else {
        digitalWrite(dirPin, HIGH);
      }
    } else {
      if (posHigh) {
        digitalWrite(dirPin, HIGH);
      } else {
        digitalWrite(dirPin, LOW);
      }
    }
    analogWrite(pwmPin, mCommand);
  }

};

// define all encoder pins
#define ENCA1 15 // Yellow
#define ENCB1 14 // White
#define ENCA2 21 // Yellow
#define ENCB2 20 // White
#define ENCA3 23 // Yellow
#define ENCB3 22 // White
#define ENCA4 17 // Yellow
#define ENCB4 16 // White

//initialize motors
Motor M1;
Motor M2;
Motor M3;
Motor M4;

void initEncoders(){
  pinMode(M1.encoder.EA, INPUT);
  pinMode(M1.encoder.EB, INPUT);
  pinMode(M2.encoder.EA, INPUT);
  pinMode(M2.encoder.EB, INPUT);
  pinMode(M3.encoder.EA, INPUT);
  pinMode(M3.encoder.EB, INPUT);
  pinMode(M4.encoder.EA, INPUT);
  pinMode(M4.encoder.EB, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1.encoder.EA),ISR_readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(M2.encoder.EA),ISR_readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(M3.encoder.EA),ISR_readEncoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(M4.encoder.EA),ISR_readEncoder4,RISING);
}

// interrupt service routine for encoder updates - Note: you can not pass variables to ISR because it is hardware driven, 
//                                                       you must hard code the check for each encoder 
void ISR_readEncoder1(){
  if (digitalReadFast(M1.encoder.EB)) { M1.encoder.pos --; }   //hand selected ++ vs -- based on mechanical configuration of encoder, ++ = spooling up
  else { M1.encoder.pos ++; }
  }
void ISR_readEncoder2(){
  if (digitalReadFast(M2.encoder.EB)) { M2.encoder.pos ++; }   //hand selected ++ vs -- based on mechanical configuration of encoder, ++ = spooling up
  else { M2.encoder.pos --; }
  }
void ISR_readEncoder3(){
  if (digitalReadFast(M3.encoder.EB)) { M3.encoder.pos --; }   //hand selected ++ vs -- based on mechanical configuration of encoder, ++ = spooling up
  else { M3.encoder.pos ++; }
  }
void ISR_readEncoder4(){
  if (digitalReadFast(M4.encoder.EB)) { M4.encoder.pos ++; }   //hand selected ++ vs -- based on mechanical configuration of encoder, ++ = spooling up
  else { M4.encoder.pos --; }
  }

unsigned long PID_prevTime;
unsigned long startTime;
int counter;
int stepRate = 10;

void setup() {
  // put your setup code here, to run once:
  M1.encoder = {0, ENCA1, ENCB1};
  M1.kp = 50;
  M1.kd = 8;
  M1.ki = 3;
  M1.PIDlimit = 1000;
  M1.PIDrate = 5; //50 ms refresh, or 
  M1.policy = 0;
  M1.pwmInvert = true;
  M1.posHigh = true;
  M1.pwmPin = 11; 
  M1.dirPin = 32;
  pinMode(M1.dirPin, INPUT);
  pinMode(M1.pwmPin, INPUT);

  M2.encoder = {0, ENCA2, ENCB2};
  M2.kp = 50;
  M2.kd = 8;
  M2.ki = 3;
  M2.PIDlimit = 1000;
  M2.PIDrate = 5; //50 ms refresh, or 
  M2.policy = 0;
  M2.pwmInvert = false;
  M2.posHigh = false;
  M2.pwmPin = 9; 
  M2.dirPin = 30;
  pinMode(M2.dirPin, INPUT);
  pinMode(M2.pwmPin, INPUT);

  M3.encoder = {0, ENCA3, ENCB3};
  M3.kp = 50;
  M3.kd = 8;
  M3.ki = 3;
  M3.PIDlimit = 1000;
  M3.PIDrate = 5; //50 ms refresh, or 
  M3.policy = 0;
  M3.pwmInvert = false;
  M3.posHigh = false;
  M3.pwmPin = 10; 
  M3.dirPin = 31;
  pinMode(M3.dirPin, INPUT);
  pinMode(M3.pwmPin, INPUT);

  M4.encoder = {0, ENCA4, ENCB4};
  M4.kp = 50;
  M4.kd = 8;
  M4.ki = 3;
  M4.PIDlimit = 1000;
  M4.PIDrate = 5; //50 ms refresh, or 
  M4.policy = 0;
  M4.pwmInvert = true;
  M4.posHigh = true;
  M4.pwmPin = 12; 
  M4.dirPin = 33;
  pinMode(M4.dirPin, INPUT);
  pinMode(M4.pwmPin, INPUT);

  initEncoders();
  Serial.begin(9600);
  PID_prevTime = millis();
  startTime = millis();

}

void loop() {
  // update all positions -- This atomic allows you to create stable variables when ISR funcs cutoff mid update
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    M1.curPos = M1.encoder.pos;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    M2.curPos = M2.encoder.pos;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    M3.curPos = M3.encoder.pos;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    M4.curPos = M4.encoder.pos;
  }


  if (millis() - PID_prevTime > M4.PIDrate) {
    //M1.command(100);
    //M2.command(100);
    //M4.command(100);
    //M3.command(100);
    PID_prevTime = millis();
    
  } 

}

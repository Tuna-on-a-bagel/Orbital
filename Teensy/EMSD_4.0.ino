#include <util/atomic.h>
#include "motor.h"
#include "comm.h"
#include "positions.h"

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

//init communication bus
Comm bus;

//init global structs
Positions curPos;
Positions desPos;

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

  M1.encoder = {0, ENCA1, ENCB1};
  M1.kp = 50;
  M1.kd = 8;
  M1.ki = 1;
  M1.pwmInvert = true;
  M1.posHigh = true;
  M1.pwmPin = 11; 
  M1.dirPin = 32;
  pinMode(M1.dirPin, INPUT);
  pinMode(M1.pwmPin, INPUT);

  M2.encoder = {0, ENCA2, ENCB2};
  M2.kp = 50;
  M2.kd = 8;
  M2.ki = 1;
  M2.pwmInvert = false;
  M2.posHigh = false;
  M2.pwmPin = 9; 
  M2.dirPin = 30;
  pinMode(M2.dirPin, INPUT);
  pinMode(M2.pwmPin, INPUT);

  M3.encoder = {0, ENCA3, ENCB3};
  M3.kp = 50;
  M3.kd = 8;
  M3.ki = 1;
  M3.pwmInvert = false;
  M3.posHigh = false;
  M3.pwmPin = 10; 
  M3.dirPin = 31;
  pinMode(M3.dirPin, INPUT);
  pinMode(M3.pwmPin, INPUT);

  M4.encoder = {0, ENCA4, ENCB4};
  M4.kp = 50;
  M4.kd = 8;
  M4.ki = 1;
  M4.pwmInvert = true;
  M4.posHigh = true;
  M4.pwmPin = 12; 
  M4.dirPin = 33;
  pinMode(M4.dirPin, INPUT);
  pinMode(M4.pwmPin, INPUT);

  initEncoders();
  Serial1.begin(500000);
  
  PID_prevTime = millis();
  startTime = millis();

  curPos = {0, 0, 0, 0};
  desPos = {0, 0, 0, 0};
}

void loop() {
  // update all positions -- This atomic allows you to create stable variables when ISR funcs cutoff mid update

  int pos1;
  int pos2;
  int pos3;
  int pos4;

  //Serial.println("in loop");
  
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
  ///*
  // update struct and Tx position data to Rpi
  if ((millis() - bus.prevTxTime) > bus.TxRx_rate) {
    curPos = {M1.curPos, M2.curPos, M3.curPos, M4.curPos};
    bus.sendComm(curPos);
    bus.prevTxTime = millis();
  }
  
  // Rx position commands from Rpi
  if ((millis() - bus.prevRxTime) > bus.TxRx_rate) {
    Positions newDesPos;
    bool rx = bus.getComm(newDesPos);
    bus.prevRxTime = millis();
    //delay(1);
    if (rx) {
      desPos = newDesPos;
      //Serial.println("Valid data received:");
      Serial.println(desPos.pos1);
      M1.command(desPos.pos1);
      M2.command(desPos.pos2);
      M3.command(desPos.pos3);
      M4.command(desPos.pos4);
    } else {
      //Serial.println("Invalid data received, keeping previous positions.");
    }
  }//*/
  
  
  

  if (millis() - PID_prevTime > M4.PIDrate) {
    //M1.command(100);
    //M2.command(100);
    //M4.command(100);
    //M3.command(100);
    PID_prevTime = millis();
    
  } 

}
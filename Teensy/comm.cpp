#include "comm.h"
#include <Arduino.h>

Comm::Comm() {
  //Serial1.begin(750000);
  TxRx_rate = 1; //ms waiting for data transmission
  prevTxTime = 0.0;
  prevRxTime = 0.0;
  }

void Comm::serializeData(const Positions& data, uint8_t* buffer){
  memcpy(buffer, &data, sizeof(Positions));
  } 

void Comm::deserializeData(uint8_t* buffer, Positions& data) {
  memcpy(&data, buffer, sizeof(Positions));
  }

bool Comm::sync() {
  if (Serial1.available() == 0) {
    return false;
  } else {
    while (Serial1.available() > 0) {
      if (Serial1.read() == 'R') {
        //sCount ++;
        return true;
      }
      //Serial.println("Synced!");
    }  
    //Serial.println("syncFailed");
    return false;
    }
  }

bool Comm::verify() {
  if (Serial1.available() == 0) {
    //Serial.println("no data on verify");
    return false;
  } else {
    while (Serial1.available() > 0) {
      if (Serial1.read() == 'r') {
        //vCount ++;
        return true;
      }
      //Serial.println("Synced!");
    }  
    //Serial.println("verify failed");
    return false;
    }
  }

void Comm::sendComm(const Positions& data){
  Serial1.write('T');                               //Synchronization start byte
  uint8_t buffer[sizeof(Positions)];
  serializeData(data, buffer);
  for (size_t i = 0; i < sizeof(Positions); i++){
    Serial1.write(buffer[i]);
    }
  Serial1.write('t');                               //Synchronization end byte
  }

bool Comm::getComm(Positions& desiredPositions) {
  
  uint8_t buffer[sizeof(Positions)];
  //Serial.println("into sync");
  if (!sync()) { // Verify correct init byte
    //Serial.println("Sync failed.");
    return false;
  }
  
  for (size_t i = 0; i < sizeof(Positions); i++) {
    while (Serial1.available() < 1) {}// Wait for the next byte
    buffer[i] = Serial1.read();
  }
 
  if (!verify()) { // Verify correct terminal byte
    //Serial.println("Verify failed.");
    //errorCount ++;
    return false;
  }

  deserializeData(buffer, desiredPositions);
  //Serial.println("Exiting getComm with good data");
  return true;
  }
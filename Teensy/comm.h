// comm.h
#ifndef COMM_H
#define COMM_H
#include "positions.h"
#include <Arduino.h>
#include <cstdint>

class Comm {
  public:
    Comm();

    //Serial1.begin(750000);

    bool getComm(Positions& desiredPositions);
    void sendComm(const Positions& data);
    bool sync();
    bool verify();
    void serializeData(const Positions& data, uint8_t* buffer);
    void deserializeData(uint8_t* buffer, Positions& data);

    unsigned long TxRx_rate;
    unsigned long prevTxTime;
    unsigned long prevRxTime;

};

#endif
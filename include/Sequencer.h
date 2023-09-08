#pragma once
#include <Arduino.h>


enum {SEQ_STATE_IDLE = 0, SEQ_STATE_START_POLL_CYCLE};


class Sequencer {

private:
    uint8_t keypadPacketSeqNumBits;
    uint8_t state;
    uint32_t rxParityErrors;
    uint32_t rxChecksumErrors;
    uint32_t rxTimeoutErrors;
    uint32_t pollInactiveTime;
    uint32_t pollStartTime;


public:
    void setup();

    void update();


};
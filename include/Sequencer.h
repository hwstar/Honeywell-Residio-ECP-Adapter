#pragma once
#include <Arduino.h>
#include <EcpSoftwareSerial.h>

#define SEQ_READ_TIMEOUT_TIME_MS 100

enum {SEQ_STATE_IDLE = 0, SEQ_STATE_WAIT_POLL_CYCLE, SEQ_STATE_WAIT_BEFORE_F6, 
SEQ_STATE_WAIT_F6_TIMER, SEQ_STATE_WAIT_KEYPAD_RESPONSE, SEQ_STATE_WAIT_ACK_COMMAND, 
SEQ_STATE_RESET_AND_START_OVER};

typedef void (*keypad_callback)(uint8_t *, uint8_t);

class Sequencer {

private:
    EcpSoftwareSerial *pEcp;

    bool validPacket;

    uint8_t addressAndPacketSequenceNumber;
    uint8_t state;
    uint8_t pollByteCount;
    uint8_t keypadAddress;
    uint8_t pollBuffer[3];
    uint8_t packetLength;
    uint8_t packet[64];

    uint32_t rxParityErrors;
    uint32_t rxChecksumErrors;
    uint32_t rxTimeoutErrors;
    uint32_t pollInactiveTime;
    uint32_t readStartTime;
    uint32_t pollWaitBeforeF6Time;

    keypad_callback pCallback;
   
    uint8_t readBytes(uint8_t *buffer, uint8_t byte_count);


public:
    void begin(EcpSoftwareSerial *ecp_obj, void (*keypad_action)(uint8_t *buffer, uint8_t length) = NULL);

    void update();


};
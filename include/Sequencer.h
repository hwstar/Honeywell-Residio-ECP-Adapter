#pragma once
#include <Arduino.h>
#include <EcpSoftwareSerial.h>

#define SEQ_READ_TIMEOUT_TIME_MS 100

#define KEYPAD_DISPLAY_LINE_SIZE 16
#define DISPLAY_PACKET_SIZE_F7 48
#define PACKET_BUFFER_SIZE 64
#define MAX_CODE_LENGTH 4


enum {CHIME_NONE=0, CHIME_ONCE, CHIME_TWICE, CHIME_THREE_TIMES, CHIME_FAST_REPEATING, CHIME_SLOW_REPEATING, CHIME_UNUSED, CHIME_LOUD};

enum {SEQ_STATE_IDLE = 0, SEQ_STATE_WAIT_POLL_CYCLE, SEQ_STATE_WAIT_BEFORE_F6, 
SEQ_STATE_WAIT_F6_TIMER, SEQ_STATE_WAIT_KEYPAD_RESPONSE, SEQ_STATE_WAIT_ACK_COMMAND, 
SEQ_STATE_FINISH_UP, SEQ_WAIT_BEFORE_F7, SEQ_WAIT_INIT_COMMAND, SEQ_WAIT_FOR_F7_SENT,
SEQ_STATE_WAIT_NEXT_POLL_TIME};

enum {CODE_STATE_IDLE, CODE_STATE_INTERDIGIT};


typedef void (*keypad_callback)(uint8_t *, uint8_t);


typedef struct Packet_F7 {
        uint8_t f7; // Holds the F7 pattern
        uint8_t uf1;
        uint8_t uf2;
        uint8_t keypadAddress;
        uint8_t uf4; 
        uint8_t zone;
        uint8_t chime;
        uint8_t readyArmedStay;
        uint8_t statusBits;
        uint8_t progMode;
        uint8_t promptPosition;
        uint8_t uf11;
        uint8_t lcdLine1[KEYPAD_DISPLAY_LINE_SIZE];
        uint8_t lcdLine2[KEYPAD_DISPLAY_LINE_SIZE];
        uint8_t checksum;
        uint8_t pad1;
        uint8_t pad2;
        uint8_t pad3;
} __attribute__((aligned(1))) Packet_F7; // Can get away with non-aligned structure members on an Arm cortex M3.

typedef struct Code_Info {
    uint32_t timer;
    uint32_t keypadAddress;
    uint32_t state;
    uint8_t length;
    uint8_t buffer[MAX_CODE_LENGTH];
} Code_Info;


class Sequencer {

private:
    EcpSoftwareSerial *pEcp;

    bool validPacket;
    bool displayUpdateBusy;

    uint8_t addressAndPacketSequenceNumber;
    uint8_t state;
    uint8_t pollByteCount;
    uint8_t keypadAddress;
    uint8_t pollBuffer[3];
    uint8_t packetLength;
    uint8_t index_f7;
    uint8_t packet[PACKET_BUFFER_SIZE];
    uint8_t displayPacketF7[DISPLAY_PACKET_SIZE_F7];
    uint32_t rxParityErrors;
    uint32_t rxChecksumErrors;
    uint32_t rxTimeoutErrors;
    uint32_t pollInactiveTime;
    uint32_t readStartTime;
    uint32_t pollWaitTime;

    Packet_F7 f7;

    Code_Info code_data[8];

    keypad_callback pCallback;
   
    




    uint8_t readBytes(uint8_t *buffer, uint8_t byte_count);


public:

    // Initializes the sequencer

    void begin(EcpSoftwareSerial *ecp_obj, void (*keypad_action)(uint8_t *buffer, uint8_t length) = NULL);

    // Must be called periodically to update the sequencer.

    void update();

    void formatDisplayPacket(void *p);
    
    void setKeypadAddressBits(void *dp, uint8_t address_bits);

    void setChimeMode(void *dp, uint8_t mode);

    void setArmedStay(void *dp, bool state);

    void setReady(void *dp, bool state);

    void setChimeFlag(void *dp, bool state);

    void setAcPowerFlag(void *dp, bool state);

    void setArmedAway(void *dp, bool state);

    void setLcdBackLight(void *dp, bool state);

    void setLCDLine1(void *dp, const char *line, uint8_t length);

    void setLCDLine2(void *dp, const char *line, uint8_t length);

    bool submitDisplayPacket(void *dp);

    bool getDisplayUpdateBusy();

    void getParityErrorCount(bool reset = false);

    void getChecksumErrorCount(bool reset = false);
    
    void getTimeOutErrorCount(bool reset = false);

    
};
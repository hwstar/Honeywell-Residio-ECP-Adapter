#include <Arduino.h>
#include <common.h>
#include <Sequencer.h>
#pragma once

#define RAW_PACKET_BUFFER_SIZE 64
#define TX_DATA_PACKET_POOL_SIZE 16
#define STUFF_CODE 0x00
#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define STUFFED_BYTE_THRESHOLD 0x04
#define PT_ACK 0x00
#define PT_NAK 0x01
#define PT_DATA_SHORT 0x02
#define CRC_INIT_VEC 0x55AA
#define PANEL_MAX_RETRIES 5
#define PACKET_TX_TIMEOUT_MS 2000
#define RX_FRAME_TIMEOUT_MS 2000
#define INIT_MESSAGE_DELAY_MS 10000

enum {PSF_CLEAR = 0x00, PSF_RX_ACK = 0x01, PSF_RX_NAK = 0x02, PSF_RX_DATA = 0x04, PSF_BAD_PACKET = 0x08, PSF_TX_BUSY = 0x80, PSF_RX_FLAGS = 0x0F};
enum {RX_GOT_NOTHING = 0, RX_GOT_STX, RX_GOT_ETX, RX_GOT_DATA};
enum {PRX_STATE_INIT = 0, PRX_STATE_IDLE, PRX_TX, PRX_TX_WAIT_ACK};
enum {RF_STATE_IDLE = 0, RF_WAIT_DATA_ETX, RF_WAIT_CLEAR_FLAGS};
enum {SRX_STATE_IDLE = 0, SRX_STATE_WAIT_SECOND};


class Panel {
private:
    HardwareSerial *_uart;
    ErrorCounters _ec;
    PanelPacketAckNak _txAckNakPacket;
    Packet_F7 _f7;
    bool _helloReceived;
    bool _initMessageSent;
    uint8_t _txDataDequeuedPacket[RAW_PACKET_BUFFER_SIZE];
    uint8_t _txDataQueuedPacket[RAW_PACKET_BUFFER_SIZE];
    uint8_t _rxDataPacket[RAW_PACKET_BUFFER_SIZE];
    uint8_t _rxAckPacketSequenceNumber;
    uint8_t _rxDataPacketSequenceNumber;
    uint8_t _txDataPoolHead;
    uint8_t _txDataPoolTail;
    uint8_t _txPacketPool[TX_DATA_PACKET_POOL_SIZE][RAW_PACKET_BUFFER_SIZE];
    uint8_t _stuffedRxState;
    uint8_t _rxFrameState;
    uint8_t _rxFrameIndex;
    uint8_t _packetState;
    uint8_t _packetStateFlags;
    uint8_t _txSeqNum;
    uint8_t _lastRxSeqNum;
    uint8_t _txRetries;
    uint32_t _rxFrameTimer;
    uint32_t _txTimer;
    uint32_t _initMessageTimer;
    uint32_t _ecpLedFlashTimer;
    uint32_t _cbusLedFlashTimer;
    
  


    void _logDebugHex(const char *dest, void *p, uint32_t length);
    void _makeTxDataPacket(uint8_t record_type, void *data);
    void _makeTxAckNakPacket(uint8_t data_type, uint8_t seq_num);
    bool _queueTxPacket(void *tx_packet_in);
    bool _deQueueTxPacket(void *tx_packet_out);
    bool _validateRxPacket(uint8_t data_len, void *data, uint8_t *packet_type );
    void _stuffedTx(uint8_t tx_byte);
    int _stuffedRx(uint8_t *rx_byte);
    void _rxFrame();
    void _commStateMachine();
    void _txFrame(void *tx_packet_in);
    void _processDataPacket();
    void _reportCbusLinkError();


    uint16_t _crc16(const uint8_t *data, uint16_t len, uint16_t crc, uint16_t poly=0x1021, bool refin=false, bool refout=false);

public:
    void begin(HardwareSerial *uart);
    void loop();
    void messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action);
    void getErrorCounters(ErrorCounters *dest);


};

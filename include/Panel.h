#include <Arduino.h>
#include <common.h>
#pragma once

#define RAW_PACKET_BUFFER_SIZE 64
#define MAX_PANEL_PAYLOAD (RAW_PACKET_BUFFER_SIZE - 5)
#define TX_DATA_PACKET_POOL_SIZE 4
#define STUFF_CODE 0x00
#define STX 0x02
#define ETX 0x03
#define STUFFED_BYTE_THRESHOLD 0x04
#define PT_ACK 0x00
#define PT_NAK 0x01
#define PT_DATA_SHORT 0x02
#define CRC_INIT_VEC 0x55AA
#define PANEL_MAX_RETRIES 5

enum {PRX_STATE_IDLE = 0};
enum {SRX_STATE_IDLE = 0, SRX_STATE_WAIT_SECOND};


typedef struct PanelPacketAckNak {
    uint8_t type;
    uint8_t seq_num;
    uint8_t crc16_l;
    uint8_t crc16_h;
} __attribute__((aligned(1))) PanelPacketAckNak;


typedef struct PanelPacketHeader {
    uint8_t type;
    uint8_t seq_num;
    uint8_t payload_len;
} __attribute__((aligned(1))) PanelPacketHeader;


typedef struct PanelKeyboardEvent {
    uint8_t record_type;
    uint8_t keypad_address;
    uint8_t action;
    uint8_t record_data_length;
    uint8_t record_data[MAX_CODE_LENGTH];
} __attribute__((aligned(1))) PanelKeyboardEvent; 


class Panel {
private:
    HardwareSerial *_uart;
    PanelPacketAckNak _rxAckNakPacket;
    uint8_t _rxDataPacket[RAW_PACKET_BUFFER_SIZE];
    PanelPacketAckNak _txAckNakPacket;
    uint8_t _txDataDequeuedPacket[RAW_PACKET_BUFFER_SIZE];
    uint8_t _txDataQueuedPacket[RAW_PACKET_BUFFER_SIZE];
    uint8_t _txDataPoolHead;
    uint8_t _txDataPoolTail;
    uint8_t _txPacketPool[TX_DATA_PACKET_POOL_SIZE][RAW_PACKET_BUFFER_SIZE];
    uint8_t _stuffedRxState;
    uint8_t _packetRxState;
    uint8_t _txSeqNum;
    uint8_t _lastRxSeqNum;
    uint8_t _txRetries;


    void _makeTxDataPacket(uint8_t data_len, void *data);
    void _makeTxAckNakPacket(uint8_t data_type, uint8_t seq_num);
    bool _queueTxPacket(void *tx_packet_in);
    bool _deQueueTxPacket(void *tx_packet_out);
    bool _validateRxPacket(uint8_t data_len, void *data, uint8_t *packet_type );
    void _stuffedTx(uint8_t tx_byte);
    bool _stuffedRx(uint8_t *rx_byte);


    uint16_t _crc16(const uint8_t *data, uint16_t len, uint16_t crc, uint16_t poly=0x1021, bool refin=false, bool refout=false);

public:
    void begin(HardwareSerial *uart);
    void loop();
    void messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action);


};

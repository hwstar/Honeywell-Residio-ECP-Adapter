#include <common.h>
#include <easylog.h>
#include <Panel.h>
#ifdef USE_ESP32
#include esphome/core/helpers.h
#endif

#define TAG Panel

/*
* Calculate a 16 bit CRC
*/

uint16_t Panel::_crc16(const uint8_t *data, uint16_t len, uint16_t crc, uint16_t poly, bool refin, bool refout) {

    #ifndef USE_ESP32
    // Emit local code for CRC32
    if (refin) {
      crc ^= 0xffff;
    }
    while (len--) {
        crc ^= (((uint16_t) *data++) << 8);
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return refout ? (crc ^ 0xffff) : crc;
    #else
    // Use ESPHome helper function
    return esphome::crc16be(data, len, crc, poly, refin, refout);

    #endif
 }


/*
* Make a TX packet from payload data
*/
 
void Panel::_makeTxDataPacket(uint8_t data_len, void *data) {

    uint8_t clipped_data_len = (data_len > MAX_PANEL_PAYLOAD) ? MAX_PANEL_PAYLOAD : data_len;
    PanelPacketHeader *p = (PanelPacketHeader *) data;
    uint8_t *data_start = ((uint8_t *) data) + sizeof(PanelPacketHeader);
    uint8_t *crc_start =  ((uint8_t *) data) + sizeof(PanelPacketHeader) + clipped_data_len;
    
    // Set header fields
    p->type = PT_DATA_SHORT;
    p->seq_num = _txSeqNum++;
    p->payload_len = clipped_data_len;
    // Copy the data
    memcpy(data_start, data, clipped_data_len);
    // Calculate CRC
    uint16_t crc = _crc16((uint8_t *) &_txDataQueuedPacket, data_len + 4, CRC_INIT_VEC);
    // Insert CRC
    crc_start[0] = (uint8_t) crc;
    crc_start[1] = (uint8_t) (crc >> 8);
}

void Panel::_makeTxAckNakPacket(uint8_t data_type, uint8_t seq_num) {
    // Set header fields
    _txAckNakPacket.type = data_type;
    _txAckNakPacket.seq_num = seq_num;
     // Calculate CRC
    uint16_t crc = _crc16((uint8_t *) &_txAckNakPacket, 2, CRC_INIT_VEC);
    // Insert CRC
    _txAckNakPacket.crc16_l = (uint8_t) crc;
    _txAckNakPacket.crc16_h = (uint8_t) (crc >> 8);
}

/*
* Validate a packet received
*/

bool Panel::_validateRxPacket(uint8_t data_len, void *data, uint8_t *packet_type ) {
    PanelPacketAckNak *a = (PanelPacketAckNak *) data;
    uint16_t crc;

    switch(a->type) {
        case PT_ACK:
        case PT_NAK: 
            crc = _crc16((uint8_t *) data, 2, CRC_INIT_VEC);
            if(((crc & 0xFF) == a->crc16_l) && ((crc >> 8 ) == a->crc16_l)){
                *packet_type = a->type;
                return true;
            }
            break;

        case PT_DATA_SHORT: {
                // Calculate the CRC
                crc = _crc16((uint8_t *) data, data_len - 2, CRC_INIT_VEC);
                // Calculate CRC location in the buffer
                uint8_t *crc_start = ((uint8_t *) data) + (data_len - 2);
                // Check CRC
                if(((crc & 0xFF) == crc_start[0]) && ((crc >> 8 ) == crc_start[1])){
                    *packet_type = a->type;
                    return true;
                }
            }
            break;

        default:
            break;
    }

    return false;

}

/*
* Enqueue packet for later transmission
*/

bool Panel::_queueTxPacket(void *tx_packet_in) {
    uint8_t next_head = (_txDataPoolHead + 1 > TX_DATA_PACKET_POOL_SIZE)? 0 : _txDataPoolHead + 1;
    // If pool is full, return false
    if(next_head == _txDataPoolHead){
        return false;
    }
    // Determine how much to transfer
    PanelPacketHeader *h = (PanelPacketHeader *) tx_packet_in;
    uint8_t byte_count = h->payload_len + 5;

    // Copy packet into pool
    memcpy(_txPacketPool[_txDataPoolHead], tx_packet_in, byte_count);
    _txDataPoolHead = next_head;
    return true;
}

/*
* Dequeue packet for transmission
*/

bool Panel::_deQueueTxPacket(void *tx_packet_out) {
    // If pool is empty, return false
    if(_txDataPoolHead == _txDataPoolTail) {
        return false;
    }
    uint8_t next_tail = (_txDataPoolTail + 1 > TX_DATA_PACKET_POOL_SIZE)? 0 : _txDataPoolTail + 1;

   
    // Determine how much to transfer
    PanelPacketHeader *h = (PanelPacketHeader *) _txPacketPool[_txDataPoolTail];
    uint8_t byte_count = h->payload_len + 5;

    // Copy packet out of pool
    memcpy(tx_packet_out, _txPacketPool[_txDataPoolTail], byte_count);
    _txDataPoolTail = next_tail;
    return true;
}

/*
* Transmit a byte with octet stuffing
*/

void Panel::_stuffedTx(uint8_t tx_byte) {
    if(tx_byte < STUFFED_BYTE_THRESHOLD){
        _uart->write(STUFF_CODE);
    }
    _uart->write(tx_byte);
}

/*
* Receive a byte with octet stuffing
*/

bool Panel::_stuffedRx(uint8_t *rx_byte) {
    switch(_stuffedRxState){
        case SRX_STATE_IDLE:
            if(_uart->available()) {
                _uart->readBytes(rx_byte, 1);
                if(*rx_byte == STUFF_CODE) {
                    _stuffedRxState = SRX_STATE_WAIT_SECOND;
                }
                else{
                    return true;
                }
            }
            break;
        
        case SRX_STATE_WAIT_SECOND:
            if(_uart->available()){
                _uart->readBytes(rx_byte, 1);
                _stuffedRxState = SRX_STATE_IDLE;
                return true;
            }
            break;

        default:
            _stuffedRxState = SRX_STATE_IDLE;
            break;
    }
    return false;
}


/*
* Initialization function
*/


void Panel::begin(HardwareSerial *uart) {
    _uart = uart;
    _stuffedRxState = SRX_STATE_IDLE;
    _packetRxState = PRX_STATE_IDLE;
    _lastRxSeqNum =_txSeqNum = 0;
    _txDataPoolHead = _txDataPoolTail = 0;
    _txRetries = 0;

}

/*
* Service function
*/


void Panel::loop() {
    
}

void Panel::messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action) {
    PanelKeyboardEvent pke;


    pke.record_type = record_type;
    pke.keypad_address = keypad_addr;
    pke.action = action;
    pke.record_data_length = record_data_length;
    uint8_t clipped_record_data_length = (record_data_length > MAX_CODE_LENGTH) ? MAX_CODE_LENGTH : record_data_length;
    if(clipped_record_data_length){
        memcpy(pke.record_data, record_data, record_data_length);
    }
    else {
        memset(pke.record_data, 0xFF, MAX_CODE_LENGTH);
    }
    _makeTxDataPacket(sizeof(PanelKeyboardEvent), &pke.record_data);

    bool res = _queueTxPacket(_txDataQueuedPacket);
    if(res){
        LOG_ERROR(TAG, "Buffer pool overflow error");
    }

}

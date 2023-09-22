#include <common.h>
#include <easylog.h>
#include <Panel.h>
#include <Sequencer.h>
#ifdef USE_ESP32
#include esphome/core/helpers.h
#endif

#define TAG panel

extern Sequencer seq;

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
 
void Panel::_makeTxDataPacket(PanelKeyboardEvent *pke) {


    PanelPacketHeader *p = (PanelPacketHeader *) _txDataQueuedPacket;
    
    uint8_t clipped_data_len = ((pke->record_data_length + (sizeof(PanelKeyboardEvent) - MAX_KEYPAD_DATA_LENGTH)) > MAX_PANEL_PAYLOAD) ?
    MAX_PANEL_PAYLOAD : pke->record_data_length + (sizeof(PanelKeyboardEvent) - MAX_KEYPAD_DATA_LENGTH);
    uint8_t *data_start = ((uint8_t *) _txDataQueuedPacket) + sizeof(PanelPacketHeader);
    uint8_t *crc_start =  ((uint8_t *) _txDataQueuedPacket) + sizeof(PanelPacketHeader) + clipped_data_len;
    
    // Set header fields
    p->type = PT_DATA_SHORT;
    p->seq_num = _txSeqNum++;
    p->payload_len = clipped_data_len;
    // Copy the data
    memcpy(data_start, pke, clipped_data_len);
    // Calculate CRC
    uint16_t crc = _crc16((uint8_t *) &_txDataQueuedPacket, clipped_data_len + sizeof(PanelPacketHeader), CRC_INIT_VEC);
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
            if(((crc & 0xFF) == a->crc16_l) && ((crc >> 8 ) == a->crc16_h)){
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

int Panel::_stuffedRx(uint8_t *rx_byte) {
    switch(_stuffedRxState){
        case SRX_STATE_IDLE:
            if(_uart->available()) {
                _uart->readBytes(rx_byte, 1);
                if(*rx_byte == STUFF_CODE) {
                    _stuffedRxState = SRX_STATE_WAIT_SECOND;
                }
                else{
                    if(*rx_byte == STX) {
                        return RX_GOT_STX;
                    }
                    else if(*rx_byte == ETX) {
                        return RX_GOT_ETX;
                    }
                    else if(*rx_byte == SOH) { // Unused SOH control byte
                        break;
                    }
                    else return RX_GOT_DATA;
                    
                }
            }
            break;
        
        case SRX_STATE_WAIT_SECOND:
            if(_uart->available()){
                _uart->readBytes(rx_byte, 1);
                _stuffedRxState = SRX_STATE_IDLE;
                return RX_GOT_DATA;
            }
            break;

        default:
            _stuffedRxState = SRX_STATE_IDLE;
            break;
    }
    return RX_GOT_NOTHING;
}







void Panel::_rxFrame() {
    uint8_t rx_byte;
    int res;
    uint8_t pt;
  
    switch(_rxFrameState){
        case RF_STATE_IDLE:
            res = _stuffedRx(&rx_byte);
            if(res == RX_GOT_STX){
                _rxFrameTimer = millis();
                _rxFrameIndex = 0;
                _rxFrameState = RF_WAIT_DATA_ETX;
            }
            break;

        case RF_WAIT_DATA_ETX:
            if(((uint32_t) millis()) - _rxFrameTimer > RX_FRAME_TIMEOUT_MS){
                // We timed out, start over
                _rxFrameTimeouts++;
                _rxFrameState = RF_STATE_IDLE;
            }
            res = _stuffedRx(&rx_byte);
            if(res != RX_GOT_NOTHING) {
                if(res == RX_GOT_ETX) { // End of frame
                    if(_validateRxPacket(_rxFrameIndex, _rxDataPacket, &pt)) {
                        // Got a packet, set the packet state flags accordingly
                        PanelPacketAckNak *ppan = (PanelPacketAckNak *) _rxDataPacket;
                        if(pt == PT_ACK) {
                            _rxAckPacketSequenceNumber = ppan->seq_num;
                            _packetStateFlags |= PSF_RX_ACK;
                            _rxFrameState = RF_WAIT_CLEAR_FLAGS;
                        }
                        else if(pt == PT_NAK) {
                            _rxAckPacketSequenceNumber = ppan->seq_num;
                            _packetStateFlags |= PSF_RX_NAK;
                            _rxFrameState = RF_WAIT_CLEAR_FLAGS;
                        }
                        else if(pt == PT_DATA_SHORT) {
                            // Save the data packet sequence number for subsequent ACK'ing.
                            _rxDataPacketSequenceNumber = ppan->seq_num; // Note: In the same position as an ACK/NAK packet data structure
                            _packetStateFlags |= PSF_RX_DATA;
                            _rxFrameState = RF_WAIT_CLEAR_FLAGS;
                        }
                        else {
                            // Got something we don't understand
                            _rxFrameState = RF_STATE_IDLE;
                        }
                    }
                    else {
                        // Packet Validation failed
                        _packetStateFlags |= PSF_BAD_PACKET;
                        _rxFrameState = RF_STATE_IDLE;
                    }
                }
                else if( res == RX_GOT_DATA) { // Data byte
                    _rxDataPacket[_rxFrameIndex++] = rx_byte;
                }
                else { // Unexpected frame control byte
                    _rxFrameState = RF_STATE_IDLE;
                }
            }
            break;

        case RF_WAIT_CLEAR_FLAGS:
            // Wait for main state machine to process the frame
            // before attempting to receive another.
            if((_packetStateFlags & PSF_RX_FLAGS) == 0)
                _rxFrameState = RF_STATE_IDLE;
            break;

            
        default:
             _rxFrameState = RF_STATE_IDLE;
            break;   
    }

}

/*
* Transmit a frame
*/


void Panel::_txFrame(void *tx_packet_in){
    PanelPacketHeader *h = (PanelPacketHeader *) tx_packet_in;
    PanelPacketAckNak *a = (PanelPacketAckNak *) tx_packet_in;
    uint8_t *p = (uint8_t *) tx_packet_in;
    uint8_t tx_length;

    if(a->type == PT_DATA_SHORT) {
        tx_length = sizeof(PanelPacketHeader) + sizeof(uint16_t) + h->payload_len; // Get total packet length (3 bytes of header plus 2 bytes of CRC)
    }
    else if ((a->type == PT_ACK) || (a->type == PT_NAK)) {
        tx_length = sizeof(PanelPacketAckNak);
    }
    else {
        return; // Unknown packet type
    }
    // Send the packet
    _uart->write(STX);
    for(int i = 0; i < tx_length; i++) {
        // Transmit the packet
        _stuffedTx(p[i]);
    }
    _uart->write(ETX);
    _uart->flush();
   
}

/*
* Process the data packet received
*/

void Panel::_processDataPacket() {
    // Data consists of a command header followed by a specific data structure for the command
    PanelPacketHeader *pph = (PanelPacketHeader *) _rxDataPacket;
    CommandPacketHeader *cph = (CommandPacketHeader *) (_rxDataPacket + sizeof(PanelPacketHeader));
    
    switch(cph->record_type) {

        case COMMAND_UPDATE_KEYPAD: {

            if(pph->payload_len == sizeof(CommandPacketHeader) + sizeof(KeypadCommand)) {
                if(cph->data_length == sizeof(KeypadCommand)) {
                    bool res = false;
                    uint32_t timer = millis();
                    do {
                        if(((uint32_t) millis()) - timer > 1000){
                            break;
                        }
                        KeypadCommand *kc = (KeypadCommand *) (_rxDataPacket + sizeof(PanelPacketHeader) + sizeof(CommandPacketHeader));
                        seq.formatDisplayPacket(&_f7);
                        seq.setReady(&_f7, kc->ready);
                        seq.setArmedAway(&_f7, kc->armedAway);
                        seq.setKeypadAddressBits(&_f7, kc->keypad_address);
                        seq.setChimeMode(&_f7, kc->chime);
                        seq.setLCDLine1(&_f7, kc->line1, kc->lenLine1);
                        seq.setLCDLine2(&_f7, kc->line2, kc->lenLine2);
                        seq.setLcdBackLight(&_f7, kc->back_light);
                        seq.setKeypadAddressBits(&_f7, kc->keypad_address);
                        res = seq.submitDisplayPacket(&_f7);

                    }
                    while(res == false);
                }
                else {
                    int len = sizeof(KeypadCommand);
                    LOG_DEBUG(TAG, "Incorrect length for command packet header and command packet size: is: %d, s/b: %d",
                    cph->data_length,
                    len);
                }
            }
            else {
                int len = sizeof(CommandPacketHeader) + sizeof(KeypadCommand);
                LOG_DEBUG(TAG, "Incorrect length for packet header and packet size: is: %d, s/b: %d",
                pph->payload_len,
                len);
            }
            break;
        }

        default:
            break;

    }

    


}

/*
* Master state machine
*/

void Panel::_commStateMachine() {

    switch(_packetState) {
        case PRX_STATE_INIT: {
            KeypadCommand cmd;
            // Say Init... on all keypads until we see panel updates
            seq.formatDisplayPacket(&cmd);
            seq.setLCDLine1(&cmd,(uint8_t *) "Init...", 7);
            seq.submitDisplayPacket(&cmd);
            _packetState = PRX_STATE_IDLE;
            break;
        }

        case PRX_STATE_IDLE:
        
            if(_packetStateFlags & PSF_RX_DATA) {
                // We received a data packet
                LOG_DEBUG(TAG, "Received data packet number: %d", _rxAckPacketSequenceNumber);
                _processDataPacket();
                // Make ACK Data packet
                _makeTxAckNakPacket(PT_ACK, _rxDataPacketSequenceNumber); 
                // Transmit it
                _txFrame(&_txAckNakPacket);
                // Allow reception of the next packet
                _packetStateFlags &= ~PSF_RX_FLAGS;
            }
        
        
   
            if(_packetStateFlags & PSF_RX_ACK) {
                PanelPacketHeader *pph = (PanelPacketHeader *) _txDataDequeuedPacket;
                if((_txRetries > 0) && (_txRetries < PANEL_MAX_RETRIES)) {
                    _txSoftErrors++;

                }
                if(_rxAckPacketSequenceNumber != pph->seq_num) {
                    LOG_WARN(TAG, "Received bad sequence number on ACK packet: is: %d, s/b: %d", _rxAckPacketSequenceNumber, pph->seq_num);
                }
                else {
                    LOG_DEBUG(TAG, "TX packet sucessfully Ack'ed");
                }

                // Allow reception and transmission.
                _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
            }
            // If we receive a NAK, or the TX is busy and the TX time out timer expires
            // Retry the current frame for a specified number of retries.
            // Give up and increment the hard error count if we exceed the retries.
            else if(_packetStateFlags & PSF_RX_NAK || 
                ( (_packetStateFlags & PSF_TX_BUSY) && (((uint32_t) millis()) - _txTimer > PACKET_TX_TIMEOUT_MS ))) {
                if(_packetStateFlags & PSF_TX_BUSY) {
                    if(_txRetries < PANEL_MAX_RETRIES) {
                        _txRetries++;
                        LOG_DEBUG(TAG, "Received NAK or TX timeout, at retry number: %d", _txRetries);
                        // Retransmit the current packet
                        _packetState = PRX_TX;
                        _packetStateFlags &= ~(PSF_RX_FLAGS);
                    }
                    else {
                        //The link is really messed up, or there is a bug.  We have to discard the packet
                         LOG_ERROR(TAG, "Transmit packet hard error");
                        _txHardErrors++;
                        _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
                    }
                }
                else {
                    // Ignore NAK if the TX isn't busy. It could be a NAK of an ACK/NAK packet which is meaningless.
                     _packetStateFlags &= ~(PSF_RX_FLAGS);
                }
            }
            else if(_packetStateFlags & PSF_BAD_PACKET) {
                // Packet failed validation send NAK
                _makeTxAckNakPacket(PT_NAK, 0); 
                // Transmit it
                _txFrame(&_txAckNakPacket);
                // Allow reception of the next packet
                _rxBadPackets++;
                _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);

            }
      
            //_packetStateFlags &= ~PSF_TX_BUSY; // DEBUG release all packets for now as there is no code on the ESP32 end yet

            // Dequeue next packet if there is one and we are not busy
            if(((_packetStateFlags & PSF_TX_BUSY) == 0) && (_deQueueTxPacket(_txDataDequeuedPacket))){
                _packetStateFlags |= PSF_TX_BUSY;
                _txRetries = 0;
                _packetState = PRX_TX;
            }
            break;

        case PRX_TX: // Transmit a packet in the pool
            LOG_DEBUG(TAG, "Transmitting packet number: %d", _txDataDequeuedPacket[1]);
            _txFrame(_txDataDequeuedPacket);
            _txTimer = millis();
            _packetState = PRX_STATE_IDLE;
            break;


        default:
            // Catch all
            _packetState = PRX_STATE_IDLE;
    }
    
}

/*
* Initialization function
*/


void Panel::begin(HardwareSerial *uart) {
    _uart = uart;
    _stuffedRxState = SRX_STATE_IDLE;
    _rxFrameState = RF_STATE_IDLE;
    _packetState = PRX_STATE_INIT;
    _packetStateFlags = PSF_CLEAR;
    _lastRxSeqNum =_txSeqNum = 0;
    _txDataPoolHead = _txDataPoolTail = 0;
    _txRetries = 0;
    _bufferPoolOverflowErrors = 0;
    _txTimeoutErrors = 0;
    _rxFrameTimeouts = 0;
    _rxBadPackets = 0;
    _txSoftErrors = 0;
    _txHardErrors = 0;
    _txTimer = 0;
_txTimer = millis();
}

/*
* Service function
*/


void Panel::loop() {
    _rxFrame();
    _commStateMachine();
}

void Panel::messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action) {
    PanelKeyboardEvent pke;


    pke.record_type = record_type;
    pke.keypad_address = keypad_addr;
    pke.action = action;
    pke.record_data_length = record_data_length;
   
    memset(pke.record_data, 0xFF, MAX_KEYPAD_DATA_LENGTH);
    uint8_t clipped_record_data_length = (record_data_length > MAX_KEYPAD_DATA_LENGTH) ? MAX_KEYPAD_DATA_LENGTH : record_data_length;
    if(clipped_record_data_length){
        memcpy(pke.record_data, record_data, clipped_record_data_length);
    }
  
    _makeTxDataPacket(&pke);
  
   
    bool res = _queueTxPacket(_txDataQueuedPacket);
    if(res == false){
        _bufferPoolOverflowErrors++;
        LOG_ERROR(TAG, "Buffer pool overflow error. Total overflow errors: %d",_bufferPoolOverflowErrors);
    }

}

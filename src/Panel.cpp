#include <common.h>
#include <easylog.h>
#include <Panel.h>
#include <Sequencer.h>
#include <Led.h>
#ifdef USE_ESP32
#include esphome / core / helpers.h
#endif

#define TAG panel

extern Sequencer seq;
extern Led led;

void Panel::_logDebugHex(const char *desc, void *p, uint32_t length) {
  char hex_string[16 * 3 + 1];
  uint32_t lines;
  uint32_t len;

  if (length > 16) {
    lines = (length / 16);
    if (length % 16) {
      lines++;
    }
    
  } else {
    lines = 1;
  }

  LOG_DEBUG(TAG, "%s", desc);
  for (uint32_t line = 0; line < lines; line++) {
    if (length > 16) {
      len = 16;
      length -= 16;
    } else {
      len = length;
    }
    for (uint32_t i = 0; i < len; i++) {
      snprintf(hex_string + (i * 3), 4, "%02X ", ((uint8_t *) p)[i + (line * 16)]);
    }
    LOG_DEBUG(TAG, "%s", hex_string);
  }
}

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

void Panel::_makeTxDataPacket(uint8_t *buffer, uint8_t record_type, void *data) {
  uint8_t clipped_data_len;
  uint8_t payload_bytes_remaining;
  PanelPacketHeader *p = (PanelPacketHeader *) _txDataQueuedPacket;
  PanelKeyboardEvent *pke;

  if (buffer == NULL) {
    return;
  }

  switch (record_type) {
    case RTYPE_HELLO:
      LOG_DEBUG(TAG, "makeTxPacket() making packet RTYPE_HELLO");
      clipped_data_len = 0;
      break;

    case RTYPE_DATA_FROM_KEYPAD:
      LOG_DEBUG(TAG, "makeTxPacket() making packet RTYPE_DATA_FROM_KEYPAD");
      pke = (PanelKeyboardEvent *) data;

      payload_bytes_remaining = RAW_PACKET_BUFFER_SIZE - (sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader) +
                                                          sizeof(PanelKeyboardEvent) - MAX_KEYPAD_DATA_LENGTH);

      clipped_data_len =
          ((pke->record_data_length + (sizeof(PanelKeyboardEvent) - MAX_KEYPAD_DATA_LENGTH)) > payload_bytes_remaining)
              ? payload_bytes_remaining
              : pke->record_data_length + (sizeof(PanelKeyboardEvent) - MAX_KEYPAD_DATA_LENGTH);
      break;

    case RTYPE_SEND_ERROR_COUNTERS:
      LOG_DEBUG(TAG, "makeTxPacket() making packet RTYPE_SEND_ERROR_COUNTERS");
      clipped_data_len = sizeof(ErrorCounters);
      break;

    case RTYPE_ECHO:
      LOG_DEBUG(TAG, "makeTxPacket() making packet RTYPE_ECHO");
      clipped_data_len = sizeof(EchoCommand);
      break;

    default:
      LOG_ERROR(TAG, "makeTxPacket() received unhandled record type: %d", record_type);
      return;  // Don't know what the record type is
  }

  // Point to record type header
  RecordTypeHeader *rth = (RecordTypeHeader *) (buffer + sizeof(PanelPacketHeader));

  // Point to data start
  uint8_t *data_start = buffer + (sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));

  // Point to CRC start
  uint8_t *crc_start = buffer + (sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader) + clipped_data_len);

  // Set header fields
  p->type = PT_DATA_SHORT;
  p->seq_num = _txSeqNum++;
  LOG_DEBUG(TAG, "makeTxPacket() packet sequence number: %d", p->seq_num);
  p->payload_len = clipped_data_len + sizeof(RecordTypeHeader);
  // Set the record type and length
  rth->record_type = record_type;
  rth->data_length = clipped_data_len;

  // Copy the data
  if ((clipped_data_len != 0) && (data != NULL)) {
    memcpy(data_start, data, clipped_data_len);
  }
  // Calculate CRC
  uint16_t crc = _crc16((uint8_t *) buffer, (clipped_data_len + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader)),
                        CRC_INIT_VEC);
  // Insert CRC
  crc_start[0] = (uint8_t) crc;
  crc_start[1] = (uint8_t) (crc >> 8);
}

/*
 * Make an ACK or NAK packet
 */

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

bool Panel::_validateRxPacket(uint8_t data_len, void *data, uint8_t *packet_type) {
  PanelPacketAckNak *a = (PanelPacketAckNak *) data;
  uint16_t crc;

  switch (a->type) {
    case PT_ACK:
    case PT_NAK:
      crc = _crc16((uint8_t *) data, 2, CRC_INIT_VEC);
      if (((crc & 0xFF) == a->crc16_l) && ((crc >> 8) == a->crc16_h)) {
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
      if (((crc & 0xFF) == crc_start[0]) && ((crc >> 8) == crc_start[1])) {
        *packet_type = a->type;
        return true;
      }
    } break;

    default:
      break;
  }

  return false;
}

/*
 * Enqueue packet for later transmission
 */

bool Panel::_queueTxPacket(void *tx_packet_in) {
  uint8_t next_head = (_txDataPoolHead + 1 >= TX_DATA_PACKET_POOL_SIZE) ? 0 : _txDataPoolHead + 1;
  // If pool is full, return false
  if (next_head == _txDataPoolHead) {
    return false;
  }
  // Determine how much to transfer
  PanelPacketHeader *h = (PanelPacketHeader *) tx_packet_in;
  uint8_t byte_count = h->payload_len + sizeof(PanelPacketHeader) + 2;  // 2 bytes for CRC

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
  if (_txDataPoolHead == _txDataPoolTail) {
    return false;
  }
  uint8_t next_tail = (_txDataPoolTail + 1 >= TX_DATA_PACKET_POOL_SIZE) ? 0 : _txDataPoolTail + 1;

  // Determine how much to transfer
  PanelPacketHeader *h = (PanelPacketHeader *) _txPacketPool[_txDataPoolTail];
  uint8_t byte_count = h->payload_len + sizeof(PanelPacketHeader) + 2;  // 2 bytes for CRC

  // Copy packet out of pool
  memcpy(tx_packet_out, _txPacketPool[_txDataPoolTail], byte_count);
  _txDataPoolTail = next_tail;
  return true;
}

/*
 * Transmit a byte with octet stuffing
 */

void Panel::_stuffedTx(uint8_t tx_byte) {
  if (tx_byte < STUFFED_BYTE_THRESHOLD) {
    _uart->write(STUFF_CODE);
  }
  _uart->write(tx_byte);
}

/*
 * Receive a byte with octet stuffing
 */

int Panel::_stuffedRx(uint8_t *rx_byte) {
  switch (_stuffedRxState) {
    case SRX_STATE_IDLE:
      if (_uart->available()) {
        _uart->readBytes(rx_byte, 1);
        if (*rx_byte == STUFF_CODE) {
          _stuffedRxState = SRX_STATE_WAIT_SECOND;
        } else {
          if (*rx_byte == STX) {
            return RX_GOT_STX;
          } else if (*rx_byte == ETX) {
            return RX_GOT_ETX;
          } else if (*rx_byte == SOH) {  // Unused SOH control byte
            break;
          } else
            return RX_GOT_DATA;
        }
      }
      break;

    case SRX_STATE_WAIT_SECOND:
      if (_uart->available()) {
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

/*
 * Handler to receive a frame from the SP8
 */

void Panel::_rxFrame() {
  uint8_t rx_byte;
  int res;
  uint8_t pt;

  switch (_rxFrameState) {
    case RF_STATE_IDLE:
      res = _stuffedRx(&rx_byte);
      if (res == RX_GOT_STX) {
        _rxFrameTimer = millis();
        _rxFrameIndex = 0;
        _rxFrameState = RF_WAIT_DATA_ETX;
      }
      break;

    case RF_WAIT_DATA_ETX:
      if (((uint32_t) millis()) - _rxFrameTimer > RX_FRAME_TIMEOUT_MS) {
        // We timed out, start over
        _ec.rx_frame_timeouts++;
        _rxFrameState = RF_STATE_IDLE;
      }
      res = _stuffedRx(&rx_byte);
      if (res != RX_GOT_NOTHING) {
        if (res == RX_GOT_ETX) {  // End of frame
          LOG_DEBUG(TAG, "Got frame type: %d, sequence number: %d, time (ms): %d", _rxDataPacket[0], _rxDataPacket[1],
                    millis());
          if (_validateRxPacket(_rxFrameIndex, _rxDataPacket, &pt)) {
            // Got a packet, set the packet state flags accordingly
            PanelPacketAckNak *ppan = (PanelPacketAckNak *) _rxDataPacket;
            if (pt == PT_ACK) {
              _rxAckPacketSequenceNumber = ppan->seq_num;
              _packetStateFlags |= PSF_RX_ACK;
              _rxFrameState = RF_WAIT_CLEAR_FLAGS;
            } else if (pt == PT_NAK) {
              _rxAckPacketSequenceNumber = ppan->seq_num;
              _packetStateFlags |= PSF_RX_NAK;
              _rxFrameState = RF_WAIT_CLEAR_FLAGS;
            } else if (pt == PT_DATA_SHORT) {
              // Save the data packet sequence number for subsequent ACK'ing.
              _rxDataPacketSequenceNumber =
                  ppan->seq_num;  // Note: In the same position as an ACK/NAK packet data structure
              _packetStateFlags |= PSF_RX_DATA;
              _rxFrameState = RF_WAIT_CLEAR_FLAGS;
            } else {
              // Got something we don't understand
              _rxFrameState = RF_STATE_IDLE;
            }
          } else {
            // Packet Validation failed
            _packetStateFlags |= PSF_BAD_PACKET;
            _rxFrameState = RF_STATE_IDLE;
          }
        } else if (res == RX_GOT_DATA) {  // Data byte
          _rxDataPacket[_rxFrameIndex++] = rx_byte;
        } else {  // Unexpected frame control byte
          _rxFrameState = RF_STATE_IDLE;
        }
      }

      break;

    case RF_WAIT_CLEAR_FLAGS:
      // Wait for main state machine to process the frame
      // before attempting to receive another.
      if ((_packetStateFlags & PSF_RX_FLAGS) == 0)
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

void Panel::_txFrame(void *tx_packet_in) {
  PanelPacketHeader *h = (PanelPacketHeader *) tx_packet_in;
  PanelPacketAckNak *a = (PanelPacketAckNak *) tx_packet_in;
  uint8_t *p = (uint8_t *) tx_packet_in;
  uint8_t tx_length;

  if (a->type == PT_DATA_SHORT) {
    tx_length = sizeof(PanelPacketHeader) + sizeof(uint16_t) +
                h->payload_len;  // Get total packet length (3 bytes of header plus 2 bytes of CRC)
  } else if ((a->type == PT_ACK) || (a->type == PT_NAK)) {
    tx_length = sizeof(PanelPacketAckNak);
  } else {
    return;  // Unknown packet type
  }
  // Send the packet
  _uart->write(STX);
  for (int i = 0; i < tx_length; i++) {
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
  RecordTypeHeader *cph = (RecordTypeHeader *) (_rxDataPacket + sizeof(PanelPacketHeader));

  led.cbusFlash();

  switch (cph->record_type) {
    case RTYPE_HELLO:
      LOG_DEBUG(TAG, "Received Hello message from SP8");
      // This releases the TX handler to send packets to the SP8
      _helloReceived = true;
      _makeTxDataPacket(_txDataQueuedPacket, RTYPE_HELLO); // Send response to the SP8
      _queueTxPacket(_txDataQueuedPacket);
      break;

    case RTYPE_ECHO: {
      // Return what was sent to us
      const int len1 = sizeof(EchoCommand);
      const int len2 = sizeof(RecordTypeHeader) + sizeof(EchoCommand);
      if (pph->payload_len == len2) {
        if (cph->data_length == len1) {
          EchoCommand *ec = (EchoCommand *) (_rxDataPacket + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));
          _makeTxDataPacket(_txDataQueuedPacket, RTYPE_ECHO, ec);
          _queueTxPacket(_txDataQueuedPacket);
        } else {
          LOG_DEBUG(TAG, "Incorrect length for payload packet header and payload packet size: is: %d, s/b: %d",
                    cph->data_length, len1);
        }
      } else {
        LOG_DEBUG(TAG, "Incorrect length for packet header and packet size: is: %d, s/b: %d", pph->payload_len, len2);
      }
      break;
    }

    case RTYPE_SEND_ERROR_COUNTERS:
      // Update ECP error counters
      _ec.ecp_checksum_errors = seq.getChecksumErrorCount(false);
      _ec.ecp_parity_errors = seq.getParityErrorCount(false);
      // Send the error counters
      LOG_INFO(TAG, "Received request for error counters");
      _makeTxDataPacket(_txDataQueuedPacket, RTYPE_SEND_ERROR_COUNTERS, &_ec);
      _queueTxPacket(_txDataQueuedPacket);
      LOG_INFO(TAG, "Error counters queued for TX");
      break;


    case RTYPE_UPDATE_KEYPAD: {
      const int len1 = sizeof(KeypadCommand);
      const int len2 = sizeof(RecordTypeHeader) + sizeof(KeypadCommand);
      // Update keypad LED's, Buzzer, and LCD display
      if (pph->payload_len == len2) {
        if (cph->data_length == len1) {
          KeypadCommand *kc = (KeypadCommand *) (_rxDataPacket + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));
          _logDebugHex("Keypad Command Received", kc, sizeof(KeypadCommand));  // DEBUG
          seq.formatDisplayPacket(&_f7);
          seq.setReady(&_f7, kc->ready);
          seq.setArmed(&_f7, kc->armed);
          seq.setKeypadAddressBits(&_f7, kc->keypad_address);
          seq.setChimeMode(&_f7, kc->chime);
          seq.setLCDLine1(&_f7, kc->line1, kc->lenLine1);
          seq.setLCDLine2(&_f7, kc->line2, kc->lenLine2);
          seq.setLcdBackLight(&_f7, kc->back_light);
          seq.setKeypadAddressBits(&_f7, kc->keypad_address);

          // Submit display update to sequencer buffer pool
          if (seq.submitDisplayPacket(&_f7) == false) {
            LOG_WARN(TAG, "Display Packet sequencer buffer pool full, display update will be ignored");
          }
        } else {
          LOG_DEBUG(TAG, "Incorrect length for payload packet header and payload packet size: is: %d, s/b: %d",
                    cph->data_length, len1);
        }
      } else {
        LOG_DEBUG(TAG, "Incorrect length for packet header and packet size: is: %d, s/b: %d", pph->payload_len, len2);
      }
      break;
    }

    default:
      break;
  }
}

/*
 * Report CBUS Link Error
 */
void Panel::_reportCbusLinkError() {
  const char *cbus_message = "CBUS Link Error";
  const char *check_message = "Chk. CBUS cable";
  seq.formatDisplayPacket(&_f7);
  seq.setLcdBackLight(&_f7, true);
  seq.setLCDLine1(&_f7, (uint8_t *) cbus_message, strlen(cbus_message));
  seq.setLCDLine2(&_f7, (uint8_t *) check_message, strlen(check_message));
  seq.submitDisplayPacket(&_f7);
}

/*
 * Master state machine
 */

void Panel::_commStateMachine() {
  uint32_t now = millis();

  switch (_packetState) {
    case PRX_STATE_INIT: {
      _packetState = PRX_STATE_IDLE;
      break;
    }

    case PRX_STATE_IDLE:

      if (_packetStateFlags & PSF_RX_DATA) {
        // We received a data packet
        LOG_DEBUG(TAG, "Received data packet. Sequence number: %d, record_type: %d", _rxDataPacket[1],
                  _rxDataPacket[3]);
        _processDataPacket();
        // Make ACK Data packet
        _makeTxAckNakPacket(PT_ACK, _rxDataPacketSequenceNumber);
        // Transmit it
        _txFrame(&_txAckNakPacket);
        LOG_DEBUG(TAG, "ACK transmitted to SP8 for packet sequence number: %d", _rxDataPacket[1]);
        // Unlock the receiver to allow reception of the next packet
        _packetStateFlags &= ~PSF_RX_FLAGS;
      }

      // ACK Case
      if (_packetStateFlags & PSF_RX_ACK) {
        PanelPacketHeader *pph = (PanelPacketHeader *) _txDataDequeuedPacket;
        if ((_txRetries > 0) && (_txRetries < PANEL_MAX_RETRIES)) {
          _ec.tx_soft_errors++;
        }
        if (_rxAckPacketSequenceNumber != pph->seq_num) {
          LOG_WARN(TAG, "Received bad sequence number from SP8 on ACK packet: is: %d, s/b: %d",
                   _rxAckPacketSequenceNumber, pph->seq_num);
        } else {
          LOG_DEBUG(TAG, "TX packet sequence number %d from SP8 successfully Ack'ed", pph->seq_num);
        }

        // Allow reception and transmission.
        _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
      }
      // If TX busy
      else if (_packetStateFlags & PSF_TX_BUSY) {
        // If NAK
        if (_packetStateFlags & PSF_RX_NAK) {
          if (_txRetries < PANEL_MAX_RETRIES) {
            _txRetries++;
            // Log the type of error
            if (_packetStateFlags & PSF_RX_NAK) {
              LOG_DEBUG(TAG, "SP8 NAK'ed the previously TX'd packet %d, at retry number: %d", _txDataDequeuedPacket[1],
                        _txRetries);
            }
            // Retransmit the current packet
            _packetState = PRX_TX;
            _packetStateFlags &= ~(PSF_RX_FLAGS);  // Keep TX busy
          } else {
            // The link is really messed up, or there is a bug.  We have to discard the packet
            LOG_ERROR(TAG, "SP8 Transmit NAK hard error");
            _reportCbusLinkError();
            _ec.tx_hard_errors++;
            _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
          }
        }
        // If packet transmit timeout
        else if (now - _txTimer > PACKET_TX_TIMEOUT_MS) {
          if (_txRetries < PANEL_MAX_RETRIES) {
            _txRetries++;
            // Log the type of error
            LOG_DEBUG(TAG, "SP8 TX timeout, packet %d, at retry number: %d, _now: %d, _txtimer: %d",
                      _txDataDequeuedPacket[1], _txRetries, now, _txTimer);

            // Retransmit the current packet
            _packetState = PRX_TX;
            _packetStateFlags &= ~(PSF_RX_FLAGS);  // Keep TX busy

          } else {
            // The link is really messed up, or there is a bug.  We have to discard the packet
            LOG_ERROR(TAG, "SP8 Transmit time out hard error");
            _reportCbusLinkError();
            _ec.tx_hard_errors++;
            _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
          }
        }
      }
      // If any bad packet
      else if (_packetStateFlags & PSF_BAD_PACKET) {
        LOG_DEBUG(TAG, "Received bad packet from SP8, sending NAK");
        // Packet failed validation send NAK
        _makeTxAckNakPacket(PT_NAK, 0);
        // Transmit it
        _txFrame(&_txAckNakPacket);
        // Allow reception of the next packet
        _ec.rx_bad_packets++;
        _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
      }
      // Ignore NAK outside of TX busy
      else if (_packetStateFlags == PSF_RX_NAK) {
        _packetStateFlags &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
      }

      // Dequeue next packet if hello has been received and there is one and we are not busy
      if ((_helloReceived == true) && ((_packetStateFlags & PSF_TX_BUSY) == 0) &&
          (_deQueueTxPacket(_txDataDequeuedPacket))) {
        _packetStateFlags |= PSF_TX_BUSY;
        _txRetries = 0;
        _packetState = PRX_TX;
      }
      break;

    case PRX_TX:  // Transmit a packet in the pool
      _txTimer = millis();
      LOG_DEBUG(TAG, "Transmitting packet number to SP8: %d, _txTimer %d", _txDataDequeuedPacket[1], _txTimer);
      _txFrame(_txDataDequeuedPacket);
      _packetState = PRX_STATE_IDLE;
      break;

    default:
      // Catch all
      _packetState = PRX_STATE_IDLE;
      break;
  }
}


/*
 * Return a copy of the error counters
 */

void Panel::getErrorCounters(ErrorCounters *dest) { memcpy(dest, &_ec, sizeof(ErrorCounters)); }

/*
 * Initialization function
 */

void Panel::begin(HardwareSerial *uart) {
  digitalWrite(KEYPAD_POWER_ENA, true); // Turn on power to keypads
  _uart = uart;
  _stuffedRxState = SRX_STATE_IDLE;
  _rxFrameState = RF_STATE_IDLE;
  _packetState = PRX_STATE_INIT;
  _packetStateFlags = PSF_CLEAR;
  _helloReceived = false;
  _initMessageSent = false;
  _lastRxSeqNum = _txSeqNum = 0;
  _txDataPoolHead = _txDataPoolTail = 0;
  _txRetries = 0;
  _txTimer = _initMessageTimer = _messageInactivityTimer = _keypadPowerTimer =  millis();
  
  // Clear keypad info
  memset(&_keypadInfo, 0, sizeof(PanelKeypadInfo));

  // Clear error counters
  memset(&_ec, 0, sizeof(ErrorCounters));
}

/*
 * Service function
 */

void Panel::loop() {
  _rxFrame();
  _commStateMachine();
}



void Panel::messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data,
                      uint8_t action) {
  PanelKeyboardEvent pke;

  pke.record_type = record_type;
  pke.keypad_address = keypad_addr;
  pke.action = action;
  pke.record_data_length = record_data_length;

  led.ecpFlash();
  _messageInactivityTimer = millis();

  LOG_DEBUG(TAG, "Received message in, record type: %u, record data length %u", record_type, record_data_length);

  if(record_type == KEYPAD_RECORD_TYPE_PRESENT) {
    const char *model = NULL;
    // Case for keypad addresses
    
    if ((record_data[0] >= 16) && (record_data[0] <= 23)) {
      uint8_t index = record_data[0] & 0x07; // Make index from address
      if ((record_data[4] == 0x04) && (record_data[5] == 0x04) && (record_data[6] == 0x04)) {
        model = "6160";
      }
      else if((record_data[4] == 0x04) && (record_data[5] == 0x06) && (record_data[6] == 0x04)) {
        model = "6139";
      }
      else {
        return; // Unknown keypad type
      }
      // Copy model string and length
      LOG_DEBUG(TAG, "Adding keypad model: %s", model);
      uint8_t model_len = (strlen(model) > KP_MODEL_LEN) ? KP_MODEL_LEN : strlen(model);
      _keypadInfo.info[index].length = model_len;
      memcpy(_keypadInfo.info[index].model, model, model_len);
    }
    return; 
  }
  else if (record_type != KEYPAD_RECORD_KEYS) {
    return; // Unknown record type
  }
  
 
  memset(pke.record_data, 0xFF, MAX_KEYPAD_DATA_LENGTH);
  uint8_t clipped_record_data_length =
      (record_data_length > MAX_KEYPAD_DATA_LENGTH) ? MAX_KEYPAD_DATA_LENGTH : record_data_length;
  if (clipped_record_data_length) {
    memcpy(pke.record_data, record_data, clipped_record_data_length);
  }

  _makeTxDataPacket(_txDataQueuedPacket, RTYPE_DATA_FROM_KEYPAD, &pke);

  bool res = _queueTxPacket(_txDataQueuedPacket);
  if (res == false) {
    _ec.tx_buffer_pool_overflow_errors++;
    LOG_ERROR(TAG, "Buffer pool overflow error. Total overflow errors: %d", _ec.tx_buffer_pool_overflow_errors);
    // If the buffer pool is full and no hello was received, this indicates that there is no
    // communication to the SP8 panel. Post a CBUS link error to all the displays.
    if (_helloReceived == false) {
      _reportCbusLinkError();
    }
  }
}

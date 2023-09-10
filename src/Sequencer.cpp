#include <Arduino.h>
#include <Sequencer.h>
#include <EcpSoftwareSerial.h>


#define ARMED_STAY_BIT 0x80
#define READY_BIT 0x10 
#define CHIME_BIT 0x20 
#define AC_POWER_ON_BIT 0x08
#define ARMED_AWAY_BIT 0x04
#define LCD_BACKLIGHT_BIT 0x80
#define INIT_MESSAGE_BIT 0x80


/*
* Read with time out
*/

uint8_t Sequencer::readBytes(uint8_t *buffer, uint8_t byte_count){
    readStartTime = millis();
    uint8_t i;
    for (i = 0; i < byte_count;){
        if (pEcp->available()) {
            buffer[i++] = pEcp->read();
        }
        else if (((uint32_t) millis()) - readStartTime > SEQ_READ_TIMEOUT_TIME_MS) {
            rxTimeoutErrors++;
            return 0;
        }

    }
    return i;
}


/*
* Setup function. Call in main's setup funtion to initistate = SEQ_STATE_FINISH_UP; // DEBUGalize the sequencer.
*/


void Sequencer::begin(EcpSoftwareSerial *ecp_obj, void (*keypad_action)(uint8_t *buffer, uint8_t length)) {
    pEcp = ecp_obj;
    pCallback = keypad_action;
    state = SEQ_STATE_IDLE;
    rxParityErrors = 0;
    rxChecksumErrors = 0;
    rxTimeoutErrors = 0;
    pollByteCount = 0;
    validPacket=false;
    pollInactiveTime = millis();
  



}

void Sequencer::update() {

    // ECP object sanity check
    if(pEcp == nullptr || pEcp == NULL){
        return;
    }

    switch(state) {

        case SEQ_STATE_IDLE:
            // Wait for poll interval timer to expire
            if(millis() - pollInactiveTime > DELAY_POLL_INACTIVE_MS) {
                pEcp->initiateKeypadPollSequence();
                state = SEQ_STATE_WAIT_POLL_CYCLE;
            }
            break;

        case SEQ_STATE_WAIT_POLL_CYCLE:
            // Wait for interrupt state machine to do its job
            if(!pEcp->getKeypadPollBusy()) {
                pollByteCount = readBytes(pollBuffer, 3);
                if(pollByteCount == 3) {
                    // See if there's a keypad requiring service
                    if(pollBuffer[2] != 0xFF) {
                        // Get the binary address of the keypad
                        uint8_t requesting_keypads = pollBuffer[2];
            
                        for(keypadAddress = 16; keypadAddress < 24; keypadAddress ++)
                            if((requesting_keypads & 1) == 0) {
                                // We found a keypad needing service
                                break;
                            }
                            // Next keypad bit
                            requesting_keypads >>= 1;

                        if(keypadAddress < 24) {
                            // We have a keypad to service
                            // We need to wait a prescribed amount of time, then
                            // we can send the F6 message
                            pollWaitTime = millis();
                            state = SEQ_STATE_WAIT_BEFORE_F6;
                        }
                        else {
                            // No keypads requesting service
                            state = SEQ_STATE_FINISH_UP;
                        }
                    }
                    else {
                        // No keypads requesting service
                        state = SEQ_STATE_FINISH_UP;
                    }

                }
                else {
                    // We didn't see 3 bytes
                    state = SEQ_STATE_FINISH_UP;   
                }

            }
            // Wait for poll interrupt state machine to finish
            break;

        case SEQ_STATE_WAIT_BEFORE_F6:
            // Wait here until the  wait before F6 timer expires
            if (((uint32_t ) millis() - pollWaitTime ) >= DELAY_KEYPAD_POLL_TO_F6_MS) {
                // Start the F6 sequence
                pEcp->initiateNewCommand();
                state = SEQ_STATE_WAIT_F6_TIMER;
            }
            break;


        case SEQ_STATE_WAIT_F6_TIMER:
            // Wait for the poll sequence to finish
            if(!pEcp->getKeypadPollBusy()) {
                // Flush RX buffer
 
                // We can now send the F6 command
                packet[0] = 0xF6;
                packet[1] = keypadAddress;
                pEcp->writeBytes(packet, 2);
                //Wait for last byte to TX
                while(pEcp->getTxDone())
                    ;
                // Set TX true so keypad can respond
                pEcp->setParity(true); // Clear parity error flag
                pEcp->setTxPinState(true);
                // Wait for response
                state = SEQ_STATE_WAIT_KEYPAD_RESPONSE;
  
            }
            break;

        case SEQ_STATE_WAIT_KEYPAD_RESPONSE:
            // Read the first two bytes
            if(readBytes(packet, 2) == 2){
                // First byte is address and packet sequence number
                addressAndPacketSequenceNumber = packet[0];
                // Second byte is number of bytes which follow including checksum
                // See if it will fit in the packet buffer
                if(packet[1] >= (sizeof(packet) - 2)){
                    // It's too big.  Abort
                    state = SEQ_STATE_FINISH_UP;
                    return;
                }
                // The initial message sent by the keypad will have the high bit set.
                // We need to strip it off when saving the packet length.
                packetLength = packet[1] & ~INIT_MESSAGE_BIT;
                // We get the rest of the bytes here
                if (readBytes(packet + 2, packetLength) != packetLength) {
                    // We timed out on the packet, reset and start over
                    state = SEQ_STATE_FINISH_UP;
                    return;
                }
                // Check for parity errors
                if(pEcp->getParityError() == true) {
                    // Abort packet due to parity error
                    rxParityErrors++;
                    state = SEQ_STATE_FINISH_UP;
                    return;
                }
                // Verify the checksum of the packet
                uint8_t res = pEcp->calculateChecksum(packet, packet[1] + 2);
                if(res){
                    // Bad checksum
                    rxChecksumErrors++;
                    state = SEQ_STATE_FINISH_UP;
                    return;
                }
                // We have a good packet
                // TODO: acknowledge it here
                validPacket=true;
                pEcp->initiateNewCommand();
                state = SEQ_STATE_WAIT_ACK_COMMAND;
            }
            else {
                // Didn't get 2 bytes
                state = SEQ_STATE_FINISH_UP;
            }
            break;

        case SEQ_STATE_WAIT_ACK_COMMAND:
            // Wait for command sequence to complete
            if(!pEcp->getKeypadPollBusy()){
                // Send the keypad address and packet sequence number
                pEcp->write(addressAndPacketSequenceNumber);
                while(pEcp->getTxDone())
                    ;
                pEcp->setParity(true); // Clear parity error flag
                pEcp->setTxPinState(true);  
                state = SEQ_STATE_FINISH_UP;
            }
            break;

        case SEQ_STATE_FINISH_UP:
            if(pCallback){
                if(validPacket) {
                    (*pCallback)(packet, packetLength + 2);
                }
                else {
                    (*pCallback)(NULL, 0);
                }
            }
            validPacket=false;
            // If a display message packet is pending
            // Send it here
            if(displayUpdateBusy){
                pollWaitTime = millis();
                state = SEQ_WAIT_BEFORE_F7;
            }
            else {
                state = SEQ_STATE_WAIT_NEXT_POLL_TIME;
            }

            break;


        case SEQ_WAIT_BEFORE_F7:
            // Wait before sending F7 packet
            if((((uint32_t) millis()) - pollWaitTime) >= DELAY_KEYPAD_POLL_TO_F6_MS) {
                // Tell the keypad we are going to send a command
                pEcp->initiateNewCommand();
                state = SEQ_WAIT_INIT_COMMAND;
               
            }
            break;

        case SEQ_WAIT_INIT_COMMAND:
            // Wait for new command sequence to complete
            if(!pEcp->getKeypadPollBusy()){
                index_f7 = 0;
                pEcp->write(displayPacketF7[index_f7++]);
                state = SEQ_WAIT_FOR_F7_SENT;
            }
            break;



        case SEQ_WAIT_FOR_F7_SENT:
            // Check for transmit done
            if(pEcp->getTxDone()) {
                pEcp->write(displayPacketF7[index_f7++]);
                if(index_f7 >= sizeof(Packet_F7)) {
                    pEcp->setTxPinState(true);  
                    displayUpdateBusy = false;
                    state = SEQ_STATE_WAIT_NEXT_POLL_TIME;
                }
                
            }
            break;


        case SEQ_STATE_WAIT_NEXT_POLL_TIME:
            pollInactiveTime = millis();
            state = SEQ_STATE_IDLE;
            break;



        default:
            validPacket=false;
            pollInactiveTime = millis();
            state = SEQ_STATE_IDLE;
            break;

    }

}

void Sequencer::formatDisplayPacket(void *dp) {
    Packet_F7 *f7  = (Packet_F7 *) dp;
    // Zero out the struct to start
    memset(f7, 0, sizeof(Packet_F7));
    // Set the packet type
    f7->f7 = 0xF7;
    // Set byte 4 to 0x10
    f7->uf4 = 0x10;
    // Set keypad address (send to all keypads)
    f7->keypadAddress = 0xFF;

    // Set the display lines to all spaces
    memset(f7->lcdLine1, 0x20, KEYPAD_DISPLAY_LINE_SIZE);
    memset(f7->lcdLine2, 0x20, KEYPAD_DISPLAY_LINE_SIZE);

}


void Sequencer::setKeypadAddressBits(void *dp, uint8_t address_bits) {
    ((Packet_F7 * ) dp)->keypadAddress = address_bits;
}
  

void Sequencer::setChimeMode(void *dp, uint8_t mode) {
    ((Packet_F7 * ) dp)->chime = mode;
}


void Sequencer::setArmedStay(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= ARMED_STAY_BIT;
    }
    else {
        packet->readyArmedStay &= ~ARMED_STAY_BIT;
    }
}

void Sequencer::setReady(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= READY_BIT;
    }
    else {
        packet->readyArmedStay &= ~READY_BIT;
    }
    
}

void Sequencer::setChimeFlag(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= CHIME_BIT;
    }
    else {
        packet->readyArmedStay &= ~CHIME_BIT;
    }

}

void Sequencer::setAcPowerFlag(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= AC_POWER_ON_BIT;
    }
    else {
        packet->readyArmedStay &= ~AC_POWER_ON_BIT;
    }

}

void Sequencer::setArmedAway(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= ARMED_AWAY_BIT;
    }
    else {
        packet->readyArmedStay &= ~ARMED_AWAY_BIT;
    }

}

void Sequencer::setLcdBackLight(void *dp, bool state){
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->lcdLine1[0] |= LCD_BACKLIGHT_BIT;
    }
    else {
        packet->lcdLine1[0] &= ~LCD_BACKLIGHT_BIT;
    }

}

void Sequencer::setLCDLine1(void *dp, const char *line, uint8_t length) {
    Packet_F7 *packet = (Packet_F7 * ) dp;

    // Limit the length to the size of the line buffer
    uint8_t l = (length > KEYPAD_DISPLAY_LINE_SIZE) ? KEYPAD_DISPLAY_LINE_SIZE : length;
    // Save LCD backlight bit
    uint8_t bit_7 = packet->lcdLine1[0] & 0x80;

    // Copy the line to the packet
    memcpy(packet->lcdLine1, line, l);

    // Restore LCD backlight bit
    packet->lcdLine1[0] &= ~LCD_BACKLIGHT_BIT;
    packet->lcdLine1[0] |= bit_7;

}

void Sequencer::setLCDLine2(void *dp, const char *line, uint8_t length) {
    Packet_F7 *packet = (Packet_F7 * ) dp;

    // Limit the length to the size of the line buffer
    uint8_t l = (length > KEYPAD_DISPLAY_LINE_SIZE) ? KEYPAD_DISPLAY_LINE_SIZE : length;
  
    // Copy the line to the packet
    memcpy(packet->lcdLine2, line, l);

}

bool Sequencer::submitDisplayPacket(void *dp) {
    if(displayUpdateBusy){
        return false;
    }
    // Copy the packet
    memcpy(displayPacketF7, dp, DISPLAY_PACKET_SIZE_F7);
    // Calculate and add the checksum
    displayPacketF7[DISPLAY_PACKET_SIZE_F7 - 4] = pEcp->calculateChecksum(displayPacketF7, DISPLAY_PACKET_SIZE_F7 - 4);
    // Tell the sequencer we would like to update the display.
    displayUpdateBusy = true;
    return true;
}



bool Sequencer::getDisplayUpdateBusy() {
    return displayUpdateBusy;
    
}


void Sequencer::getParityErrorCount(bool reset){
    uint32_t x = rxParityErrors;
    if(reset){
        rxParityErrors = 0;
    }

}

void Sequencer::getChecksumErrorCount(bool reset){
    uint32_t x = rxChecksumErrors;
    if(reset){
        rxChecksumErrors = 0;
    }
    
}

void Sequencer::getTimeOutErrorCount(bool reset){
    uint32_t x = rxTimeoutErrors;
    if(reset){
        rxTimeoutErrors = 0;
    }
    
}



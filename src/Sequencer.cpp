


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

uint8_t Sequencer::_readBytes(uint8_t *buffer, uint8_t byte_count){
    readStartTime = millis();
    uint8_t i;
    for (i = 0; i < byte_count;) {
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

// Translate keypad digit to ASCII.

uint8_t Sequencer::_translateKeypadDigit(uint8_t digit){
    if(digit <= 9) { 
        return digit + '0';
    }
    else if(digit == 0x0A) {
        return  '*';
    }
    else if(digit == 0x0B) {
        return '#';
    }
    else if (digit == 0x1C) {
        return 'A';
    }
    else if (digit == 0x1D) {
        return 'B';
    }
    else if (digit == 0x1E) {
        return 'C';
    }
    else if (digit == 0x1F) {
        return 'D';
    }
    else {
        return 'U';
    }
}



/*
* Setup function. 
*/


void Sequencer::begin(EcpSoftwareSerial *ecp_obj, void (*keypad_action)(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action)) {
    pEcp = ecp_obj;
    pCallback = keypad_action;
    state = SEQ_STATE_IDLE;
    rxParityErrors = 0;
    rxChecksumErrors = 0;
    rxTimeoutErrors = 0;
    pollByteCount = 0;
    codeDigitsReceived = false;
    validPacket=false;
    for(int i = 0; i < MAX_KEYPADS; i++) {
        memset(&codeData[i], 0, sizeof(Code_Info));
    }

    pollInactiveTime = millis();
  



}

void Sequencer::_handleECP() {

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
                pollByteCount = _readBytes(pollBuffer, 3);
                if(pollByteCount == 3) {
                    // See if there's a keypad requiring service
                    if(pollBuffer[2] != 0xFF) {
                        // Get the binary address of the keypad
                        uint8_t requesting_keypads = pollBuffer[2];
            
                        for(keypadAddress = 16; keypadAddress < 24; keypadAddress ++) {
                            if((requesting_keypads & 1) == 0) {
                                // We found a keypad needing service
                                break;
                            }
                            // Next keypad bit 
                            requesting_keypads >>= 1;
                        }
                    

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
                while(pEcp->getTxBusy())
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
            if(_readBytes(packet, 2) == 2){
                // First byte is address and packet sequence number
                addressAndPacketSequenceNumber = packet[0];
                // Second byte is number of bytes which follow including checksum
                // See if it will fit in the packet buffer
                // The initial message sent by the keypad will have the high bit set.
                // We need to strip it off when saving the packet length.
                packetLength = packet[1] & ~INIT_MESSAGE_BIT;
                if(packetLength >= (sizeof(packet) - 2)){
                    // It's too big.  Abort
                    state = SEQ_STATE_FINISH_UP;
                    return;
                }
                // We get the rest of the bytes here
                if (_readBytes(packet + 2, packetLength) != packetLength) {
                    // We timed out on the packet, reset and start over
                    packetLength = 0;
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
                uint8_t res = pEcp->calculateChecksum(packet, packetLength + 2);
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
                while(pEcp->getTxBusy())
                    ;          
                pEcp->setParity(true); // Clear parity error flag
                pEcp->setTxPinState(true);  
                state = SEQ_STATE_FINISH_UP;
            }
            break;

        case SEQ_STATE_FINISH_UP:
            if(pollByteCount != 3){
                // If 3 poll bytes weren't received, skip this state and check for a display request
                state = SEQ_CHECK_DISPLAY_REQUEST;
                return;
            }

            if(pCallback){
                if(validPacket) {
                    if(packet[1] & 0x80) {
                        // Keypad present packet
                        (*pCallback)(KEYPAD_RECORD_TYPE_PRESENT, packet[0] & 0x3F, packet[1] & 0x7F, packet, 0 );
                     }
                    else {
                        // Keypad digits packet
                        // Copy the code digits we received to the codeDigits buffer for processing by the code buffer state machine.
                        uint8_t num_code_digits = packetLength - 1; // Exclude the checksum byte
                        uint8_t byte_count = (num_code_digits > CODE_DIGIT_BUFFER_SIZE) ? CODE_DIGIT_BUFFER_SIZE : num_code_digits;
                        
                        // Copy to code digits buffer
                        memcpy(codeDigits, packet + 2, byte_count);
                        // Copy the keypad address
                        codeDigitsKeypadAddress = keypadAddress;
                        // Set the number of digits received
                        codeDigitsReceived = byte_count; 
                    }
                }
            }
            validPacket=false;
            state = SEQ_CHECK_DISPLAY_REQUEST;
            break;

        case SEQ_CHECK_DISPLAY_REQUEST:
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
                indexF7 = 0;
                pEcp->write(displayPacketF7[indexF7++]);
                state = SEQ_WAIT_FOR_F7_SENT;
            }
            break;



        case SEQ_WAIT_FOR_F7_SENT:
            // Check for transmit done
            if(!pEcp->getTxBusy()) {
                pEcp->write(displayPacketF7[indexF7++]);
                // Wait for last byte to TX
                while(pEcp->getTxBusy())
                    ;               
                if(indexF7 >= sizeof(Packet_F7)) {
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

/*
* This function handles the code buffers for the individual keypads 
* It stores the code digits for each keypad until the code digit length is reached
* then waits for the command digit which follows the code. Once is has all of this
* information, it calls the callback function supplied when begin was called.
*/


void Sequencer::_handleKeypads() {
    for(int i = 0; i < MAX_KEYPADS; i++){
        switch(codeData[i].state) {
            case CODE_STATE_IDLE:

                if(codeDigitsReceived && (codeDigitsKeypadAddress == i + 16)){
                    codeData[i].timer = millis();
                    codeData[i].state = CODE_STATE_CODE_INTERDIGIT;
                }
                break;

            case CODE_STATE_CODE_INTERDIGIT:
                if(((uint32_t) millis()) - codeData[i].timer >= CODE_INTERDIGIT_TIME) {
                    // Max interdigit time reached, reset the code buffer and start over
                    codeData[i].length = 0;
                    codeDigitsReceived = 0;
                    codeData[i].state = CODE_STATE_IDLE;
                }
                if(codeDigitsReceived && (codeDigitsKeypadAddress == i + 16)) {
                    // Check for the panic function keys
                    uint8_t key_index;
                    for(key_index = 0; key_index < codeDigitsReceived; key_index++){
                        if ((codeDigits[key_index] >= 0x1C) && (codeDigits[key_index] <= 0x1F)) {
                            // Quirk: A zero is returned on the second press of any panic key
                            // after the first press before the display contents are updated. 
                            // This will be ignored.
                            
                            // Call the callback function and indicate a panic key was pressed
                            (*pCallback)(KEYPAD_RECORD_TYPE_PANIC, i + 16, 0, NULL, _translateKeypadDigit(codeDigits[key_index]));
                            codeData[i].length = 0;
                            codeDigitsReceived = 0;
                            codeData[i].state = CODE_STATE_IDLE;
                            return;
                        }
                    }

                    // We got normal keys
                    if (codeData[i].length + codeDigitsReceived > MAX_CODE_LENGTH) { // Case 1: number of digits received exceeds the max code length
                        // Copy the code digits
                        uint8_t copy_digits = MAX_CODE_LENGTH - codeData[i].length;
                        memcpy(codeData[i].buffer + codeData[i].length, codeDigits, copy_digits);
                        // Update length
                        codeData[i].length = MAX_CODE_LENGTH;
                        // Check to see if there is at least one more digit for the command
                        // and if so, process it here.
                        if(((int) codeDigitsReceived) - ((int) copy_digits > 0)) {
                            // We received all of the code and command digits
                            codeData[i].command = codeDigits[copy_digits];
                            codeDigitsReceived = 0;
                            codeData[i].state = CODE_STATE_CALL_CALLBACK;

                        }
                        else {
                            // We received all of the code digits, but not the command digit
                            codeData[i].timer = millis();
                            codeDigitsReceived = 0;
                            codeData[i].state = CODE_STATE_COMMAND_INTERDIGIT;
                        }
                    }
                    else { // Case 2: number of digits received is equal to or less than the max code length
                        memcpy(codeData[i].buffer + codeData[i].length, codeDigits, codeDigitsReceived);
                        codeData[i].length += codeDigitsReceived;
                        codeDigitsReceived = 0;
                        if(codeData[i].length >= MAX_CODE_LENGTH){
                            codeData[i].timer = millis();
                            codeData[i].state = CODE_STATE_COMMAND_INTERDIGIT;
                        }
                    }
                }
                break;

            case CODE_STATE_COMMAND_INTERDIGIT:
                // Wait for the last command digit
                if(((uint32_t) millis()) - codeData[i].timer >= CODE_INTERDIGIT_TIME) {
                    // Max interdigit time reached, reset the code buffer and start over
                    codeDigitsReceived = 0;
                    codeData[i].length = 0;
                    codeData[i].state = CODE_STATE_IDLE;
                }

                if(codeDigitsReceived && (codeDigitsKeypadAddress == i + 16)){
                    codeData[i].command = codeDigits[0];
                    codeDigitsReceived = 0;
                    codeData[i].state = CODE_STATE_CALL_CALLBACK;
                }
                break;


            case CODE_STATE_CALL_CALLBACK:
                // Translate keypad digits to ASCII
                for(int j = 0; j < codeData[i].length; j++){
                    codeData[i].buffer[j] = _translateKeypadDigit(codeData[i].buffer[j]);
                }
                codeData[i].command = _translateKeypadDigit(codeData[i].command);

                // Call the callback function with the keypad data
                (*pCallback)(KEYPAD_RECORD_TYPE_CODE, i + 16, codeData[i].length, codeData[i].buffer, codeData[i].command);
                codeData[i].length = 0;
                codeData[i].state = CODE_STATE_IDLE;
                break;
        
            
            default:
                codeDigitsReceived = 0;
                codeData[i].length = 0;
                codeData[i].state = CODE_STATE_IDLE;
                break;
        }
    }

}

/*
* This must be called periodically from the loop() function in main.cpp 
*/

void Sequencer::update() {
    _handleECP();
    _handleKeypads();
}


/*
* Format display packet from a chunk of randomly initialized memory
*/


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


/*
* Allow keypad specific addressing
*/

void Sequencer::setKeypadAddressBits(void *dp, uint8_t address_bits) {
    ((Packet_F7 * ) dp)->keypadAddress = address_bits;
}
  
/*
* Cause the keypad to chime in a defined manner
*/

void Sequencer::setChimeMode(void *dp, uint8_t mode) {
    ((Packet_F7 * ) dp)->chime = mode;
}

/*
* Send the alarm stay state to the keypad
*/

void Sequencer::setArmedStay(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= ARMED_STAY_BIT;
    }
    else {
        packet->readyArmedStay &= ~ARMED_STAY_BIT;
    }
}

/*
* Set the ready LED on the keypad
*/

void Sequencer::setReady(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= READY_BIT;
    }
    else {
        packet->readyArmedStay &= ~READY_BIT;
    }
    
}

/*
* Set the chime flag. English displays seem to not do anything with this
*/

void Sequencer::setChimeFlag(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= CHIME_BIT;
    }
    else {
        packet->readyArmedStay &= ~CHIME_BIT;
    }

}

/*
* Set the AC power flag. English displays seem to not do anything with this
*/

void Sequencer::setAcPowerFlag(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->readyArmedStay |= AC_POWER_ON_BIT;
    }
    else {
        packet->readyArmedStay &= ~AC_POWER_ON_BIT;
    }

}


/*
* Set the Armed LED on the keypad/.
*/

void Sequencer::setArmedAway(void *dp, bool state) {
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->statusBits |= ARMED_AWAY_BIT;
    }
    else {
        packet->statusBits &= ~ARMED_AWAY_BIT;
    }

}

/*
* Set the backlight for the LCD display
*/

void Sequencer::setLcdBackLight(void *dp, bool state){
    Packet_F7 *packet = (Packet_F7 * ) dp;
    if(state){
        packet->lcdLine1[0] |= LCD_BACKLIGHT_BIT;
    }
    else {
        packet->lcdLine1[0] &= ~LCD_BACKLIGHT_BIT;
    }

}


/*
* Set the content for Line 1 on the keypad
*/


void Sequencer::setLCDLine1(void *dp, uint8_t *line, uint8_t length) {
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

/*
* Set the content for Line 2 on the keypad
*/

void Sequencer::setLCDLine2(void *dp, uint8_t *line, uint8_t length) {
    Packet_F7 *packet = (Packet_F7 * ) dp;

    // Limit the length to the size of the line buffer
    uint8_t l = (length > KEYPAD_DISPLAY_LINE_SIZE) ? KEYPAD_DISPLAY_LINE_SIZE : length;
  
    // Copy the line to the packet
    memcpy(packet->lcdLine2, line, l);

}


/*
* Submit the display packet for transmission to the displays.
*
* Will return false if a prior transmission is in progress else true.
*/




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

/*
* Return true if we are busy transmitting a display message
*/

bool Sequencer::getDisplayUpdateBusy() {
    return displayUpdateBusy;
    
}



uint32_t Sequencer::getParityErrorCount(bool reset){
    uint32_t x = rxParityErrors;
    if(reset){
        rxParityErrors = 0;
    }
    return x;

}

uint32_t Sequencer::getChecksumErrorCount(bool reset){
    uint32_t x = rxChecksumErrors;
    if(reset){
        rxChecksumErrors = 0;
    }
    return x;
    
}

uint32_t Sequencer::getTimeOutErrorCount(bool reset){
    uint32_t x = rxTimeoutErrors;
    if(reset){
        rxTimeoutErrors = 0;
    }
    return x;
    
}



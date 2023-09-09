#include <Arduino.h>
#include <Sequencer.h>
#include <EcpSoftwareSerial.h>


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
* Setup function. Call in main's setup funtion to initistate = SEQ_STATE_RESET_AND_START_OVER; // DEBUGalize the sequencer.
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
                            pollWaitBeforeF6Time = millis();
                            state = SEQ_STATE_WAIT_BEFORE_F6;
                        }
                        else {
                            // No keypads requesting service
                            state = SEQ_STATE_RESET_AND_START_OVER;
                        }
                    }
                    else {
                        // No keypads requesting service
                        state = SEQ_STATE_RESET_AND_START_OVER;
                    }

                }
                else {
                    // We didn't see 3 bytes
                    state = SEQ_STATE_RESET_AND_START_OVER;   
                }

            }
            // Wait for poll interrupt state machine to finish
            break;

        case SEQ_STATE_WAIT_BEFORE_F6:
            // Wait here until the  wait before F6 timer expires
            if (((uint32_t ) millis() - pollWaitBeforeF6Time ) >= DELAY_KEYPAD_POLL_TO_F6_MS) {
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
                    state = SEQ_STATE_RESET_AND_START_OVER;
                    return;
                }
                // For the 6160 keypad , the high bit of packet length gets set on initial packet from keypad. 
                // I don't know what the significance of this is. It seems to be 
                // firmware version info or keypad capabilities. It will need to 
                // be researched further with other keypad models. We will just acknowledge the packet and send
                // it to the callback for any processing.

                packetLength = packet[1] & 0x7F;
                // We get the rest of the bytes here
                if (readBytes(packet + 2, packetLength) != packetLength) {
                    // We timed out on the packet, reset and start over
                    state = SEQ_STATE_RESET_AND_START_OVER;
                    return;
                }
                // Check for parity errors
                if(pEcp->getParityError() == true) {
                    // Abort packet due to parity error
                    rxParityErrors++;
                    state = SEQ_STATE_RESET_AND_START_OVER;
                    return;
                }
                // Verify the checksum of the packet
                uint8_t res = pEcp->calculateChecksum(packet, packet[1] + 2);
                if(res){
                    // Bad checksum
                    rxChecksumErrors++;
                    state = SEQ_STATE_RESET_AND_START_OVER;
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
                state = SEQ_STATE_RESET_AND_START_OVER;
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
                state = SEQ_STATE_RESET_AND_START_OVER;
            }
            break;

        case SEQ_STATE_RESET_AND_START_OVER:
          
            if(pCallback){
                if(validPacket) {
                    (*pCallback)(packet, packetLength + 2);
                }
                else {
                    (*pCallback)(NULL, 0);
                }
            }
            // TODO: Dequeue a keypad message queued (if any here)
            // and send it to the keypad



    
            validPacket=false;
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
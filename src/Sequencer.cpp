#include <Arduino.h>
#include <Sequencer.h>
#include <EcpSoftwareSerial.h>


/*
* Read with time out
*/

uint8_t Sequencer::readBytes(uint8_t *buffer, uint8_t byte_count){
    readStartTime = millis();
    uint8_t i;
    for (i = 0; i < byte_count; i++){
        if (pEcp->available()) {
            buffer[i] = pEcp->read();
        }
        else if (((uint32_t) millis()) - readStartTime > SEQ_READ_TIMEOUT_TIME_MS) {
            rxTimeoutErrors++;
            return 0;
        }

    }
    return i;
}


/*
* Setup function. Call in main's setup funtion to initialize the sequencer.
*/


void Sequencer::begin(EcpSoftwareSerial *ecp_obj) {
    pEcp = ecp_obj;
    state = SEQ_STATE_IDLE;
    rxParityErrors = 0;
    rxChecksumErrors = 0;
    rxTimeoutErrors = 0;
    pollByteCount = 0;
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
                            //state = SEQ_STATE_WAIT_BEFORE_F6;
                            state = SEQ_STATE_RESET_AND_START_OVER; // DEBUG
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
                pEcp->setParity(true); // Clear parity error flag
                pEcp->rx_flush();
                // We can now send the F6 command
                packet[0] = 0xF6;
                packet[1] = keypadAddress;
                pEcp->writeBytes(packet, 2);
                //Wait for last byte to TX
                while(pEcp->getTxDone())
                    ;
                // Set TX true so keypad can respond
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
                // We get the rest of the bytes here
                if (readBytes(packet + 2, packet[1]) != packet[1]) {
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
                state = SEQ_STATE_RESET_AND_START_OVER;
            }
            else {
                // Didn't get 2 bytes
                state = SEQ_STATE_RESET_AND_START_OVER;
            }
            break;




        case SEQ_STATE_RESET_AND_START_OVER:
        default:
            pollInactiveTime = millis();
            state = SEQ_STATE_IDLE;
            break;

    }

}
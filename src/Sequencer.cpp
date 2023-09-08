#include <Arduino.h>
#include <Sequencer.h>
#include <EcpSoftwareSerial.h>

void Sequencer::setup() {
    state = SEQ_STATE_IDLE;
    rxParityErrors = 0;
    rxChecksumErrors = 0;
    rxTimeoutErrors = 0;
    pollInactiveTime = millis();
  



}

void Sequencer::update() {
    switch(state) {

        case SEQ_STATE_IDLE:
            if(millis() - pollInactiveTime > DELAY_POLL_INACTIVE_MS) {
                state = SEQ_STATE_START_POLL_CYCLE;
                
            }
            break;




        default: // Catch all
            pollInactiveTime = millis();
            state = SEQ_STATE_IDLE;
            break;

    }

}
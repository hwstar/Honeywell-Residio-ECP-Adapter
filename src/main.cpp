#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>
#include <Sequencer.h>


EcpSoftwareSerial ecp(2, 3, true, true, 4);
Sequencer seq;
HardwareSerial Serial1(USART1);


void message_callback(uint8_t *packet, uint8_t length){
}



void setup() {
  // Math check (using debugger breakpoints)
  volatile uint32_t x;
  x = TICKS_TX_REQUEST;
  x = TICKS_POLL_START;
  x = TICKS_POLL_LOW;
  x = TICKS_POLL_HIGH;
  x = DELAY_KEYPAD_POLL_TO_F6_MS;

  ecp.begin();
  seq.begin(&ecp, message_callback);
}


void loop() {
  seq.update();
}

 


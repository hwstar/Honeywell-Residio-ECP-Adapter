#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>
#include <Sequencer.h>


EcpSoftwareSerial ecp(2, 3, true, true, 4);
Sequencer seq;
HardwareSerial Serial1(USART1);

uint32_t test_timer;

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

  x = sizeof(Packet_F7);

  ecp.begin();
  seq.begin(&ecp, message_callback);
  test_timer = millis();

}


void loop() {
  seq.update();
  if((((uint32_t) millis()) - test_timer) > 10000) {
    Packet_F7 packet;
    test_timer = millis();
    seq.formatDisplayPacket(&packet);
    seq.setLCDLine1(&packet, "SP8 Alarm System",16);
    seq.setLCDLine2(&packet, "Test 9/10/23", 12 );
    seq.setArmedAway(&packet, true);
    seq.setLcdBackLight(&packet, true);
    seq.submitDisplayPacket(&packet);
  }
}

 


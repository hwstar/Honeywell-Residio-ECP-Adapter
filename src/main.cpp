#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>

EcpSoftwareSerial ecp(2, 3, true, true, 4);

HardwareSerial Serial1(USART1);

void send_f6(uint8_t keypad_address) {
  ecp.notifyFirstByte();
  ecp.write(0xf6);
  ecp.write(keypad_address);
}



void setup() {
  ecp.begin();
  ecp.setParity(true);
  Serial1.begin(115200);
}

void loop() {
volatile uint32_t x;

// Math check (using debugger breakpoints)
x = TICKS_TX_REQUEST;
x = TICKS_POLL_START;
x = TICKS_POLL_LOW;
x = TICKS_POLL_HIGH;
x = DELAY_KEYPAD_POLL_TO_F6_MS;




ecp.initiateKeypadPollSequence();

while(ecp.getKeypadPollBusy())
  ;

delay(DELAY_KEYPAD_POLL_TO_F6_MS);
send_f6(0x10);
while(ecp.getTxDone());
  ;
ecp.setTxPinState(true);


 

delay(DELAY_POLL_INACTIVE_MS);

}

 


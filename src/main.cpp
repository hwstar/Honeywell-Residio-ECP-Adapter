#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>

EcpSoftwareSerial ecp(2, 3, false, false, 4);

HardwareSerial Serial1(USART1);

void setup() {
  ecp.begin();
  ecp.setParity(true);
  Serial1.begin(115200);
}

void loop() {

 /*
  // keypad poll test
  while(ecp.getKeypadPollBusy())
    ;
  ecp.initiateKeypadPollSequence();
  delay(100);
  return;
*/

/*
  static uint8_t tx_byte;
  tx_byte = (uint8_t) int(random());
  
  ecp.write(tx_byte);
  while(!ecp.available());
  uint8_t x = ecp.read();
  if(ecp.getParityError()){
    Serial1.printf("Parity Error!\n");
  }
  else if(x != tx_byte) {
      Serial1.printf("Bad!\n");
  }

*/
 ecp.write(0x55, false);




delay(100);

}

 


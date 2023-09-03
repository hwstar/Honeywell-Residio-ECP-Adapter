#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>

EcpSoftwareSerial ecp(2, 3, false, 4);

HardwareSerial Serial1(USART1);

void setup() {
  ecp.begin();
  ecp.setParity(true);
  Serial1.begin(115200);
}

void loop() {
  
  ecp.write(0x55);
  if(ecp.available()){
    uint8_t x = ecp.read();
    Serial1.printf("%02X\n", x);
  }



  delay(100);
}

 


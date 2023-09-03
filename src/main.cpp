#include <Arduino.h>
#include <EcpSoftwareSerial.h>


EcpSoftwareSerial ecp(2, 3, false, 4);

void setup() {
  ecp.begin();
  ecp.setParity(true);
}

void loop() {
  
  ecp.write(0x00);
  ecp.write(0x01);
  ecp.write(0x03);
  ecp.write(0x07);
  ecp.write(0x0F);
  ecp.write(0x1F);
  ecp.write(0x3f);
  ecp.write(0x7f);
  ecp.write(0xff);


  delay(100);
}

 


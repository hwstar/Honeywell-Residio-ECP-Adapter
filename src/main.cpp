#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>
#include <Sequencer.h>
#include <Console.h>
#include <common.h>
#include <Panel.h>


EcpSoftwareSerial ecp(ECP_RX, ECP_TX, true, true, ECP_DEBUG); // UART for the Honeywell ECP bus
Sequencer seq; // Keypad sequencer
HardwareSerial SerialConsole(CONSOLE_RX, CONSOLE_TX); // Console UART
HardwareSerial SerialPanel(PANEL_RX, PANEL_TX); // Panel UART
Console console;
Panel panel;


void message_callback(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action){
  panel.messageIn(record_type, keypad_addr, record_data_length, record_data, action);
  console.messageIn(record_type, keypad_addr, record_data_length, record_data, action);
}



void setup() {

  // Outputs 
  pinMode(ECP_COMM_LED, OUTPUT);
  pinMode(CBUS_COMM_LED, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);

  digitalWrite(ECP_COMM_LED, true); // ECP comm led off
  digitalWrite(CBUS_COMM_LED, true); // CBUS comm led off
  digitalWrite(DEBUG_PIN, false);

  SerialConsole.begin(115200);
  SerialPanel.begin(4800);
  panel.begin(&SerialPanel);
  console.begin(&SerialConsole);
  ecp.setInterruptPriority(0,0); 
  ecp.begin();
  seq.begin(&ecp, message_callback);

}


void loop() {
  seq.update();
  panel.loop();
  console.loop();
}

 


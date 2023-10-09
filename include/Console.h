#pragma once
#include <Arduino.h>
#include <lilParser.h>

enum commands { NO_COMMAND = 0, GET_CBUS_ERRORS, GET_ECP_ERRORS };

class Console {
 private:
  HardwareSerial *_uart;
  lilParser _parser;

 public:
  void begin(HardwareSerial *uart);
  void loop();
  void messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data,
                 uint8_t action);
};

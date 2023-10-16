#include <common.h>
#include <easylog.h>
#include <Console.h>
#include <Sequencer.h>
#include <Panel.h>

#define TAG console

extern Panel panel;
extern Sequencer seq;

void Console::begin(HardwareSerial *uart) {
  _uart = uart;
  _parser.addCmd(GET_CBUS_ERRORS, "cbus-errors");
  _parser.addCmd(GET_ECP_ERRORS, "ecp-errors");

  LOG_DEBUG(TAG, "Console initialized");
  _uart->print("\r\n:");
}

void Console::loop() {
  bool line_term = false;
  if (_uart->available()) {
    // Get the char
    char c = _uart->read();
    if (c == '\r') {
      // Prepend \n when we see \r
      _uart->print('\n');
      line_term = true;
    }
    // Echo the character received
    _uart->print(c);

    // Lil parser needs to see \n, but we got \r
    if (c == '\r')
      c = '\n';

    int command = _parser.addChar(c);

    switch (command) {
      case NO_COMMAND:
        break;

      case GET_CBUS_ERRORS: {
        ErrorCounters ec;
        panel.getErrorCounters(&ec);
        _uart->print("\r\n");
        _uart->printf("TX soft errors              : %u\r\n", ec.tx_soft_errors);
        _uart->printf("TX hard_errors              : %u\r\n", ec.tx_hard_errors);
        _uart->printf("Buffer pool overflow errors : %u\r\n", ec.tx_buffer_pool_overflow_errors);
        _uart->printf("RX bad packets              : %u\r\n", ec.rx_bad_packets);
        _uart->printf("RX frame timeouts           : %u\r\n", ec.rx_frame_timeouts);
        _uart->print("\r\n");
        break;
      }

      case GET_ECP_ERRORS: {
        _uart->print("\r\n");
        uint32_t parity_errors = seq.getParityErrorCount();
        uint32_t checksum_errors = seq.getChecksumErrorCount();
        _uart->printf("Parity Errors              : %u\r\n", parity_errors);
        _uart->printf("Checksum Errors            : %u\r\n", checksum_errors);
        _uart->print("\r\n");
        break;
      }

      default:
        _uart->print("What?\r\n");
        break;
    }
    if (line_term)
      _uart->print(':');  // Print command prompt
  }
}

/*
 * Print message from keypad
 */

void Console::messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data,
                        uint8_t action) {
  uint8_t i;
  char hex_string[(MAX_KEYPAD_DATA_LENGTH * 3) + 3];

  uint8_t clipped_length = (record_data_length > MAX_KEYPAD_DATA_LENGTH) ? MAX_KEYPAD_DATA_LENGTH : record_data_length;
  // Create hex string
  for (i = 0; i < clipped_length; i++) {
    snprintf((hex_string + (i * 3)), 4, "%02X ", record_data[i]);
  }
  hex_string[((i - 1) * 3) + 2] = 0;
  if (record_data_length) {
    LOG_INFO(TAG, "Message from keypad keypad address %02X: Record type: %02X, Data length: %d, Data: %s\r\n",
             keypad_addr, record_type, record_data_length, hex_string);
  }
}

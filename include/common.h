#pragma once
#include <Arduino.h>

#define MAX_KEYPAD_LINE 16

#define ADDR_ALL_KEYPADS 255

#define MAX_KEYPAD_DATA_LENGTH 16

#define LOG_LEVEL LOG_LEVEL_DEBUG

/*
 *  Pin assignments
 */

#define ECP_RX PA7
#define ECP_TX PA6
#define DEBUG_PIN PA4                 // General purpose debug pin for scoping.
#define DEBUG_PIN_ECP_PARITY_ERR PB3  // Rises on parite error, falls at the beginning of a ECP packet transfer.
#define ECP_DEBUG PA5                 // Debug output for software UART sampling point
#define DEBUG_RX_SAMPLING  // Uncomment for output on ECP_DEBUG which will show the rx data sampling point on each edge
                           // on a scope

#define CONSOLE_RX PA10
#define CONSOLE_TX PA9

#define PANEL_RX PA3
#define PANEL_TX PA2

#define ECP_COMM_LED PC13
#define CBUS_COMM_LED PA0

/*
 * Macros
 */

#define NEXT_QUEUE_INDEX(prev, depth) ((prev + 1 >= depth) ? 0 : prev + 1)
#define TEST_TIMER(timer, duration) ((((uint32_t) millis()) - timer) > duration)

/*
 * Enums used in communication with the panel
 */

enum {
  CHIME_NONE = 0,
  CHIME_ONCE,
  CHIME_TWICE,
  CHIME_THREE_TIMES,
  CHIME_FAST_REPEATING,
  CHIME_SLOW_REPEATING,
  CHIME_UNUSED,
  CHIME_LOUD
};
enum { KEYPAD_RECORD_TYPE_PRESENT = 0, KEYPAD_RECORD_KEYS };
enum { RTYPE_HELLO = 0, RTYPE_SEND_ERROR_COUNTERS, RTYPE_UPDATE_KEYPAD, RTYPE_DATA_FROM_KEYPAD, RTYPE_ECHO };

/*
 * Structs used in communication with the panel
 */

typedef struct alignas(1) PanelPacketAckNak {
  uint8_t type;
  uint8_t seq_num;
  uint8_t crc16_l;
  uint8_t crc16_h;
} PanelPacketAckNak;

typedef struct alignas(1) PanelPacketHeader {
  uint8_t type;
  uint8_t seq_num;
  uint8_t payload_len;
} PanelPacketHeader;

typedef struct alignas(1) RecordTypeHeader {
  uint8_t record_type;
  uint8_t data_length;
} RecordTypeHeader;

typedef struct alignas(1) KeypadCommand {
  bool ready;
  bool armed;
  bool back_light;
  uint8_t keypad_address;
  uint8_t chime;
  uint8_t lenLine1;
  uint8_t lenLine2;
  uint8_t line1[MAX_KEYPAD_LINE];
  uint8_t line2[MAX_KEYPAD_LINE];

} KeypadCommand;

typedef struct alignas(1) PanelKeyboardEvent {
  uint8_t record_type;
  uint8_t keypad_address;
  uint8_t action;
  uint8_t record_data_length;
  uint8_t record_data[MAX_KEYPAD_DATA_LENGTH];
} PanelKeyboardEvent;

typedef struct alignas(1) ErrorCounters {
  uint32_t tx_soft_errors;
  uint32_t tx_hard_errors;
  uint32_t tx_buffer_pool_overflow_errors;
  uint32_t rx_bad_packets;
  uint32_t rx_frame_timeouts;
  uint32_t ecp_parity_errors;
  uint32_t ecp_checksum_errors;
  uint32_t pad;
} ErrorCounters;

typedef struct alignas(1) EchoCommand {
  uint8_t length;
  uint8_t data[8];
} EchoCommand;
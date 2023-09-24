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
#define ECP_DEBUG PA5

#define CONSOLE_RX PA10
#define CONSOLE_TX PA9

#define PANEL_RX PA3
#define PANEL_TX PA2

#define ECP_COMM_LED PC13
#define CBUS_COMM_LED PA0

#define DEBUG_PIN PA4


/*
* Enums used in communication with the panel
*/

enum {CHIME_NONE = 0, CHIME_ONCE, CHIME_TWICE, CHIME_THREE_TIMES, CHIME_FAST_REPEATING, CHIME_SLOW_REPEATING, CHIME_UNUSED, CHIME_LOUD};
enum {KEYPAD_RECORD_TYPE_PRESENT = 0, KEYPAD_RECORD_KEYS };
enum {COMMAND_ECHO = 0, COMMAND_RETRIEVE_ERROR_COUNTERS, COMMAND_UPDATE_KEYPAD};
enum {RTYPE_ECHO=0, RTYPE_SEND_ERROR_COUNTERS, RTYPE_UPDATE_KEYPAD};

/*
* Structs used in communication with the panel
*/


typedef struct PanelPacketAckNak {
    uint8_t type;
    uint8_t seq_num;
    uint8_t crc16_l;
    uint8_t crc16_h;
} __attribute__((aligned(1))) PanelPacketAckNak;


typedef struct PanelPacketHeader {
    uint8_t type;
    uint8_t seq_num;
    uint8_t payload_len;
} __attribute__((aligned(1))) PanelPacketHeader;


typedef struct RecordTypeHeader {
    uint8_t record_type;
    uint8_t data_length;
} __attribute__((aligned(1))) RecordTypeHeader;


typedef struct KeypadCommand{
    bool ready;
    bool armedAway;
    bool back_light;
    uint8_t keypad_address;
    uint8_t chime;
    uint8_t lenLine1;
    uint8_t lenLine2;
    uint8_t line1[MAX_KEYPAD_LINE];
    uint8_t line2[MAX_KEYPAD_LINE];

} __attribute__((aligned(1))) KeypadCommand;

typedef struct PanelKeyboardEvent {
    uint8_t record_type;
    uint8_t keypad_address;
    uint8_t action;
    uint8_t record_data_length;
    uint8_t record_data[MAX_KEYPAD_DATA_LENGTH];
} __attribute__((aligned(1))) PanelKeyboardEvent; 

typedef struct ErrorCounters {
    uint32_t tx_soft_errors;
    uint32_t tx_hard_errors;
    uint32_t tx_buffer_pool_overflow_errors;
    uint32_t rx_bad_packets;
    uint32_t rx_frame_timeouts;
    uint32_t pad[3];
} ErrorCounters;

typedef struct EchoCommand {
    uint8_t length;
    uint8_t data[8];
} EchoCommand;
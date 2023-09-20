#pragma once
#include <Arduino.h>

#define MAX_KEYPAD_LINE 16

#define ADDR_ALL_KEYPADS 255

#define MAX_CODE_LENGTH 4

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
* Enums
*/

enum {CHIME_NONE=0, CHIME_ONCE, CHIME_TWICE, CHIME_THREE_TIMES, CHIME_FAST_REPEATING, CHIME_SLOW_REPEATING, CHIME_UNUSED, CHIME_LOUD};
enum {KEYPAD_RECORD_TYPE_PRESENT=0, KEYPAD_RECORD_TYPE_CODE, KEYPAD_RECORD_TYPE_PANIC };


/*
* Structs
*/

typedef struct Keypad_Command{
    bool ready;
    bool armedAway;
    uint8_t keypad_address;
    uint8_t chime;
    uint8_t lenLine1;
    uint8_t lenLine2;
    uint8_t line1[MAX_KEYPAD_LINE];
    uint8_t line2[MAX_KEYPAD_LINE];

} __attribute__((aligned(1))) Keypad_Command;
#include <common.h>
#include <easylog.h>
#include <Console.h>
#include <Sequencer.h>

#define TAG console

 

void Console::begin(HardwareSerial *uart) {
    _uart = uart;
    LOG_DEBUG(TAG, "Console initialized");
}


void Console::loop() {
    if(_uart->available()) {
        char c = _uart->read();
        if(c == '\r'){
            _uart->print('\n');
        }
        _uart->print(c);
        int command = _parser.addChar(c);
        switch(command) {
            case NO_COMMAND:
                break;
            
            case SEND_MESSAGE:
                break;
        }

    }

}

/*
* Print message from keypad
*/

void Console::messageIn(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data, uint8_t action) {
    char rd[16];
    int clipped_length = (record_data_length <= sizeof(rd)-1) ? record_data_length : sizeof(rd)-1;

    if (record_type == KEYPAD_RECORD_TYPE_CODE) {
        // Convert data into zero terminated string
        memcpy(rd, record_data, clipped_length);
        rd[clipped_length] = 0;
            
        _uart->printf("Code from keypad keypad address %02X: Record type: %02X, Data length: %d, Action: %02X, Data: %s\r\n", 
            keypad_addr, record_type, record_data_length, action, rd);
    }
    else if (record_type == KEYPAD_RECORD_TYPE_PANIC) {
                _uart->printf("Panic from keypad keypad address %02X: Record type: %02X, Action: %02X\r\n", keypad_addr, record_type, action);
    }
    else if (record_type == KEYPAD_RECORD_TYPE_PRESENT) {
        char hex_string[3*7+3];
        int i, clipped_length = (record_data_length <= 7) ? record_data_length : 7;
        // Create hex string
        for (i = 0; i < clipped_length; i++) {
            snprintf((hex_string + (i*3)), 4, "%02X ", record_data[i]);
        }
        hex_string[((i-1)*3)+2] = 0;
        _uart->printf("Present message from keypad keypad address %02X: Record type: %02X, Data length: %d, Action: %02X, Data: %s\r\n",
            keypad_addr, record_type, record_type, action, hex_string);

        

    }

}
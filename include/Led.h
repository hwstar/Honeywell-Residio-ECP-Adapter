
#define LED_FLASH_TIME_MS 50

#pragma once

#include <Arduino.h>
#include <common.h>

class Led {
    private:
        bool _ecpLedActive;
        bool _cbusLedActive;
        uint32_t _ecpLedTimer;
        uint32_t _cbusLedTimer;

    public:
        void ecpFlash();
        void cbusFlash();
        void begin();
        void loop();

};


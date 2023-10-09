#include <Arduino.h>
#include <common.h>
#include <Led.h>

void _setLed(int gpio, bool state) { digitalWrite(gpio, !state); }

void Led::begin() {
  _ecpLedTimer = _cbusLedTimer = millis();
  _ecpLedActive = _cbusLedActive = false;
}

void Led::loop() {
  if ((_cbusLedActive == true) && (((uint32_t) millis()) - _cbusLedTimer) > LED_FLASH_TIME_MS) {
    _setLed(CBUS_COMM_LED, false);
    _cbusLedActive = false;
  }

  if ((_ecpLedActive == true) && (((uint32_t) millis()) - _ecpLedTimer) > LED_FLASH_TIME_MS) {
    _setLed(ECP_COMM_LED, false);
    _ecpLedActive = false;
  }
}

void Led::ecpFlash() {
  _ecpLedTimer = millis();
  _ecpLedActive = true;
  _setLed(ECP_COMM_LED, true);
}

void Led::cbusFlash() {
  _cbusLedTimer = millis();
  _cbusLedActive = true;
  _setLed(CBUS_COMM_LED, true);
}

#include <Arduino.h>
#include <EcpSoftwareSerial.h>
#include <HardwareSerial.h>
#include <Sequencer.h>
#include <Console.h>
#include <common.h>
#include <Panel.h>
#include <Led.h>

EcpSoftwareSerial ecp(ECP_RX, ECP_TX, true, true, ECP_DEBUG);  // UART for the Honeywell ECP bus
Sequencer seq;                                                 // Keypad sequencer
HardwareSerial SerialConsole(CONSOLE_RX, CONSOLE_TX);          // Console UART
HardwareSerial SerialPanel(PANEL_RX, PANEL_TX);                // Panel UART
Console console;
Panel panel;
Led led;

void message_callback(uint8_t record_type, uint8_t keypad_addr, uint8_t record_data_length, uint8_t *record_data,
                      uint8_t action) {
  panel.messageIn(record_type, keypad_addr, record_data_length, record_data, action);
  console.messageIn(record_type, keypad_addr, record_data_length, record_data, action);
}

void setSystemFreq() {
  RCC->CIR = 0x009F0000;

  // Turn HSE ON
  RCC->CR |= RCC_CR_HSEON;

  // Wait Until HSE Is Ready
  while (!(RCC->CR & RCC_CR_HSERDY))
    ;
  // serialPrint("HSE ON\n");

  // set HSE as system clock
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSE;

  // AHB prescaler
  RCC->CFGR &= ~(RCC_CFGR_HPRE);    // remove old prescaler
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // set AHB prescaler = 1.
  // set ADC prescaler = 8
  RCC->CFGR &= ~(RCC_CFGR_ADCPRE);
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;
  // set APB1 prescaler
  RCC->CFGR &= ~(RCC_CFGR_PPRE1);
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
  // set APB2 prescaler
  RCC->CFGR &= ~(RCC_CFGR_PPRE2);
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

  // set flash wait states to 2 wait states
  FLASH->ACR &= ~(FLASH_ACR_LATENCY);
  FLASH->ACR |= FLASH_ACR_LATENCY_2;

  // Set PLL Multiplier
  // RCC->CFGR |= 0b1100 << 18;
  // at HSE=8MHz, 8*9 = 72MHz.
  RCC->CFGR &= ~(RCC_CFGR_PLLMULL);
  RCC->CFGR |= RCC_CFGR_PLLMULL9;

  // //Set HSE as PLL Source. bit set -> HSE, bit unser -> HSI
  RCC->CFGR |= RCC_CFGR_PLLSRC;

  // Set HSE Prescaler On PLL Entry
  RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;
  RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE;  // no HSE prescaler before PLL entry

  // Turn On PLL Clock
  RCC->CR |= RCC_CR_PLLON;
  // serialPrint("wait for PLL ON\n");
  // ms_delay(1000);

  // Wait Until PLL Is Ready
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;
  // serialPrint("PLL rdy\n");

  // serialPrint("Clock ON\n");
  // ms_delay(1000);
  //  Set System To PLL CLock
  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
  // serialPrint("switched to PLL clock\n");

  // Clear All Interrupts
  RCC->CIR = 0x009F0000;
}

void setup() {
  setSystemFreq();

  // Outputs
  pinMode(ECP_COMM_LED, OUTPUT);
  pinMode(CBUS_COMM_LED, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);
  pinMode(DEBUG_PIN_ECP_PARITY_ERR, OUTPUT);
  pinMode(KEYPAD_POWER_ENA, OUTPUT);

  digitalWrite(ECP_COMM_LED, true);   // ECP comm led off
  digitalWrite(CBUS_COMM_LED, true);  // CBUS comm led off
  digitalWrite(DEBUG_PIN, false);
  digitalWrite(DEBUG_PIN_ECP_PARITY_ERR, false);
  digitalWrite(KEYPAD_POWER_ENA, false);

  SerialConsole.begin(115200);
  SerialPanel.begin(19200);
  led.begin();
  panel.begin(&SerialPanel);
  console.begin(&SerialConsole);
  ecp.setInterruptPriority(0, 0);
  ecp.begin();
  seq.begin(&ecp, message_callback);
}

void loop() {
  seq.update();
  panel.loop();
  console.loop();
  led.loop();
}

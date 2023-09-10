/*
 * ECPSoftwareSerial.h (formerly SoftwareSerial.h)
 *
 * Multi-instance software serial library for Arduino/Wiring
 * -- Interrupt-driven receive and other improvements by ladyada
 *    (http://ladyada.net)
 * -- Tuning, circular buffer, derivation from class Print/Stream,
 *    multi-instance support, porting to 8MHz processors,
 *    various optimizations, PROGMEM delay tables, inverse logic and
 *    direct port writing by Mikal Hart (http://www.arduiniana.org)
 * -- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
 * -- 20MHz processor support by Garrett Mace (http://www.macetech.com)
 * -- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)
 * -- Modified for Ademco ECP protocol by Steve Rodgers (http://github.com/hwstar)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * The latest version of this library can always be found at
 * http://arduiniana.org.
 */

/*
*
* Credit for reverse engineering and documenting the Honeywell ECP protocol goes to:
*
* Tom Vickers: https://github.com/jsb5151/Arduino2keypad/blob/master/docs/Keybus%20Protocol.pdf
*
*/

# pragma once

#include <Arduino.h>

/******************************************************************************
* Definitions
******************************************************************************/

//#define DEBUG_RX_SAMPLING // Enables Debug GPIO for RX pin sampling in ISR
//#define DEBUG_POLL_TICKS


#define _SS_MAX_RX_BUFF 64 // RX buffer size
#define TIMER_SERIAL TIM4  

#define OVERSAMPLE 3 // in RX, Timer will generate interruption OVERSAMPLE time during a bit. Thus OVERSAMPLE ticks in a bit. (interrupt not synchronized with edge).
#define ECP_BAUD_RATE 4800.0
#define TIME_ONE_TICK (1 / (ECP_BAUD_RATE * OVERSAMPLE))
#define ONE_BIT_TIME (1 / ECP_BAUD_RATE)
#define DELAY_POLL_LOW 1015.0E-6
#define DELAY_POLL_HIGH (2.0 * DELAY_POLL_LOW)
#define DELAY_POLL_KEYPADS 13000.0E-6
#define DELAY_TX_REQUEST 4000.0E-6
#define DELAY_ZERO_BYTE (9 * ONE_BIT_TIME)
#define DELAY_KEYPAD_POLL_TO_F6_MS 55 // Crucial for getting response from the keypad.
#define DELAY_POLL_INACTIVE_MS 400 // Time between polling events
#define DELAY_RX_TIMEOUT_MS 100


#define TICKS_TX_REQUEST ((unsigned int) (DELAY_TX_REQUEST / TIME_ONE_TICK))
#define TICKS_POLL_START ((unsigned int) (DELAY_POLL_KEYPADS / TIME_ONE_TICK))
#define TICKS_POLL_LOW ((unsigned int) (DELAY_POLL_LOW / TIME_ONE_TICK))
#define TICKS_POLL_HIGH ((unsigned int) (DELAY_POLL_HIGH / TIME_ONE_TICK))


enum {POLL_SM_IDLE=0, POLL_SM_START_SEQ, 
POLL_SM_WAIT_START_DONE, POLL_SM_SEND_ZERO_BYTE, 
POLL_SM_SEND_BYTE_DELAY, POLL_SM_START_TX_REQUEST,
POLL_SM_CHECK_TX_REQUEST};


class EcpSoftwareSerial {
  private:
    // per object data
    uint16_t _receivePin;
    uint16_t _transmitPin;
    uint16_t _debugPin;
    
    GPIO_TypeDef *_receivePinPort;
    uint32_t _receivePinNumber;
    GPIO_TypeDef *_transmitPinPort;
    uint32_t _transmitPinNumber;
    GPIO_TypeDef *_debugPinPort;
    uint32_t _debugPinNumber;

    uint16_t _buffer_overflow: 1;
    uint16_t _rx_inverse_logic: 1;
    uint16_t _tx_inverse_logic: 1;
    uint16_t _output_pending: 1;
    uint16_t _debug_pin_toggle_state: 1;
    
  

    unsigned char _receive_buffer[_SS_MAX_RX_BUFF];
    volatile uint8_t _receive_buffer_tail;
    volatile uint8_t _receive_buffer_head;
    bool parity;
    volatile bool rxParityError;
    volatile uint8_t tx_total_bits;
   

    static HardwareTimer timer_bit;
    static EcpSoftwareSerial *active_listener;
    static EcpSoftwareSerial *volatile active_out;
    static EcpSoftwareSerial *volatile active_in;
    static int32_t tx_tick_cnt;
    static volatile int32_t rx_tick_cnt;
    static uint32_t tx_buffer;
    static int32_t tx_bit_cnt;
    static uint32_t rx_buffer;
    static int32_t rx_bit_cnt;
    static volatile uint32_t startBitLength;
    static volatile uint8_t pollState;
    static volatile uint8_t pollResult;
    static volatile uint8_t pollByteCount;
    
    

    // private methods
    void setSpeed(uint32_t speed);
    void do_poll();
    void send();
    void recv();
    void setTX();
    void setRX();
    static void handleInterrupt();

  public:
    // public methods
    EcpSoftwareSerial(uint16_t receivePin, uint16_t transmitPin, bool rx_inverse_logic = false, bool tx_inverse_logic = false, uint16_t debugPin = 0);
    virtual ~EcpSoftwareSerial();
    void begin();
    bool listen();
    void end();
    bool isListening()
    {
      return active_listener == this;
    }
    bool stopListening();
    bool overflow()
    {
      bool ret = _buffer_overflow;
      if (ret) {
        _buffer_overflow = false;
      }
      return ret;
    }
    int peek();

    size_t write(uint8_t byte, bool first_byte = false);
    int read();
    int available();
    void rx_flush();


    void writeBytes(uint8_t *buffer, uint8_t length); 
   
    // Set parity
    void setParity(bool parity_enabled){
      noInterrupts();
      parity = parity_enabled;
      rxParityError = false;
      interrupts();
    }

    // ECPSpecific get parity error state, reset it, and return the state prior to reset

    bool getParityError(void) {
      noInterrupts();
      bool res = rxParityError;
      interrupts();
      rxParityError = false;
      return res;

    }

    // Initiate keypad polling sequence
    // Returns true if successful.
    bool initiateKeypadPollSequence();

    bool initiateNewCommand();

    // Return true if we are in the middle of polling the keypads
    bool getKeypadPollBusy() {
      noInterrupts();
      bool pollBusy =  (pollState != POLL_SM_IDLE);
      interrupts();
      return pollBusy;

    }
    // Sets the state (high or low) of the software uart TX pin
    void setTxPinState(bool state);

    // Calculate the checksum of a packet
    uint8_t calculateChecksum(uint8_t *packet, uint8_t length);


    // Return whether or not the transmitter is done transmitting a byte
    bool getTxDone();

    // Set the interrupt priority of the ECP software UART
    static void setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority);

};


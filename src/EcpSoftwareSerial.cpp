/*
 * SoftwareSerial.cpp (formerly NewSoftSerial.cpp)
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
 * -- STM32 support by Armin van der Togt
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

//
// Includes
//
#include "EcpSoftwareSerial.h"


//
// Static
//
HardwareTimer EcpSoftwareSerial::timer_bit(TIM4);
EcpSoftwareSerial *EcpSoftwareSerial::active_listener = nullptr;
EcpSoftwareSerial *volatile EcpSoftwareSerial::active_out = nullptr;
EcpSoftwareSerial *volatile EcpSoftwareSerial::active_in = nullptr;

int32_t EcpSoftwareSerial::tx_tick_cnt = 0; // OVERSAMPLE ticks needed for a bit
int32_t volatile EcpSoftwareSerial::rx_tick_cnt = 0;  // OVERSAMPLE ticks needed for a bit
uint32_t EcpSoftwareSerial::tx_buffer = 0;
int32_t EcpSoftwareSerial::tx_bit_cnt = 0;
uint32_t EcpSoftwareSerial::rx_buffer = 0;
int32_t EcpSoftwareSerial::rx_bit_cnt = -1; // rx_bit_cnt = -1 :  waiting for start bit





//
// Private methods
//

void EcpSoftwareSerial::setSpeed(uint32_t speed)
{

  timer_bit.pause();
  if (speed != 0) {
    // Disable the timer
    uint32_t clock_rate, cmp_value;
    // Get timer clock
    clock_rate = timer_bit.getTimerClkFreq();
    int pre = 1;
    // Calculate prescale an compare value
    do {
      cmp_value = clock_rate / (speed * OVERSAMPLE);
      if (cmp_value >= UINT16_MAX) {      rx_bit_cnt++; // Prepare for next bit
        rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
        clock_rate = clock_rate / 2;
        pre *= 2;
      }
    } while (cmp_value >= UINT16_MAX);
    timer_bit.setPrescaleFactor(pre);
    timer_bit.setOverflow(cmp_value);
    timer_bit.setCount(0);
    timer_bit.attachInterrupt(&handleInterrupt);
    timer_bit.resume();
  } else {
    timer_bit.detachInterrupt();
  }

}

// This function sets the current object as the "listening"
// one and returns true if it replaces another
bool EcpSoftwareSerial::listen()
{
  if (active_listener != this) {
    // wait for any transmit to complete as we may change speed
    while (active_out);
    if (active_listener != nullptr) {      rx_bit_cnt++; // Prepare for next bit
        rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
      active_listener->stopListening();
    }
    rx_tick_cnt = 1; // 1 : next interrupt will decrease rx_tick_cnt to 0 which means RX pin level will be considered.
    rx_bit_cnt = -1; // rx_bit_cnt = -1 :  waiting for start bit
    active_listener = this;
    active_in = this;
    return true;
  }
  return false;
}

// Stop listening. Returns true if we were actually listening.
bool EcpSoftwareSerial::stopListening()
{
  if (active_listener == this) {
    // wait for any output to complete
    while (active_out);
    active_listener = nullptr;
    active_in = nullptr;
    return true;
  }
  return false;
}

inline void EcpSoftwareSerial::setTX()
{
  if (_tx_inverse_logic) {
    LL_GPIO_ResetOutputPin(_transmitPinPort, _transmitPinNumber);
  } else {
    LL_GPIO_SetOutputPin(_transmitPinPort, _transmitPinNumber);
  }
  pinMode(_transmitPin, OUTPUT);
}

inline void EcpSoftwareSerial::setRX()
{
  pinMode(_receivePin, _tx_inverse_logic ? INPUT_PULLDOWN : INPUT_PULLUP); // pullup for normal logic!
}

//
// The transmit  routine called by the interrupt handler
//

inline void EcpSoftwareSerial::send()
{
  if (--tx_tick_cnt <= 0) { // if tx_tick_cnt > 0 interrupt is discarded. Only when tx_tick_cnt reach 0 we set TX pin.
    if (tx_bit_cnt++ < tx_total_bits) { // ECP-specific tx_bit_cnt: 11 (11: = 1 start + 8 bits + 2 stop) or 12: (1 start + 8 bits + even parity + 2 stop)
      // send data (including start and stop bits)
      if (tx_buffer & 1) {
        LL_GPIO_SetOutputPin(_transmitPinPort, _transmitPinNumber);
      } else {
        LL_GPIO_ResetOutputPin(_transmitPinPort, _transmitPinNumber);
      }
      tx_buffer >>= 1;
      tx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE tick to send next bit
    } else { // Transmission finished
      tx_tick_cnt = 1;
      if (_output_pending) {
        active_out = nullptr;
      } else if (tx_bit_cnt > tx_total_bits) {
        active_out = nullptr;
      }
    }
  }
}

//
// The receive routine called by the interrupt handler
//
inline void EcpSoftwareSerial::recv()
{

  if(parity == false){ // Case 2 stop bits, no parity
   
    if (--rx_tick_cnt <= 0) { // if rx_tick_cnt > 0 interrupt is discarded. Only when rx_tick_cnt reach 0 RX pin is considered
      if(_debugPin){
        // Toggle debug pin to show where the sample occured on the scope

      }
      bool inbit = LL_GPIO_IsInputPinSet(_receivePinPort, _receivePinNumber) ^ _rx_inverse_logic;
      if (rx_bit_cnt == -1) {  // rx_bit_cnt = -1 :  waiting for start bit
        if (!inbit) {
          #ifdef DEBUG_RX_SAMPLING
          if (_debugPin) {
            if(_debug_pin_toggle_state) {
              LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
            } else {
              LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
            }
            _debug_pin_toggle_state ^= 1;
          }
          #endif
          // got start bit
          rx_bit_cnt = 0; // rx_bit_cnt == 0 : start bit received
          rx_tick_cnt = OVERSAMPLE + 1; // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the bit cell (and not too close to the edge)
          rx_buffer = 0;
        } else {
          rx_tick_cnt = 1; // Waiting for start bit, but we don't get right level. Wait for next Interrupt to check RX pin level
        }
      } else if (rx_bit_cnt == 8) {
          rx_bit_cnt++; // Prepare for next bit
          rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
      } else if (rx_bit_cnt >= 9) { // rx_bit_cnt >= 8 : waiting for second stop bit
        #ifdef DEBUG_RX_SAMPLING
        if (_debugPin) {
          if(_debug_pin_toggle_state) {
            LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
          } else {
            LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
          }
          _debug_pin_toggle_state ^= 1;
        }
          #endif
        if (inbit) {
          // stop bit read complete add to buffer
          uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
          if (next != _receive_buffer_head) {
            // save new data in buffer: tail points to where byte goes
            _receive_buffer[_receive_buffer_tail] = rx_buffer; // save new byte
            _receive_buffer_tail = next;
          } else { // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
            _buffer_overflow = true;
          }
        }
        // Full frame received. Restart waiting for start bit at next interrupt
        rx_tick_cnt = 1;
        rx_bit_cnt = -1;
      } else {
      #ifdef DEBUG_RX_SAMPLING
      if (_debugPin) {
        if(_debug_pin_toggle_state) {
          LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
        } else {
          LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
        }
        _debug_pin_toggle_state ^= 1;
      }
      #endif
        // data bits
        rx_buffer >>= 1;
        if (inbit) {
          rx_buffer |= 0x80;
        }
        rx_bit_cnt++; // Prepare for next bit
        rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
      }
    }
  }
  else { // Case: 2 stop bits, with parity
    if (--rx_tick_cnt <= 0) { // if rx_tick_cnt > 0 interrupt is discarded. Only when rx_tick_cnt reaches 0 RX pin is considered
      bool inbit = LL_GPIO_IsInputPinSet(_receivePinPort, _receivePinNumber) ^ _rx_inverse_logic;
      if (rx_bit_cnt == -1) {  // rx_bit_cnt = -1 :  waiting for start bit
        if (!inbit) {
          // got start bit
          #ifdef DEBUG_RX_SAMPLING
          if (_debugPin) {
            if(_debug_pin_toggle_state) {
              LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
            } else {
              LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
            }
            _debug_pin_toggle_state ^= 1;
          }
          #endif
          rx_bit_cnt = 0; // rx_bit_cnt == 0 : start bit received
          rx_tick_cnt = OVERSAMPLE + 1; // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the bit cell (and not too close to the edge)
          rx_buffer = 0;
        } else {
          rx_tick_cnt = 1; // Waiting for start bit, but we don't get right level. Wait for next Interrupt to check RX pin level
        }
      } else if (rx_bit_cnt == 8) { // check parity bit
          uint8_t cp;
          #ifdef DEBUG_RX_SAMPLING
          if (_debugPin) {
            if(_debug_pin_toggle_state) {
              LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
            } else {
              LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
            }
            _debug_pin_toggle_state ^= 1;
          }
          #endif
          // Calculate parity bit
          cp = rx_buffer;
          cp = cp ^ (cp >> 4 | cp << 4);
          cp = cp ^ (cp >> 2);
          cp = cp ^ (cp >> 1);
          cp &= 1; 
          if(cp != inbit){
            rxParityError = true;
          }
          rx_bit_cnt++; // Prepare for next bit
          rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit


      } else if (rx_bit_cnt == 9)  { // first stop bit
          #ifdef DEBUG_RX_SAMPLING
          if (_debugPin) {
            if(_debug_pin_toggle_state) {
              LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
            } else {
              LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
            }
            _debug_pin_toggle_state ^= 1;
          }
          #endif
        rx_bit_cnt++; // Prepare for next bit
        rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit

      } else if (rx_bit_cnt >= 10) { // rx_bit_cnt >= 10 :  second stop bit
        #ifdef DEBUG_RX_SAMPLING
        if (_debugPin) {
          if(_debug_pin_toggle_state) {
            LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
          } else {
            LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
          }
            _debug_pin_toggle_state ^= 1;
        }
          #endif
        if (inbit) {
          // stop bit read complete add to buffer
          uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
          if (next != _receive_buffer_head) {
            // save new data in buffer: tail points to where byte goes
            _receive_buffer[_receive_buffer_tail] = rx_buffer; // save new byte
            _receive_buffer_tail = next;
          } else { // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
            _buffer_overflow = true;
          }
        }
        // Full frame received. Restart waiting for start bit at next interrupt
        rx_tick_cnt = 1;
        rx_bit_cnt = -1;
      } else {
        #ifdef DEBUG_RX_SAMPLING
        if (_debugPin) {
          if(_debug_pin_toggle_state) {
            LL_GPIO_SetOutputPin(_debugPinPort, _debugPinNumber);
          } else {
            LL_GPIO_ResetOutputPin(_debugPinPort, _debugPinNumber);
          }
          _debug_pin_toggle_state ^= 1;
        }
        #endif
        // data bits
        rx_buffer >>= 1;
        if (inbit) {
          rx_buffer |= 0x80;
        }
        rx_bit_cnt++; // Prepare for next bit
        rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
      }
    }
  }

}




//
// Interrupt handling
//

/* static */
inline void EcpSoftwareSerial::handleInterrupt()
{
  if (active_in) {
    active_in->recv();
  }
  if (active_out) {
    active_out->send();
  }

}
//
// Constructor
//
EcpSoftwareSerial::EcpSoftwareSerial(uint16_t receivePin, uint16_t transmitPin, bool _rx_inverse_logic /* = false */, bool _tx_inverse_logic /* = false */, uint16_t debugPin /* = 0 */) :
  _receivePin(receivePin),
  _transmitPin(transmitPin),
  _debugPin(debugPin),
  _receivePinPort(digitalPinToPort(receivePin)),
  _receivePinNumber(STM_LL_GPIO_PIN(digitalPinToPinName(receivePin))),
  _transmitPinPort(digitalPinToPort(transmitPin)),
  _transmitPinNumber(STM_LL_GPIO_PIN(digitalPinToPinName(transmitPin))),
  _buffer_overflow(false),
  _rx_inverse_logic(_rx_inverse_logic),
  _tx_inverse_logic(_tx_inverse_logic),

  _output_pending(0),
  _receive_buffer_tail(0),
  _receive_buffer_head(0)
{

  // Set up debug pin if requested to do so
  if(_debugPin){
    _debugPinPort = digitalPinToPort(_debugPin);
    _debugPinNumber = STM_LL_GPIO_PIN(digitalPinToPinName(debugPin));
    if (set_GPIO_Port_Clock(STM_PORT(digitalPinToPinName(debugPin))) == 0) {
      _Error_Handler("ERROR: invalid debug pin number\n", -1);
    }

  }
  
  /* Enable GPIO clock for tx and rx pin*/
  if (set_GPIO_Port_Clock(STM_PORT(digitalPinToPinName(transmitPin))) == 0) {
    _Error_Handler("ERROR: invalid transmit pin number\n", -1);
  }
  if ((set_GPIO_Port_Clock(STM_PORT(digitalPinToPinName(receivePin))) == 0)) {
    _Error_Handler("ERROR: invalid receive pin number\n", -1);
  }


}

//
// Destructor
//
EcpSoftwareSerial::~EcpSoftwareSerial()
{
  end();
}

//
// Public methods
//

void EcpSoftwareSerial::begin()
{
  // Set debug pin mode if defined as something other than 0
  if(_debugPin){
    pinMode(_debugPin, OUTPUT);
  }
  setSpeed(4800);
  setTX();
  setRX();
  listen();
}

void EcpSoftwareSerial::end()
{
  stopListening();
}

// Read data from buffer
int EcpSoftwareSerial::read()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail) {
    return -1;
  }

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int EcpSoftwareSerial::available()
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t EcpSoftwareSerial::write(uint8_t b)
{
  // wait for previous transmit to complete
  _output_pending = 1;
  
  // Block if transmitting
  while (active_out)
    ;


  if(parity){ // Case for parity enabled
    // Calculate parity bit
    uint8_t cp = b;
    cp = cp ^ (cp >> 4 | cp << 4);
    cp = cp ^ (cp >> 2);
    cp = cp ^ (cp >> 1);
    cp &= 1;
    // cp will be 1 if b is odd parity
    // Adding this to the transmitted bits
    // will result in even parity.

    // Add start, data, parity and stop bits.
    tx_buffer = (b << 1) | (cp << 9) | 0xc00;
    tx_total_bits = 12;
  } else {  // Case for parity disabled
    // Add start, data, and stop bits.
    tx_buffer = (b << 1) | 0x600;
    tx_total_bits = 11;
  }
  // If output inverted
  if (_rx_inverse_logic) {
    tx_buffer = ~tx_buffer;
  }
  tx_bit_cnt = 0;
  tx_tick_cnt = OVERSAMPLE;
  _output_pending = 0;
  // make us active
  active_out = this;
  return 1;
}

void EcpSoftwareSerial::rx_flush()
{
  noInterrupts();
  _receive_buffer_head = _receive_buffer_tail = 0;
  interrupts();
}

int EcpSoftwareSerial::peek()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail) {
    return -1;
  }

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

void EcpSoftwareSerial::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority)
{
  timer_bit.setInterruptPriority(preemptPriority, subPriority);
}



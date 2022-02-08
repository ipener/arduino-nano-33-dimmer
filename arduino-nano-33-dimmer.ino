/*
MIT License

Copyright (c) 2022 Igor Pener

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This minimal hardware timer implementation works out of the box for all
Arduino Nano 33 boards. It was tested with the RobotDyn 4-channel AC dimmer
module wired up as follows:

| Arduino | Dimmer |
|---------|--------|
| GND     | GND    |
| 3.3V    | VCC    |
| D2      | Z-C    |
| D3      | D1     |
| D4      | D2     |
| D5      | D3     |
| D6      | D4     |
*/

#include <stdint.h>

// Note: Comment out this line if 60Hz is preferred
#define NETWORK_FREQUENCY_50HZ

#if defined(NETWORK_FREQUENCY_50HZ)
#define NETWORK_SEMIPERIOD 10000
#else
#define NETWORK_SEMIPERIOD 8333
#endif


#if defined(ARDUINO_ARCH_MBED)

#include <nrf_timer.h>

#define TIMER_IRQ   TIMER3_IRQn
#define IRQ_HANDLER TIMER3_IRQHandler_v

#elif defined(ARDUINO_ARCH_SAMD)

#define TIMER_IRQ   TC3_IRQn
#define IRQ_HANDLER TC3_Handler

// 8000000 MHz / 10000000 us / 2 = 4
#define us_to_tick(us) (uint16_t)(4 * (us))
#define digitalPinToPinName(p) (p)
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6

#else
#error "Only ARDUINO_ARCH_MBED and ARDUINO_ARCH_SAMD are supported"
#endif

#define LIGHT_D3 0x1
#define LIGHT_D4 0x2
#define LIGHT_D5 0x4
#define LIGHT_D6 0x8
// Note: Add the desired lights to this bitmask `ENABLED_LIGHTS_MASK`
#define ENABLED_LIGHTS_MASK (LIGHT_D3 | LIGHT_D4 | LIGHT_D5)

struct light_t {
#if defined(ARDUINO_ARCH_MBED)
  const    PinName pin;
#else
  const    uint8_t pin;
#endif
  const    uint8_t bit;
  volatile uint8_t brightness;
};

#ifndef ENABLED_LIGHTS_MASK
#error "You need to #define ENABLED_LIGHTS_MASK (LIGHT_D3 | LIGHT_D4 | ...)"
#endif

#if (ENABLED_LIGHTS_MASK & LIGHT_D3)
static light_t light_D3 = { digitalPinToPinName(D3), LIGHT_D3, 0 };
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D4)
static light_t light_D4 = { digitalPinToPinName(D4), LIGHT_D4, 0 };
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D5)
static light_t light_D5 = { digitalPinToPinName(D5), LIGHT_D5, 0 };
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D6)
static light_t light_D6 = { digitalPinToPinName(D6), LIGHT_D6, 0 };
#endif

volatile uint8_t _mask = (LIGHT_D3 | LIGHT_D4 | LIGHT_D5 | LIGHT_D6);
volatile uint8_t _current_brightness = 0;

#define UPDATE_CURRENT_BRIGHTNESS(light)\
  if (!(_mask & light.bit)) {\
    _current_brightness = max(_current_brightness, light.brightness);\
    digitalWrite(light.pin, LOW);\
  }

#define UPDATE_NEXT_BRIGHTNESS(light)\
  if (light.brightness >= _current_brightness) {\
    digitalWrite(light.pin, HIGH);\
  } else if (light.brightness > 0) {\
    next_brightness = min(next_brightness, light.brightness);\
  }

void zc_isr() {
  if (_mask == (LIGHT_D3 | LIGHT_D4 | LIGHT_D5 | LIGHT_D6)) return;

  _current_brightness = 0;

#if (ENABLED_LIGHTS_MASK & LIGHT_D3)
  UPDATE_CURRENT_BRIGHTNESS(light_D3);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D4)
  UPDATE_CURRENT_BRIGHTNESS(light_D4);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D5)
  UPDATE_CURRENT_BRIGHTNESS(light_D5);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D6)
  UPDATE_CURRENT_BRIGHTNESS(light_D6);
#endif

#if defined(ARDUINO_ARCH_MBED)
  NRF_TIMER3->CC[0] = NETWORK_SEMIPERIOD - (uint32_t)_current_brightness * NETWORK_SEMIPERIOD / UINT8_MAX;
  NRF_TIMER3->TASKS_START = 1;
#else
  TC3->COUNT16.COUNT.reg = 0;
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);

  TC3->COUNT16.CC[0].reg = us_to_tick(NETWORK_SEMIPERIOD - (uint32_t)_current_brightness * NETWORK_SEMIPERIOD / UINT8_MAX);
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);

  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);
#endif

  NVIC_EnableIRQ(TIMER_IRQ);
}

extern "C" void IRQ_HANDLER(void) {
  NVIC_DisableIRQ(TIMER_IRQ);

#if defined(ARDUINO_ARCH_MBED)
  if (NRF_TIMER3->EVENTS_COMPARE[0] == 1) {
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
#else
  if (TC3->COUNT16.INTFLAG.bit.MC0 == 1) {
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;
#endif

    uint8_t next_brightness = _current_brightness;

#if (ENABLED_LIGHTS_MASK & LIGHT_D3)
    UPDATE_NEXT_BRIGHTNESS(light_D3);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D4)
    UPDATE_NEXT_BRIGHTNESS(light_D4);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D5)
    UPDATE_NEXT_BRIGHTNESS(light_D5);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D6)
    UPDATE_NEXT_BRIGHTNESS(light_D6);
#endif

    if (next_brightness < _current_brightness) {
#if defined(ARDUINO_ARCH_MBED)
      NRF_TIMER3->CC[0] = NETWORK_SEMIPERIOD * (uint32_t)(_current_brightness - next_brightness) / UINT8_MAX;
#else
      TC3->COUNT16.COUNT.reg = 0;
      while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);

      TC3->COUNT16.CC[0].reg = us_to_tick(NETWORK_SEMIPERIOD * (uint32_t)(_current_brightness - next_brightness) / UINT8_MAX);
      while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);

      TC3->COUNT16.CTRLA.bit.ENABLE = 1;
      while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);
#endif
      _current_brightness = next_brightness;
    } else {
#if defined(ARDUINO_ARCH_MBED)
      NRF_TIMER3->TASKS_STOP = 1;
#else
      TC3->COUNT16.CTRLA.bit.ENABLE = 0;
#endif
      return;
    }
  }
  NVIC_EnableIRQ(TIMER_IRQ);
}

void set_brightness(struct light_t *light, uint8_t brightness) {
  if (!light || light->brightness == brightness) return;

  light->brightness = brightness;

  if (brightness == 0) {
    _mask |= light->bit;
    digitalWrite(light->pin, LOW);
  } else if (brightness == UINT8_MAX) {
    _mask |= light->bit;
    digitalWrite(light->pin, HIGH);
  } else {
    _mask &= ~light->bit;
  }
}


void setup() {
  #if defined(ARDUINO_ARCH_MBED)
  NRF_TIMER3->BITMODE   = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER3->MODE      = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  NRF_TIMER3->PRESCALER = 4UL << TIMER_PRESCALER_PRESCALER_Pos; // f = 16Mhz / 2^prescaler = 1Mhz
  NRF_TIMER3->INTENSET  = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  NRF_TIMER3->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
#else
  SYSCTRL->OSC8M.bit.PRESC = 0;
  SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ENABLE;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3;
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK7;
  GCLK->CLKCTRL.bit.CLKEN = 1;

  GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_OSC8M_Val;
  GCLK->GENCTRL.bit.ID = 0x07;
  GCLK->GENCTRL.bit.GENEN = 1;

  GCLK->GENDIV.bit.ID = 0x07;
  GCLK->GENDIV.bit.DIV = 0;

  PM->APBCSEL.bit.APBCDIV = 0;
  PM->APBCMASK.bit.TC3_ = 1;

  TC3->COUNT16.CTRLA.bit.MODE = 0;
  TC3->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV2_Val;
  TC3->COUNT16.CTRLBCLR.bit.DIR = 1;

  TC3->COUNT16.CTRLC.bit.CPTEN0 = 0;
  TC3->COUNT16.INTENSET.bit.MC0 = 1;
#endif

  pinMode(digitalPinToInterrupt(D2), INPUT);
  attachInterrupt(digitalPinToInterrupt(D2), zc_isr, RISING);

#if (ENABLED_LIGHTS_MASK & LIGHT_D3)
  pinMode(D3, OUTPUT);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D4)
  pinMode(D4, OUTPUT);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D5)
  pinMode(D5, OUTPUT);
#endif
#if (ENABLED_LIGHTS_MASK & LIGHT_D6)
  pinMode(D6, OUTPUT);
#endif
}

void loop() {
  set_brightness(&light_D3,   0);
  set_brightness(&light_D4, 150);
  set_brightness(&light_D5, 255);
  delay(2000);

  set_brightness(&light_D3,   1);
  set_brightness(&light_D4,  75);
  set_brightness(&light_D5, 254);
  delay(2000);

  set_brightness(&light_D3, 255);
  set_brightness(&light_D4,  50);
  set_brightness(&light_D5, 100);
  delay(2000);

  set_brightness(&light_D3,   0);
  set_brightness(&light_D4, 255);
  set_brightness(&light_D5,   0);
  delay(2000);
}


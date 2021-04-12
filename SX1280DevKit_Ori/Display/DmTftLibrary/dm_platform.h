#ifndef DM_PLATFORM_h
#define DM_PLATFORM_h

// Determine type of system
#if defined (__AVR__)
  #define DM_TOOLCHAIN_ARDUINO
#elif defined(TOOLCHAIN_ARM) || defined(TOOLCHAIN_ARM_MICRO)
  #define DM_TOOLCHAIN_MBED
#else
  #error Only Arduino and Mbed toolchains are supported
#endif

// Arduino
#if defined (DM_TOOLCHAIN_ARDUINO)

  // Mandatory includes for Arduino
  #include <Arduino.h>
  #include <avr/pgmspace.h>
  #include <SPI.h>

  // Clear bit, Set bit, High pulse and Low pulse macros
  #define cbi(reg, _bitmask) *reg &= ~_bitmask
  #define sbi(reg, _bitmask) *reg |= _bitmask
  #define pulse_high(reg, _bitmask) sbi(reg, _bitmask); cbi(reg, _bitmask);
  #define pulse_low(reg, _bitmask) cbi(reg, _bitmask); sbi(reg, _bitmask);

  // Map of mandatory pin names, from Arduino names to D* and A*
  #define D2   2
  #define D3   3
  #define D4   4
  #define D5   5
  #define D6   6
  #define D9   9
  #define D10 10
  #define A2  16
  #define A3  17
  #define A4  18
  #define A5  19

  // Needed typedefs, not normally present in the Arduino environment
  #ifndef uint8_t
    #define uint8_t unsigned char
  #endif
  #ifndef int8_t
    #define int8_t signed char
  #endif
  #ifndef uint16_t
    #define uint16_t unsigned short
  #endif
  #ifndef uint32_t
    #define uint32_t unsigned long
  #endif

// Mbed
#elif defined(DM_TOOLCHAIN_MBED)

  // Mandatory includes for Mbed
  #include "mbed.h"

  // Clear bit, Set bit, High pulse, Low pulse, Boundary limits and Delay macros
  #define sbi(reg, _bitmask) (*(reg) = 1)
  #define cbi(reg, _bitmask) (*(reg) = 0)
  #define pulse_high(reg, _bitmask) do { *(reg) = 1; *(reg) = 0; } while(0)
  #define pulse_low(reg, _bitmask) do { *(reg) = 0; *(reg) = 1; } while(0)
  #define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>(high)?(high):(amt)))
  #define delay(ms) wait_ms(ms)

  // On Arduino no extra delay is needed, but on faster platforms the simulated
  // SPI bus may get a too high frequency so a delay here will lower it. This
  // delay should ideally be configurable per platform but for now it will have
  // to be ok to lower the frequency to 500KHz  
  #if defined(__LPC407x_8x_177x_8x_H__)
    //#define slow_pulse_delay()  wait_us(1)
    #define slow_pulse_delay() do {\
      volatile unsigned _xyz = 10; \
      for (; _xyz > 0; _xyz--); \
    } while(0)
  #else
    #define slow_pulse_delay()
  #endif
  #define slow_pulse_high(reg, _bitmask) do {\
       *(reg) = 1;    \
       slow_pulse_delay(); \
       *(reg) = 0;    \
       slow_pulse_delay(); \
    } while(0)
  #define slow_pulse_low(reg, _bitmask) do {\
       *(reg) = 0;    \
       slow_pulse_delay(); \
       *(reg) = 1;    \
       slow_pulse_delay(); \
    } while(0)

  // Special handling for the LPC1549 LPCXpresso board with solder bump
  #ifdef LPC15XX_H
    #define SPECIAL_D5  P0_11
  #else
    #define SPECIAL_D5  D5
  #endif
#endif

#endif /* DM_PLATFORM_h */


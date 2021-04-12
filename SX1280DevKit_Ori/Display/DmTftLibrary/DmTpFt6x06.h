/**********************************************************************************************
 Copyright (c) 2015 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#ifndef DM_TPFT6X06_h
#define DM_TPFT6X06_h

#include "dm_platform.h"
#if defined (DM_TOOLCHAIN_ARDUINO)
#include <Wire.h>
#elif defined (DM_TOOLCHAIN_MBED)
#include "mbed.h"
#endif

#if defined (DM_TOOLCHAIN_ARDUINO)
#define FT6x06_ADDR                   0x38
#elif defined (DM_TOOLCHAIN_MBED)
#define FT6x06_ADDR                   0x70
#endif

#define FT6x06_DEV_MODE               0x00
#define FT6x06_GEST_ID                0x01
#define FT6x06_TD_STATUS              0x02
#define FT6x06_TH_GROUP               0x80
#define FT6x06_PERIODACTIVE           0x88
#define FT6x06_CHIPER                 0xA3
#define FT6x06_G_MODE                 0xA4
#define FT6x06_FIRMID                 0xA6
#define FT6x06_FOCALTECH_ID           0xA8
#define FT6x06_RELEASE_CODE_ID        0xAF

#define WAIT_ACK_TIMEOUT              100
#define COM_OK                        0
#define ACK_ERROR                     1   

class DmTpFt6x06
{
public:
  enum Display {
    DM_TFT28_116 = 116,
  };
 
#if defined (DM_TOOLCHAIN_ARDUINO)  
  DmTpFt6x06(Display disp, bool useIrq=true);
#elif defined (DM_TOOLCHAIN_MBED)
  DmTpFt6x06(Display disp, I2C &i2c, bool useIrq=true);
#endif  
  void init();
  void readTouchData(uint16_t& posX, uint16_t& posY, bool& touching);
  bool readTouchData(uint16_t& posX, uint16_t& posY);
  bool isTouched();
  
private:
 uint8_t readRegister8(uint8_t reg);  
 void writeRegister8(uint8_t reg, uint8_t val); 
  void enableIrq();
#if defined (DM_TOOLCHAIN_ARDUINO)
  int8_t _irq; 
  regtype *_pinIrq;
  regsize _bitmaskIrq;
#elif defined (DM_TOOLCHAIN_MBED)
  PinName _irq;
  long map(long x, long in_min, long in_max, long out_min, long out_max);
  DigitalIn *_pinIrq;
  I2C *_i2c;
#endif
};
#endif

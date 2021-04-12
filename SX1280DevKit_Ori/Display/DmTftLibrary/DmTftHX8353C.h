/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#ifndef DM_TFT_HX8353C_h
#define DM_TFT_HX8353C_h

#include "DmTftBase.h"

class DmTftHX8353C : public DmTftBase
{
public:
#if defined (DM_TOOLCHAIN_ARDUINO)
  DmTftHX8353C(uint8_t mosi=D2, uint8_t clk=D3, uint8_t cs=D4, uint8_t dc=D5, uint8_t rst=D6);
#elif defined (DM_TOOLCHAIN_MBED)
  DmTftHX8353C(PinName mosi, PinName clk, PinName cs, PinName dc, PinName rst);
#endif
  virtual ~DmTftHX8353C();
  virtual void init(void);
private:
  void send8BitData(uint8_t data);
  void writeBus(uint8_t data);

  virtual void setAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
  virtual void sendCommand(uint8_t index);
  virtual void sendData(uint16_t data);

  
  static const uint16_t _width;
  static const uint16_t _height;
#if defined (DM_TOOLCHAIN_ARDUINO)
  uint8_t _mosi, _clk, _cs, _dc, _rst;
  regtype *_pinDC, *_pinRST, *_pinMOSI, *_pinCLK;
  regsize _bitmaskDC, _bitmaskRST, _bitmaskMOSI, _bitmaskCLK;
#elif defined (DM_TOOLCHAIN_MBED)
  PinName _mosi, _clk, _cs, _dc, _rst;
  DigitalOut* _pinDC, *_pinRST, *_pinMOSI, *_pinCLK;
#endif
};
#endif




/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#ifndef DM_TFT_ILI9325_h
#define DM_TFT_ILI9325_h

#include "DmTftBase.h"

class DmTftIli9325 : public DmTftBase
{
public:
#if defined (DM_TOOLCHAIN_ARDUINO)
  DmTftIli9325(uint8_t wr=A4, uint8_t cs=A3, uint8_t dc=A5, uint8_t rst=A2);
#elif defined (DM_TOOLCHAIN_MBED)  
  DmTftIli9325(PinName wr, PinName cs, PinName dc, PinName rst);
#endif   
  virtual ~DmTftIli9325();
  virtual void init(void);
private:
  void send8BitData(uint8_t data);
  void writeBus(uint8_t data);

  virtual void setAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  virtual void sendCommand(uint8_t index);
  virtual void sendData(uint16_t data);

  
  static const uint16_t _width;
  static const uint16_t _height;
#if defined (DM_TOOLCHAIN_ARDUINO)
  uint8_t _wr, _cs, _dc, _rst;
  regtype *_pinDC, *_pinRST, *_pinWR;
  regsize _bitmaskDC, _bitmaskRST, _bitmaskWR;
#elif defined (DM_TOOLCHAIN_MBED)
  PinName _wr, _cs, _dc, _rst;
  DigitalOut* _pinDC, *_pinRST, *_pinWR;
  BusOut * _virtualPortD;
#endif  
};
#endif




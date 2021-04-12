/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#ifndef DM_TFT_ILI9341_h
#define DM_TFT_ILI9341_h

#include "DmTftBase.h"

class DmTftIli9341 : public DmTftBase
{
public:
#if defined (DM_TOOLCHAIN_ARDUINO)
  DmTftIli9341(uint8_t cs, uint8_t dc);
#elif defined (DM_TOOLCHAIN_MBED)
  DmTftIli9341(PinName cs, PinName dc, PinName mosi, PinName miso, PinName clk);
#endif
  virtual ~DmTftIli9341();
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
  uint8_t _cs, _dc;
  regtype *_pinDC;
  regsize _bitmaskDC;
  uint8_t _spiSettings;
#elif defined (DM_TOOLCHAIN_MBED)
  PinName _cs, _dc;
  DigitalOut *_pinDC;
  SPI spi;
#endif
};


#endif




/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#ifndef DM_TOUCH_h
#define DM_TOUCH_h

#include "dm_platform.h"

typedef struct calibrationMatrix {
  int32_t  a, b, c,	d, e, f;
} CalibrationMatrix;

class DmTouch
{
public:
  enum Display {
    DM_TFT28_103 = 103,
    DM_TFT24_104 = 104,
    DM_TFT28_105 = 105,
	DM_TFT35_107 = 107,
	DM_TFT43_108 = 108,
	DM_TFT50_111 = 111
  };

  enum SpiMode {
	Auto,
	Software,
	Hardware
  };
  
  enum TouchId{
    IC_8875 = 0x8875,
	IC_2046 = 0x2046		
  };

#if defined (DM_TOOLCHAIN_ARDUINO)
  DmTouch(Display disp, SpiMode spiMode=Auto, bool useIrq=true);
#elif defined (DM_TOOLCHAIN_MBED)
  DmTouch(Display disp, PinName mosi=D11, PinName miso=D12, PinName clk=D13);
#endif
  void init();
  void readTouchData(uint16_t& posX, uint16_t& posY, bool& touching);
  bool isTouched();
  bool getMiddleXY(uint16_t &x, uint16_t &y); // Raw Touch Data, used for calibration
  void setCalibrationMatrix(CalibrationMatrix calibrationMatrix);
  void setPrecison(uint8_t samplesPerMeasurement);
  void waitForTouch();
  void waitForTouchRelease();
  uint32_t rescaleFactor() { return 1000000; };
  Display getDisplay() { return _disp; };

private:
  void spiWrite(uint8_t data);
  uint8_t spiRead();
  uint16_t readData12(uint8_t command);
  void enableIrq();
  void readRawData(uint16_t &x, uint16_t &y);
  void getAverageXY(uint16_t &x, uint16_t &y);
  uint16_t getDisplayCoordinateX(uint16_t x_touch, uint16_t y_touch);
  uint16_t getDisplayCoordinateY(uint16_t x_touch, uint16_t y_touch);
  uint16_t calculateMiddleValue(uint16_t values[], uint8_t count);
  bool isSampleValid();

  Display _disp;
  uint16_t _width, _height;
  bool _hardwareSpi;
  uint8_t _samplesPerMeasurement;
  CalibrationMatrix _calibrationMatrix;
  uint16_t _touch_id;

#if defined (DM_TOOLCHAIN_ARDUINO)
  uint8_t _cs, _clk, _mosi, _miso,_irq;;
  regtype *_pinDC, *_pinCS, *_pinCLK, *_pinMOSI, *_pinMISO, *_pinIrq;
  regsize _bitmaskDC, _bitmaskCS, _bitmaskCLK, _bitmaskMOSI, _bitmaskMISO, _bitmaskIrq;
  uint8_t _spiSettings;
#elif defined (DM_TOOLCHAIN_MBED)
  PinName _cs, _clk, _mosi, _miso, _irq;
  DigitalOut *_pinDC, *_pinCS, *_pinCLK, *_pinMOSI, *_led;
  DigitalIn *_pinMISO, *_pinIrq;
  SPI *_spi;
#endif

};
#endif

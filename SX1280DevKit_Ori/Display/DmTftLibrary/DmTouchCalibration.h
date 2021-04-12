/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#ifndef DM_TOUCHCALIBRATION_h
#define DM_TOUCHCALIBRATION_h

#include "dm_platform.h"
#include "DmTouch.h"
#include "DmTftBase.h"

typedef struct point {
  uint16_t x, y;
} Point;

class DmTouchCalibration
{
public:
  DmTouchCalibration(DmTftBase *tftBase, DmTouch *touch);
  bool getTouchReferencePoints(Point displayRefPoint[], Point touchRefPoint[], uint16_t tftWidth, uint16_t tftHeight);
  CalibrationMatrix calculateCalibrationMatrix(Point displayRefPoint[], Point touchRefPoint[]);
  static CalibrationMatrix getDefaultCalibrationData(DmTouch::Display disp);
  void drawCalibPoint(uint16_t x, uint16_t y) const;
  void drawCalibration(CalibrationMatrix calibrationMatrix);
  void printCalibration(CalibrationMatrix calibrationMatrix) const;
private:
  DmTftBase* _tft;
  DmTouch* _touch;
  uint16_t _textRow;
  void printHeadline(char* text);
  void printNumber(int32_t number) const ;
  void releaseTouch(uint16_t x, uint16_t y) const;
  bool getRawTouch(uint16_t& x, uint16_t& y) const;
};
#endif

/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

#include "DmTouchCalibration.h"

DmTouchCalibration::DmTouchCalibration(DmTftBase *tftBase, DmTouch *touch) {
  _tft = tftBase;
  _touch = touch;
  _textRow = 20;
}

// For best result, create a calibration for each display
CalibrationMatrix DmTouchCalibration::getDefaultCalibrationData(DmTouch::Display disp) {
  CalibrationMatrix calibrationMatrix = {0};
  switch (disp) {
    case DmTouch::DM_TFT28_103:
      calibrationMatrix.a = 67548;    //    63787;
      calibrationMatrix.b = -625;     //     -138;
      calibrationMatrix.c = -16854644;//-15921157;
      calibrationMatrix.d = 362;      //     -244;
      calibrationMatrix.e = 89504;    //    89313;
      calibrationMatrix.f = -14380636;//-10726623;
      break;

    case DmTouch::DM_TFT24_104:
      calibrationMatrix.a = -71855;
      calibrationMatrix.b = 2147;
      calibrationMatrix.c = 259719524;
      calibrationMatrix.d = -1339;
      calibrationMatrix.e = -91012;
      calibrationMatrix.f = 354268832;
      break;

    case DmTouch::DM_TFT28_105:
      calibrationMatrix.a = 65521;
      calibrationMatrix.b = -253;
      calibrationMatrix.c = -11813673;
      calibrationMatrix.d = -439;
      calibrationMatrix.e = 89201;
      calibrationMatrix.f = -10450920;
      break;

    case DmTouch::DM_TFT35_107:
      calibrationMatrix.a = 91302;    //    85984;
      calibrationMatrix.b = 817;      //      451;
      calibrationMatrix.c = -26296117;//-16494041;
      calibrationMatrix.d = -1877;    //     2308;
      calibrationMatrix.e = 73762;    //    65173;
      calibrationMatrix.f = -26384255;//-19179080;
      break;
    case DmTouch::DM_TFT43_108:   // or DM_TFT43_110
      calibrationMatrix.a = 541307;
      calibrationMatrix.b = -4288;
      calibrationMatrix.c = -36678732;
      calibrationMatrix.d = 2730;
      calibrationMatrix.e = 321714;
      calibrationMatrix.f = -31439472;    
      break;
    case DmTouch::DM_TFT50_111:   // or  DM_TFT50_112
      calibrationMatrix.a = 875894;
      calibrationMatrix.b = 1655;
      calibrationMatrix.c = -53695309;
      calibrationMatrix.d = -993;
      calibrationMatrix.e = 544421;
      calibrationMatrix.f = -41496753;                    
      break;   
    default:
      break;
  }
  return calibrationMatrix;
}


bool DmTouchCalibration::getTouchReferencePoints(Point displayRefPoint[], Point touchRefPoint[], uint16_t tftWidth, uint16_t tftHeight) {
  _touch->setPrecison(60);
  displayRefPoint[0].x = (tftWidth / 2);
  displayRefPoint[0].y = (tftHeight / 2);
  displayRefPoint[1].x = 1 * (tftWidth / 5);
  displayRefPoint[1].y = 1 * (tftHeight / 5);
  displayRefPoint[2].x = 4 * (tftWidth / 5);
  displayRefPoint[2].y = 1 * (tftHeight / 5);
  displayRefPoint[3].x = 4 * (tftWidth / 5);
  displayRefPoint[3].y = 4 * (tftHeight / 5);
  displayRefPoint[4].x = 1 * (tftWidth / 5);
  displayRefPoint[4].y = 4 * (tftHeight / 5);

  for(int n = 0; n < 5; n++) {
    drawCalibPoint(displayRefPoint[n].x, displayRefPoint[n].y);
    if (!getRawTouch(touchRefPoint[n].x, touchRefPoint[n].y)) {
      return false;
    }
    releaseTouch(displayRefPoint[n].x, displayRefPoint[n].y);
  }

  _touch->setPrecison(3);
  return true;
}

bool DmTouchCalibration::getRawTouch(uint16_t& x, uint16_t& y) const {
  bool touched = false;
  bool haveAllSamples = false;

  while(!touched) {
    if (_touch->isTouched()) {
      //Serial.println("is touched");
      haveAllSamples = _touch->getMiddleXY(x, y);
        
      // As many samples are take during calibration it is important to
      // make sure that the screen is still touched. If the user has let
      // go already then the gathered samples are not good enough.
      touched = haveAllSamples && _touch->isTouched();
    }
  }
  return haveAllSamples;
}

void DmTouchCalibration::drawCalibPoint(uint16_t x, uint16_t y) const {
  const uint8_t lineSize = 5;
  _tft->drawHorizontalLine(x-lineSize, y, lineSize*2, RED);
  _tft->drawVerticalLine(x, y-lineSize, lineSize*2, RED);
}

void DmTouchCalibration::releaseTouch(uint16_t x, uint16_t y) const {
  _tft->drawCircle(x, y, 10, YELLOW);
  delay(100);
  _tft->fillCircle(x, y, 10, GREEN);
  delay(100);
  _tft->fillCircle(x, y, 10, BLACK);
  delay(300);
  _touch->waitForTouchRelease();
}

CalibrationMatrix DmTouchCalibration::calculateCalibrationMatrix(Point displayRefPoint[], Point touchRefPoint[]) {
  double A = 0.0, B = 0.0, C = 0.0, D = 0.0, E = 0.0, F = 0.0;
  double d = 0.0, dx1 = 0.0, dx2 = 0.0, dx3 = 0.0, dy1 = 0.0, dy2 = 0.0, dy3 = 0.0;
  uint32_t X2_1 = 0, X2_2 = 0, X2_3 = 0, X2_4 = 0, X2_5 = 0;
  uint32_t Y2_1 = 0, Y2_2 = 0, Y2_3 = 0, Y2_4 = 0, Y2_5 = 0;
  uint32_t XxY_1 = 0, XxY_2 = 0, XxY_3 = 0, XxY_4 = 0, XxY_5 = 0;
  uint32_t XxXd_1 = 0, XxXd_2 = 0, XxXd_3 = 0, XxXd_4 = 0, XxXd_5 = 0;
  uint32_t YxXd_1 = 0, YxXd_2 = 0, YxXd_3 = 0, YxXd_4 = 0, YxXd_5 = 0;
  uint32_t XxYd_1 = 0, XxYd_2 = 0, XxYd_3 = 0, XxYd_4 = 0, XxYd_5 = 0;
  uint32_t YxYd_1 = 0, YxYd_2 = 0, YxYd_3 = 0, YxYd_4 = 0, YxYd_5 = 0;
  uint32_t alfa = 0, beta = 0, chi = 0, Kx = 0, Ky = 0, Lx = 0, Ly = 0;
  uint16_t epsilon = 0, fi = 0, Mx = 0, My = 0;

  X2_1 = ((uint32_t)touchRefPoint[0].x * touchRefPoint[0].x);
  X2_2 = ((uint32_t)touchRefPoint[1].x * touchRefPoint[1].x);
  X2_3 = ((uint32_t)touchRefPoint[2].x * touchRefPoint[2].x);
  X2_4 = ((uint32_t)touchRefPoint[3].x * touchRefPoint[3].x);
  X2_5 = ((uint32_t)touchRefPoint[4].x * touchRefPoint[4].x);

  Y2_1 = ((uint32_t)touchRefPoint[0].y * touchRefPoint[0].y);
  Y2_2 = ((uint32_t)touchRefPoint[1].y * touchRefPoint[1].y);
  Y2_3 = ((uint32_t)touchRefPoint[2].y * touchRefPoint[2].y);
  Y2_4 = ((uint32_t)touchRefPoint[3].y * touchRefPoint[3].y);
  Y2_5 = ((uint32_t)touchRefPoint[4].y * touchRefPoint[4].y);

  XxY_1 = ((uint32_t)touchRefPoint[0].x * touchRefPoint[0].y);
  XxY_2 = ((uint32_t)touchRefPoint[1].x * touchRefPoint[1].y);
  XxY_3 = ((uint32_t)touchRefPoint[2].x * touchRefPoint[2].y);
  XxY_4 = ((uint32_t)touchRefPoint[3].x * touchRefPoint[3].y);
  XxY_5 = ((uint32_t)touchRefPoint[4].x * touchRefPoint[4].y);

  XxXd_1 = ((uint32_t)touchRefPoint[0].x * displayRefPoint[0].x);
  XxXd_2 = ((uint32_t)touchRefPoint[1].x * displayRefPoint[1].x);
  XxXd_3 = ((uint32_t)touchRefPoint[2].x * displayRefPoint[2].x);
  XxXd_4 = ((uint32_t)touchRefPoint[3].x * displayRefPoint[3].x);
  XxXd_5 = ((uint32_t)touchRefPoint[4].x * displayRefPoint[4].x);

  YxXd_1 = ((uint32_t)touchRefPoint[0].y * displayRefPoint[0].x);
  YxXd_2 = ((uint32_t)touchRefPoint[1].y * displayRefPoint[1].x);
  YxXd_3 = ((uint32_t)touchRefPoint[2].y * displayRefPoint[2].x);
  YxXd_4 = ((uint32_t)touchRefPoint[3].y * displayRefPoint[3].x);
  YxXd_5 = ((uint32_t)touchRefPoint[4].y * displayRefPoint[4].x);

  XxYd_1 = ((uint32_t)touchRefPoint[0].x * displayRefPoint[0].y);
  XxYd_2 = ((uint32_t)touchRefPoint[1].x * displayRefPoint[1].y);
  XxYd_3 = ((uint32_t)touchRefPoint[2].x * displayRefPoint[2].y);
  XxYd_4 = ((uint32_t)touchRefPoint[3].x * displayRefPoint[3].y);
  XxYd_5 = ((uint32_t)touchRefPoint[4].x * displayRefPoint[4].y);

  YxYd_1 = ((uint32_t)touchRefPoint[0].y * displayRefPoint[0].y);
  YxYd_2 = ((uint32_t)touchRefPoint[1].y * displayRefPoint[1].y);
  YxYd_3 = ((uint32_t)touchRefPoint[2].y * displayRefPoint[2].y);
  YxYd_4 = ((uint32_t)touchRefPoint[3].y * displayRefPoint[3].y);
  YxYd_5 = ((uint32_t)touchRefPoint[4].y * displayRefPoint[4].y);

  alfa = X2_1 + X2_2 + X2_3 + X2_4 + X2_5;
  beta = Y2_1 + Y2_2 + Y2_3 + Y2_4 + Y2_5;
  chi = XxY_1 + XxY_2 + XxY_3 + XxY_4 + XxY_5;
  epsilon = touchRefPoint[0].x + touchRefPoint[1].x + touchRefPoint[2].x + touchRefPoint[3].x + touchRefPoint[4].x;
  fi = touchRefPoint[0].y + touchRefPoint[1].y + touchRefPoint[2].y + touchRefPoint[3].y + touchRefPoint[4].y;
  Kx = XxXd_1 + XxXd_2 + XxXd_3 + XxXd_4 + XxXd_5;
  Ky = XxYd_1 + XxYd_2 + XxYd_3 + XxYd_4 + XxYd_5;
  Lx = YxXd_1 + YxXd_2 + YxXd_3 + YxXd_4 + YxXd_5;
  Ly = YxYd_1 + YxYd_2 + YxYd_3 + YxYd_4 + YxYd_5;
  Mx = displayRefPoint[0].x + displayRefPoint[1].x + displayRefPoint[2].x + displayRefPoint[3].x + displayRefPoint[4].x;
  My = displayRefPoint[0].y + displayRefPoint[1].y + displayRefPoint[2].y + displayRefPoint[3].y + displayRefPoint[4].y;
  d = 5 * ( ((double)alfa * beta) - ((double)chi * chi) ) + 2 * ((double)chi * epsilon * fi) - ((double)alfa * fi * fi ) - ( (double)beta * epsilon * epsilon );
  dx1 = 5 * ( ((double)Kx * beta) - ((double)Lx * chi) ) + ((double)fi * ( ((double)Lx * epsilon) - ((double)Kx * fi) )) + ((double)Mx * ( ((double)chi * fi) - ((double)beta * epsilon) ));
  dx2 = 5 * ( ((double)Lx * alfa) - ((double)Kx * chi) ) + ((double)epsilon * ( ((double)Kx * fi) - ((double)Lx * epsilon) )) + ((double)Mx * ( ((double)chi * epsilon) - ((double)alfa * fi) ));
  dx3 = ((double)Kx * ( ((double)chi * fi) - ((double)beta * epsilon) )) + ((double)Lx * ( ((double)chi * epsilon) - ((double)alfa * fi) )) + ((double)Mx * ( ((double)alfa * beta) - ((double)chi * chi) ));
  dy1 = 5 * ( ((double)Ky * beta) - ((double)Ly * chi) ) + ((double)fi * ( ((double)Ly * epsilon) - ((double)Ky * fi) )) + ((double)My * ( ((double)chi * fi) - ((double)beta * epsilon) ));
  dy2 = 5 * ( ((double)Ly * alfa) - ((double)Ky * chi) ) + ((double)epsilon * ( ((double)Ky * fi) - ((double)Ly * epsilon) )) + ((double)My * ( ((double)chi * epsilon) - ((double)alfa * fi) ));
  dy3 = ((double)Ky * ( ((double)chi * fi) - ((double)beta * epsilon) )) + ((double)Ly * ( ((double)chi * epsilon) - ((double)alfa * fi) )) + ((double)My * ( ((double)alfa * beta) - ((double)chi * chi) ));

  A = dx1 / d;
  B = dx2 / d;
  C = dx3 / d;
  D = dy1 / d;
  E = dy2 / d;
  F = dy3 / d;

  CalibrationMatrix calibrationData;
  calibrationData.a = (int32_t)(A * _touch->rescaleFactor());
  calibrationData.b = (int32_t)(B * _touch->rescaleFactor());
  calibrationData.c = (int32_t)(C * _touch->rescaleFactor());
  calibrationData.d = (int32_t)(D * _touch->rescaleFactor());
  calibrationData.e = (int32_t)(E * _touch->rescaleFactor());
  calibrationData.f = (int32_t)(F * _touch->rescaleFactor());
  return calibrationData;
}

void DmTouchCalibration::drawCalibration(CalibrationMatrix calibrationMatrix) {
  printHeadline("CalibrationData: ");
  printHeadline("a: ");
  printNumber(calibrationMatrix.a);
  printHeadline("b: ");
  printNumber(calibrationMatrix.b);
  printHeadline("c: ");
  printNumber(calibrationMatrix.c);
  printHeadline("d: ");
  printNumber(calibrationMatrix.d);
  printHeadline("e: ");
  printNumber(calibrationMatrix.e);
  printHeadline("f: ");
  printNumber(calibrationMatrix.f);
}

void DmTouchCalibration::printHeadline(char* text) {
  _textRow += 20;
  _tft->drawString(5, _textRow, text);
}

void DmTouchCalibration::printNumber(int32_t number) const {
  _tft->drawNumber(20, _textRow, number, 10, false);
}

void DmTouchCalibration::printCalibration(CalibrationMatrix calibrationMatrix) const {
#if defined (DM_TOOLCHAIN_ARDUINO)
  Serial.println(F("-------- Calibration data ------"));
  Serial.println(F("CalibrationMatrix calibrationMatrix;"));
  Serial.print(F("calibrationMatrix.a = "));
  Serial.print(calibrationMatrix.a);
  Serial.println(F(";"));
  Serial.print(F("calibrationMatrix.b = "));
  Serial.print(calibrationMatrix.b);
  Serial.println(F(";"));
  Serial.print(F("calibrationMatrix.c = "));
  Serial.print(calibrationMatrix.c);
  Serial.println(F(";"));
  Serial.print(F("calibrationMatrix.d = "));
  Serial.print(calibrationMatrix.d);
  Serial.println(F(";"));
  Serial.print(F("calibrationMatrix.e = "));
  Serial.print(calibrationMatrix.e);
  Serial.println(F(";"));
  Serial.print(F("calibrationMatrix.f = "));
  Serial.print(calibrationMatrix.f);
  Serial.println(F(";"));
  Serial.println("-------------------------------");
#endif
}

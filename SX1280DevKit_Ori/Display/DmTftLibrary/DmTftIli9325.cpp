/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/
 
 //Tested on NUCLEO-F401RE, LPCXpresso11U68 platform.
 
#include "DmTftIli9325.h"

#if defined (DM_TOOLCHAIN_ARDUINO)
DmTftIli9325::DmTftIli9325(uint8_t wr, uint8_t cs, uint8_t dc, uint8_t rst) : DmTftBase(240, 320)
#elif defined (DM_TOOLCHAIN_MBED)
DmTftIli9325::DmTftIli9325(PinName wr, PinName cs, PinName dc, PinName rst) : DmTftBase(240, 320)
#endif
{
    _wr = wr;
    _cs = cs;
    _dc = dc;
    _rst = rst;
}

DmTftIli9325::~DmTftIli9325() {
#if defined (DM_TOOLCHAIN_MBED)  
  delete _pinRST;
  delete _pinCS;
  delete _pinWR;
  delete _pinDC;
  delete _virtualPortD;
  _pinRST = NULL;
  _pinCS = NULL;
  _pinWR = NULL;
  _pinDC = NULL;
  _virtualPortD = NULL;
#endif  
}

void DmTftIli9325::writeBus(uint8_t data) {
#if defined (DM_TOOLCHAIN_ARDUINO)
  PORTD = data;
#elif defined (DM_TOOLCHAIN_MBED)
  *_virtualPortD = data;  
  //if(data & 0x10) 
#endif
  pulse_low(_pinWR, _bitmaskWR);
}

void DmTftIli9325::sendCommand(uint8_t index) {
  cbi(_pinDC, _bitmaskDC);
  writeBus(0x00);
  writeBus(index);
}

void DmTftIli9325::sendData(uint16_t data) {
  sbi(_pinDC, _bitmaskDC);
  writeBus(data>>8);
  writeBus(data);
}

void DmTftIli9325::setAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  sendCommand(0x50); // Set Column
  sendData(x0);
  sendCommand(0x51);
  sendData(x1);

  sendCommand(0x52);  // Set Page
  sendData(y0);
  sendCommand(0x53);
  sendData(y1);

  sendCommand(0x20);
  sendData(x0);
  sendCommand(0x21);
  sendData(y0);
  sendCommand(0x22);
}

void DmTftIli9325::init(void) {
  setTextColor(BLACK, WHITE);
#if defined (DM_TOOLCHAIN_ARDUINO)
/**************************************
      DM-DmTftIli932522-102       Arduino UNO      NUM

      RST                       A2                    16
      CS                        A3                     17
      WR                       A4                     18
      RS                        A5                     19

      DB8                       0                       0
      DB9                       1                       1
      DB10                      2                      2
      DB11                      3                      3
      DB12                      4                      4
      DB13                      5                      5
      DB14                      6                      6
      DB15                      7                      7

***************************************/
  DDRD = DDRD | B11111111;  // SET PORT D AS OUTPUT

  _pinRST = portOutputRegister(digitalPinToPort(_rst));
  _bitmaskRST = digitalPinToBitMask(_rst);
  _pinCS = portOutputRegister(digitalPinToPort(_cs));
  _bitmaskCS = digitalPinToBitMask(_cs);
  _pinWR = portOutputRegister(digitalPinToPort(_wr));
  _bitmaskWR = digitalPinToBitMask(_wr);
  _pinDC = portOutputRegister(digitalPinToPort(_dc));
  _bitmaskDC = digitalPinToBitMask(_dc);

  pinMode(_rst, OUTPUT);
  pinMode(_cs, OUTPUT);
  pinMode(_wr, OUTPUT);
  pinMode(_dc,OUTPUT);
#elif defined (DM_TOOLCHAIN_MBED)
  _pinRST = new DigitalOut(_rst);
  _pinCS = new DigitalOut(_cs);
  _pinWR = new DigitalOut(_wr);
  _pinDC = new DigitalOut(_dc);
  _virtualPortD = new BusOut(D0, D1, D2, D3, D4, SPECIAL_D5, D6, D7); // LPC15XX_H
#endif

  sbi(_pinRST, _bitmaskRST);

  delay(5);
  cbi(_pinRST, _bitmaskRST);

  delay(15);
  sbi(_pinRST, _bitmaskRST);
  sbi(_pinCS, _bitmaskCS);
  sbi(_pinWR, _bitmaskWR);

  delay(15);
  cbi(_pinCS, _bitmaskCS);



  sendCommand(0xE5); sendData(0x78F0);
  sendCommand(0x01); sendData(0x0100);
  sendCommand(0x02); sendData(0x0700);
  sendCommand(0x03); sendData(0x1030);
  sendCommand(0x04); sendData(0x0000);
  sendCommand(0x08); sendData(0x0207);
  sendCommand(0x09); sendData(0x0000);
  sendCommand(0x0A); sendData(0x0000);
  sendCommand(0x0C); sendData(0x0000);
  sendCommand(0x0D); sendData(0x0000);
  sendCommand(0x0F); sendData(0x0000);

  sendCommand(0x10); sendData(0x0000);
  sendCommand(0x11); sendData(0x0007);
  sendCommand(0x12); sendData(0x0000);
  sendCommand(0x13); sendData(0x0000);

  sendCommand(0x10); sendData(0x1290);
  sendCommand(0x11); sendData(0x0227);
  sendCommand(0x12); sendData(0x001D);
  sendCommand(0x13); sendData(0x1500);

  sendCommand(0x29); sendData(0x0018);
  sendCommand(0x2B); sendData(0x000D);

  sendCommand(0x30); sendData(0x0004);
  sendCommand(0x31); sendData(0x0307);
  sendCommand(0x32); sendData(0x0002);
  sendCommand(0x35); sendData(0x0206);
  sendCommand(0x36); sendData(0x0408);
  sendCommand(0x37); sendData(0x0507);
  sendCommand(0x38); sendData(0x0204);
  sendCommand(0x39); sendData(0x0707);
  sendCommand(0x3C); sendData(0x0405);
  sendCommand(0x3D); sendData(0x0f02);

  sendCommand(0x50); sendData(0x0000);
  sendCommand(0x51); sendData(0x00EF);
  sendCommand(0x52); sendData(0x0000);
  sendCommand(0x53); sendData(0x013F);
  sendCommand(0x60); sendData(0xA700);
  sendCommand(0x61); sendData(0x0001);
  sendCommand(0x6A); sendData(0x0000);

  sendCommand(0x80); sendData(0x0000);
  sendCommand(0x81); sendData(0x0000);
  sendCommand(0x82); sendData(0x0000);
  sendCommand(0x83); sendData(0x0000);
  sendCommand(0x84); sendData(0x0000);
  sendCommand(0x85); sendData(0x0000);

  sendCommand(0x90); sendData(0x0010);
  sendCommand(0x92); sendData(0x0600);
  sendCommand(0x93); sendData(0x0003);
  sendCommand(0x95); sendData(0x0110);
  sendCommand(0x97); sendData(0x0000);
  sendCommand(0x98); sendData(0x0000);
  sendCommand(0x07); sendData(0x0133);
  sbi(_pinCS, _bitmaskCS);
  delay(500);
  clearScreen();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/


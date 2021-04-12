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

#include "DmTftHX8353C.h"
#if defined (DM_TOOLCHAIN_ARDUINO)
DmTftHX8353C::DmTftHX8353C(uint8_t mosi, uint8_t clk, uint8_t cs, uint8_t dc, uint8_t rst) : DmTftBase(128,160) 
#elif defined (DM_TOOLCHAIN_MBED)
DmTftHX8353C::DmTftHX8353C(PinName mosi, PinName clk, PinName cs, PinName dc, PinName rst) : DmTftBase(128,160) 
#endif
{
  _mosi = mosi;
  _clk = clk;
  _cs = cs;
  _dc = dc;
  _rst = rst;
}

DmTftHX8353C::~DmTftHX8353C() {
#if defined (DM_TOOLCHAIN_MBED)
  delete _pinMOSI;
  delete _pinCLK;
  delete _pinCS;
  delete _pinDC;
  delete _pinRST;

  _pinMOSI = NULL;
  _pinCLK = NULL;
  _pinCS = NULL;
  _pinDC = NULL;
  _pinRST = NULL;
#endif
}

void DmTftHX8353C::writeBus(uint8_t data) {
  if (data & 0x80) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x40) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x20) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x10) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x08) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x04) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x02) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);

  if (data & 0x01) {
    sbi(_pinMOSI, _bitmaskMOSI);
  }
  else {
    cbi(_pinMOSI, _bitmaskMOSI);
  }
  pulse_low(_pinCLK, _bitmaskCLK);
}

void DmTftHX8353C::sendCommand(uint8_t index) {
  cbi(_pinDC, _bitmaskDC);
  writeBus(index);
}

void DmTftHX8353C::send8BitData(uint8_t data) {
  sbi(_pinDC, _bitmaskDC);
  writeBus(data);
}

void DmTftHX8353C::sendData(uint16_t data) {
  sbi(_pinDC, _bitmaskDC);
  writeBus(data>>8);
  writeBus(data);
}

void DmTftHX8353C::setAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  sendCommand(0x2A); // SetColumn
  sendData(x0);
  sendData(x1);
  sendCommand(0x2B); // SetPage
  sendData(y0);
  sendData(y1);

  sendCommand(0x2c);
}


void DmTftHX8353C::init (void) {
  setTextColor(BLACK, WHITE);
#if defined (DM_TOOLCHAIN_ARDUINO)
  _pinMOSI= portOutputRegister(digitalPinToPort(_mosi));
  _bitmaskMOSI= digitalPinToBitMask(_mosi);
  _pinCLK= portOutputRegister(digitalPinToPort(_clk));
  _bitmaskCLK= digitalPinToBitMask(_clk);
  _pinCS  = portOutputRegister(digitalPinToPort(_cs));
  _bitmaskCS  = digitalPinToBitMask(_cs);
  _pinDC  = portOutputRegister(digitalPinToPort(_dc));
  _bitmaskDC  = digitalPinToBitMask(_dc);
  _pinRST = portOutputRegister(digitalPinToPort(_rst));
  _bitmaskRST = digitalPinToBitMask(_rst);

  pinMode(_mosi,OUTPUT);
  pinMode(_clk,OUTPUT);
  pinMode(_cs,OUTPUT);
  pinMode(_dc,OUTPUT);
  pinMode(_rst,OUTPUT);
#elif defined (DM_TOOLCHAIN_MBED)
  _pinMOSI = new DigitalOut(_mosi);
  _pinCLK = new DigitalOut(_clk);
  _pinCS = new DigitalOut(_cs);
  _pinDC = new DigitalOut(_dc);
  _pinRST = new DigitalOut(_rst);
#endif

  sbi(_pinRST, _bitmaskRST);
  delay(5);
  cbi(_pinRST, _bitmaskRST);
  delay(15);
  sbi(_pinRST, _bitmaskRST);
  delay(15);
  cbi(_pinCS, _bitmaskCS);

  sendCommand(0xB9);             // HX8353C INIT
  send8BitData(0xFF);
  send8BitData(0x83);
  send8BitData(0x53);

  sendCommand(0xB0);
  send8BitData(0x3C);
  send8BitData(0X01);

  sendCommand(0xB6);
  send8BitData(0x94);
  send8BitData(0x6C);
  send8BitData(0x50);

  sendCommand(0xB1);
  send8BitData(0x00);
  send8BitData(0x01);
  send8BitData(0x1B);
  send8BitData(0x03);
  send8BitData(0X01);
  send8BitData(0X08);
  send8BitData(0x77);
  send8BitData(0x89);

  sendCommand(0xE0);
  send8BitData(0x50);
  send8BitData(0x77);
  send8BitData(0X40);
  send8BitData(0X08);
  send8BitData(0xBE);
  send8BitData(0x00);
  send8BitData(0x03);
  send8BitData(0x0F);
  send8BitData(0x00);
  send8BitData(0x01);
  send8BitData(0x73);
  send8BitData(0x00);
  send8BitData(0x72);
  send8BitData(0x03);
  send8BitData(0xB0);
  send8BitData(0x0F);
  send8BitData(0x08);
  send8BitData(0x00);
  send8BitData(0x0F);
  sendCommand(0x3A);
  send8BitData(0x05);
  sendCommand(0x36);
  send8BitData(0xC0);
  sendCommand(0x11);
  delay(150);
  sendCommand(0x29);
  delay(150);
  sendCommand(0x2D);
  uint8_t i=0;

  for(i=0;i<32;i++) {
    send8BitData(2*i);
  } //Red

  for(i=0;i<64;i++) {
    send8BitData(1*i);
  } //Green

  for(i=0;i<32;i++)
  {
    send8BitData(2*i);
  } //Blue
  sendCommand(0x2C);

  delay(150);
  clearScreen();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/



/**********************************************************************************************
 Copyright (c) 2014 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/

//Tested on NUCLEO-F401RE, LPCXpresso11U68, LPCXpresso824-MAX platform.

#include "DmTftIli9341.h"
#if defined (DM_TOOLCHAIN_ARDUINO)
DmTftIli9341::DmTftIli9341(uint8_t cs, uint8_t dc)
:DmTftBase(240, 320)
#elif defined (DM_TOOLCHAIN_MBED)
DmTftIli9341::DmTftIli9341(PinName cs, PinName dc, PinName mosi, PinName miso, PinName clk)
:DmTftBase(240, 320), spi(mosi, miso, clk)
#endif
{
  _cs = cs;
  _dc = dc;
}

DmTftIli9341::~DmTftIli9341() {
#if defined (DM_TOOLCHAIN_MBED)
delete _pinCS;
delete _pinDC;

_pinCS = NULL;
_pinDC = NULL;
#endif
}

void DmTftIli9341::writeBus(uint8_t data) {
#if defined (DM_TOOLCHAIN_ARDUINO)
  SPCR = _spiSettings;         // SPI Control Register
  SPDR = data;                 // SPI Data Register
  while(!(SPSR & _BV(SPIF)));  // SPI Status Register Wait for transmission to finish
#elif defined (DM_TOOLCHAIN_MBED)
  spi.write(data);
#endif
}

void DmTftIli9341::sendCommand(uint8_t index) {
  cbi(_pinDC, _bitmaskDC);
  writeBus(index);
}

void DmTftIli9341::send8BitData(uint8_t data) {
  sbi(_pinDC, _bitmaskDC);
  writeBus(data);
}

void DmTftIli9341::sendData(uint16_t data) {
  uint8_t dh = data>>8;
  uint8_t dl = data&0xff;

  sbi(_pinDC, _bitmaskDC);
  writeBus(dh);
  writeBus(dl);
}

void DmTftIli9341::setAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  sendCommand(0x2A); // Set Column
  sendData(x0);
  sendData(x1);

  sendCommand(0x2B);  // Set Page
  sendData(y0);
  sendData(y1);

  sendCommand(0x2c);
}

void DmTftIli9341::init(void) {
  setTextColor(BLACK, WHITE);
#if defined (DM_TOOLCHAIN_ARDUINO)
  _pinCS  = portOutputRegister(digitalPinToPort(_cs));
  _bitmaskCS  = digitalPinToBitMask(_cs);
  _pinDC  = portOutputRegister(digitalPinToPort(_dc));
  _bitmaskDC  = digitalPinToBitMask(_dc);
  pinMode(_cs,OUTPUT);
  pinMode(_dc,OUTPUT);

  sbi(_pinCS, _bitmaskCS);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  _spiSettings = SPCR;
#elif defined (DM_TOOLCHAIN_MBED)
  _pinCS = new DigitalOut(_cs);
  _pinDC = new DigitalOut(_dc);
  sbi(_pinCS, _bitmaskCS);
  
  spi.format(8,0);
#if defined( TARGET_NUCLEO_L476RG )
  spi.frequency(16000000); // Max SPI speed for display is 10 and for 17 for LPC15xx
#else
  spi.frequency(8000000);
#endif
#endif
  cbi(_pinCS, _bitmaskCS);
  delay(135); // This much delay needed??

  // ILI9341 init
  sendCommand(0x11);
  delay(120);

  sendCommand(0xCF);
  send8BitData(0x00);
  send8BitData(0xc3);
  send8BitData(0X30);

  sendCommand(0xED);
  send8BitData(0x64);
  send8BitData(0x03);
  send8BitData(0X12);
  send8BitData(0X81);

  sendCommand(0xE8);
  send8BitData(0x85);
  send8BitData(0x10);
  send8BitData(0x79);

  sendCommand(0xCB);
  send8BitData(0x39);
  send8BitData(0x2C);
  send8BitData(0x00);
  send8BitData(0x34);
  send8BitData(0x02);

  sendCommand(0xF7);
  send8BitData(0x20);

  sendCommand(0xEA);
  send8BitData(0x00);
  send8BitData(0x00);

  sendCommand(0xC0);    //Power control
  send8BitData(0x22);   //VRH[5:0]

  sendCommand(0xC1);    //Power control
  send8BitData(0x11);   //SAP[2:0];BT[3:0]

  sendCommand(0xC5);    //VCM control
  send8BitData(0x3d);
  send8BitData(0x20);

  sendCommand(0xC7);    //VCM control2
  send8BitData(0xAA); //0xB0

  sendCommand(0x36);    // Memory Access Control
#if 1  // DISPLAY_INVERT
  send8BitData(0xC8);
#else
  send8BitData(0x08);
#endif

  sendCommand(0x3A);
  send8BitData(0x55);

  sendCommand(0xB1);
  send8BitData(0x00);
  send8BitData(0x13);

  sendCommand(0xB6);    // Display Function Control
  send8BitData(0x0A);
  send8BitData(0xA2);

  sendCommand(0xF6);
  send8BitData(0x01);
  send8BitData(0x30);

  sendCommand(0xF2);    // 3Gamma Function Disable
  send8BitData(0x00);

  sendCommand(0x26);    //Gamma curve selected
  send8BitData(0x01);

  sendCommand(0xE0);    //Set Gamma
  send8BitData(0x0F);
  send8BitData(0x3F);
  send8BitData(0x2F);
  send8BitData(0x0C);
  send8BitData(0x10);
  send8BitData(0x0A);
  send8BitData(0x53);
  send8BitData(0XD5);
  send8BitData(0x40);
  send8BitData(0x0A);
  send8BitData(0x13);
  send8BitData(0x03);
  send8BitData(0x08);
  send8BitData(0x03);
  send8BitData(0x00);

  sendCommand(0XE1);    //Set Gamma
  send8BitData(0x00);
  send8BitData(0x00);

  send8BitData(0x10);
  send8BitData(0x03);
  send8BitData(0x0F);
  send8BitData(0x05);
  send8BitData(0x2C);
  send8BitData(0xA2);
  send8BitData(0x3F);
  send8BitData(0x05);
  send8BitData(0x0E);
  send8BitData(0x0C);
  send8BitData(0x37);
  send8BitData(0x3C);
  send8BitData(0x0F);

  sendCommand(0x11);    //Exit Sleep
  delay(120);
  sendCommand(0x29);    //Display on
  delay(50);
  sbi(_pinCS, _bitmaskCS);
  clearScreen();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/



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

#include "DmTftSsd2119.h"
#if defined (DM_TOOLCHAIN_ARDUINO)
DmTftSsd2119::DmTftSsd2119(uint8_t cs, uint8_t dc)
:DmTftBase(320, 240)
#elif defined (DM_TOOLCHAIN_MBED)
DmTftSsd2119::DmTftSsd2119(PinName cs, PinName dc, PinName mosi, PinName miso, PinName clk)
:DmTftBase(320, 240), spi(mosi, miso, clk)
#endif
{
  _cs = cs;
  _dc = dc;
}

DmTftSsd2119::~DmTftSsd2119() {
#if defined (DM_TOOLCHAIN_MBED)
delete _pinCS;
delete _pinDC;

_pinCS = NULL;
_pinDC = NULL;
#endif
}

void DmTftSsd2119::writeBus(uint8_t data) {
#if defined (DM_TOOLCHAIN_ARDUINO)
  SPCR = _spiSettings;         // SPI Control Register
  SPDR = data;                 // SPI Data Register
  while(!(SPSR & _BV(SPIF)));  // SPI Status Register Wait for transmission to finish
#elif defined (DM_TOOLCHAIN_MBED)
  spi.write(data);
#endif
}

void DmTftSsd2119::sendCommand(uint8_t index) {
  cbi(_pinDC, _bitmaskDC);
  writeBus(0x00);
  writeBus(index);
}

void DmTftSsd2119::send8BitData(uint8_t data) {
  sbi(_pinDC, _bitmaskDC);
  writeBus(data);
}

void DmTftSsd2119::sendData(uint16_t data) {
  uint8_t dh = data>>8;
  uint8_t dl = data&0xff;

  sbi(_pinDC, _bitmaskDC);
  writeBus(dh);
  writeBus(dl);
}

void DmTftSsd2119::setAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  // Set Start and End Vertical RAM address position
  uint16_t verticalStartEndAddress;
  verticalStartEndAddress = y0&0xff; // Start vertical RAM address
  verticalStartEndAddress += (y1&0xff)<<8; // End vertical RAM address
  
  sendCommand(0x44);
  sendData(verticalStartEndAddress);

  sendCommand(0x45); // Set Start Horizontal RAM address position
  sendData(x0);
  
  sendCommand(0x46);   // Set End Horizontal RAM address position
  sendData(x1);

  // Set start position
  sendCommand(0x4e); // Set Column, RAM address X (max 320)
  sendData(x0);
  sendCommand(0x4f);  // Set Page, RAM address Y (max 240)
  sendData(y0);
  sendCommand(0x22); // RAM data write
}

// Separate setPixel, because setAddress is to many instructions
void DmTftSsd2119::setPixel(uint16_t x, uint16_t y, uint16_t color) {
  cbi(_pinCS, _bitmaskCS);

  // setAddress(x, y, x, y);
  sendCommand(0x4e); // Set Column, RAM address X (max 320)
  sendData(x);

  sendCommand(0x4f);  // Set Page, RAM address Y (max 240)
  sendData(y);
  
  sendCommand(0x22);
  sendData(color);
  
  sbi(_pinCS, _bitmaskCS);
}

void DmTftSsd2119::init(void) {
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
  spi.frequency(6000000);
#endif
  cbi(_pinCS, _bitmaskCS);
  delay(135); // This much delay needed??

  delay(120);

  sendCommand(0x28);    // VCOM OTP
  sendData(0x0006);   

  sendCommand(0x00);    // Start Oscillator
  sendData(0x0001);   

  sendCommand(0x10);    // Set Sleep mode
  sendData(0x0000);     // Sleep out
  delay(30);

  sendCommand(0x11);    // Entry Mode ,SET LCM interface Mode.
  sendData(0x6870);     //  SET 65K colors,MCU interface Mode.TypeB ,ID=11 AM=0 the address counter is updated in the horizontal direction.

  sendCommand(0x15);    // Generic Interface Control. DOCLKï¼ŒHSYNCï¼ŒVSYNCï¼ŒDE
  sendData(0x0000);

  sendCommand(0x01);    // Driver Output Control
  sendData(0x72EF);     // Set REV,GD,BGR,SM,RL,TB

  sendCommand(0x02);    // LCD Driving Waveform Control,LCD 
  sendData(0x0600);  

  sendCommand(0x08);    // Set the scanning starting position of the gate driver.
  sendData(0x0000);     // The valid range is from 1 to 240
     
  // LCD POWER CONTROL
  sendCommand(0x03);    // Power Control 1, VGHï¼ŒVGL
  sendData(0x6A38);     // Set dct=fline*4, VGH=2 x VCIX2 + VCI,VGL=-(VGH) + VCix2, DC=Fline Ã— 8, AP=Medium to large   
   
  sendCommand(0X0B);    // Frame Cycle Control.
  sendData(0x5308);     // SET NO SDT=1 clock cycle (POR) ,Sets the equalizing period,    

  sendCommand(0x0C);    // Power Control 2 VCIX2
  sendData(0x0003);     // Adjust VCIX2 output voltage=5.7V

  sendCommand(0x0D);    // Set amplitude magnification of VLCD63     
  sendData(0x000A);     // vlcd63=VREF(2.0V)* 2.335V

  sendCommand(0x0E);    // SET VCOMG VDV .VCOM
  sendData(0x2B00);     // SET VCOMG=1,VCOM=VLCD63*  //2A 

  sendCommand(0X0F);    // Gate Scan Position.
  sendData(0x0000);     // The valid range is from 1 to 240.   

  sendCommand(0x1E);    // SET nOTP VCOMH ,VCOMH
  sendData(0x00B7);     // SET nOTP=0, VCOMH=VLCD63* //B8

  sendCommand(0x25);    // Frame Frequency Control
  sendData(0x8000);     // SET OSC  65Hz //0A-70hz

  sendCommand(0x26);    // Analog setting
  sendData(0x3800);     

  sendCommand(0x27);    // Analog setting
  sendData(0x0078);

//sendCommand(0x12);    // Sleep mode
//sendData(0xD999);   

  // SET WINDOW
  sendCommand(0x4E);    // Ram Address Set
  sendData(0x0000);      

  sendCommand(0x4F);    // Ram Address Set
  sendData(0x0000);    

  //  Gamma Control
  sendCommand(0x30);
  sendData(0x0000);//1

  sendCommand(0x31);
  sendData(0x0104);//2

  sendCommand(0x32);
  sendData(0x0100);//3

  sendCommand(0x33);
  sendData(0x0305);//4

  sendCommand(0x34);
  sendData(0x0505);//4

  sendCommand(0x35);
  sendData(0x0305);//5

  sendCommand(0x36);
  sendData(0x0707);//6

  sendCommand(0x37);
  sendData(0x0300);//7

  sendCommand(0x3A);
  sendData(0x1200);//8

  sendCommand(0x3B);
  sendData(0x0800);//9	
	 
  // Final init
  delay(300);
  sendCommand(0x07);	// Display Control 
  
  sendData(0x0033);		// Display on 
  delay(100);

  sendCommand(0x22);    // RAM data write/read
		
  delay(50);
  sbi(_pinCS, _bitmaskCS);
  clearScreen();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

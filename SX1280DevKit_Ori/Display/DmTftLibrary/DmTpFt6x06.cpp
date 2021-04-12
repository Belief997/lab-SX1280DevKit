/**********************************************************************************************
 Copyright (c) 2015 DisplayModule. All rights reserved.

 Redistribution and use of this source code, part of this source code or any compiled binary
 based on this source code is permitted as long as the above copyright notice and following
 disclaimer is retained.

 DISCLAIMER:
 THIS SOFTWARE IS SUPPLIED "AS IS" WITHOUT ANY WARRANTIES AND SUPPORT. DISPLAYMODULE ASSUMES
 NO RESPONSIBILITY OR LIABILITY FOR THE USE OF THE SOFTWARE.
 ********************************************************************************************/
// Tested with FT6X06

#include "DmTpFt6x06.h"
//#define log(...) Serial.println(__VA_ARGS__)
#define log(...)

#if defined(DM_TOOLCHAIN_ARDUINO)
// disp        - which display is used
// useIrq      - Enable IRQ or disable IRQ
DmTpFt6x06::DmTpFt6x06(Display disp, bool useIrq)
#elif defined (DM_TOOLCHAIN_MBED)
// disp        - which display is used
// &i2c        - I2C class
// useIrq      - Enable IRQ or disable IRQ
DmTpFt6x06::DmTpFt6x06(Display disp,  I2C &i2c, bool useIrq)
#endif
{
    switch (disp) {
    case DmTpFt6x06::DM_TFT28_116:
        _irq = D2;
        break;          
    default:
        _irq = D2;      
        break;
    }

    if (!useIrq) {
        _irq = NC;
    }
#if defined (DM_TOOLCHAIN_MBED)
   _i2c = &i2c;
#endif     
}

void DmTpFt6x06::init() {
    log("enter init()");
    
#if defined (DM_TOOLCHAIN_ARDUINO)
    Wire.begin();   
    Serial.print("Panel ID: "); Serial.println(readRegister8(FT6x06_FOCALTECH_ID));
    Serial.print("Chip ID: "); Serial.println(readRegister8(FT6x06_CHIPER));
    Serial.print("Firmware V: "); Serial.println(readRegister8(FT6x06_FIRMID));
    Serial.print("Point Rate Hz: "); Serial.println(readRegister8(FT6x06_PERIODACTIVE));
    Serial.print("Threshold: "); Serial.println(readRegister8(FT6x06_TH_GROUP));      
#elif defined (DM_TOOLCHAIN_MBED)
    _i2c->frequency(400000);
#endif
    if (_irq != -1) { // We will use Touch IRQ
        enableIrq();
    }

    
    log("exit init()");
}

void DmTpFt6x06::enableIrq() {
    log("enter enableIrq()");   
    
#if defined (DM_TOOLCHAIN_ARDUINO)
    pinMode(_irq, INPUT);
    _pinIrq = portInputRegister(digitalPinToPort(_irq));
    _bitmaskIrq  = digitalPinToBitMask(_irq);
#elif defined (DM_TOOLCHAIN_MBED)
    _pinIrq = new DigitalIn((PinName)_irq);
    _pinIrq->mode(PullUp);
#endif  
 
    log("exit enableIrq()");    
}

uint8_t DmTpFt6x06::readRegister8(uint8_t reg) {
#if defined (DM_TOOLCHAIN_ARDUINO)    
    uint8_t x ;
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.write((byte)reg);
    Wire.endTransmission();
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.requestFrom((byte)FT6x06_ADDR, (byte)1);
    x = Wire.read();
    Wire.endTransmission();
    return x;
#elif defined (DM_TOOLCHAIN_MBED)
    char buf[1] = {reg}, data = 0;   
    _i2c->write(FT6x06_ADDR, buf, 1, true);
    _i2c->read(FT6x06_ADDR, &data, 1);     
    return data;
#endif      
}

void DmTpFt6x06::writeRegister8(uint8_t reg, uint8_t val) {
#if defined (DM_TOOLCHAIN_ARDUINO)      
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.write((byte)reg);
    Wire.write((byte)val);
    Wire.endTransmission();
#elif defined (DM_TOOLCHAIN_MBED)    
    char buf[2] = {reg, val};
    _i2c->write(FT6x06_ADDR, buf, 2);
#endif  
}


bool DmTpFt6x06::isTouched() {
    if (_irq == -1) {
        uint8_t val;
        val = readRegister8(FT6x06_TD_STATUS);
        if ((val == 1) || (val == 2)) return true;
        return false;
    }
    
#if defined (DM_TOOLCHAIN_ARDUINO)    
    if ( !gbi(_pinIrq, _bitmaskIrq) ) {
        return true;
    }
    return false;
#elif defined (DM_TOOLCHAIN_MBED)
    if (!_pinIrq->read()) {
        return true;
    }
    return false;
#endif
}


void DmTpFt6x06::readTouchData(uint16_t& posX, uint16_t& posY, bool& touching) {
#if defined (DM_TOOLCHAIN_ARDUINO)    
    uint8_t buf[16];
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.write((byte)0);  
    Wire.endTransmission();
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.requestFrom((byte)FT6x06_ADDR, (byte)16);
    for (uint8_t i=0; i<16; i++)
        buf[i] = Wire.read();
    Wire.endTransmission();  

    posX = buf[3] & 0x0F;
    posX <<= 8;
    posX |= buf[4]; 
    posY = buf[5] & 0x0F;
    posY <<= 8;
    posY |= buf[6]; 
#elif defined (DM_TOOLCHAIN_MBED)
    posX = readRegister8(0x03) & 0x0F;
    posX <<= 8;
    posX |= readRegister8(0x04); 
    posY = readRegister8(0x05) & 0x0F;
    posY <<= 8;
    posY |= readRegister8(0x06); 
    
    posX = map(posX, 0, 240, 240, 0);
    posY = map(posY, 0, 320, 320, 0);
#endif
    touching = isTouched();
}

bool DmTpFt6x06::readTouchData(uint16_t& posX, uint16_t& posY) {
#if defined (DM_TOOLCHAIN_ARDUINO)   
    uint8_t buf[16];
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.write((byte)0);    
    Wire.endTransmission();
    Wire.beginTransmission(FT6x06_ADDR);
    Wire.requestFrom((byte)FT6x06_ADDR, (byte)16);
    for (uint8_t i=0; i<16; i++)
        buf[i] = Wire.read();
    Wire.endTransmission();  

    if(buf[2] > 2){
        posX = 0;
        posY = 0;
        return false;
        }  
    posX = buf[3] & 0x0F;
    posX <<= 8;
    posX |= buf[4]; 
    posY = buf[5] & 0x0F;
    posY <<= 8;
    posY |= buf[6]; 
    return true;
#elif defined (DM_TOOLCHAIN_MBED)
    if(readRegister8(0x03) > 2){
        posX = 0;
        posY = 0;
        return false;
        } 

    posX = readRegister8(0x03) & 0x0F;
    posX <<= 8;
    posX |= readRegister8(0x04); 
    posY = readRegister8(0x05) & 0x0F;
    posY <<= 8;
    posY |= readRegister8(0x06); 
    
    posX = map(posX, 0, 240, 240, 0);
    posY = map(posY, 0, 320, 320, 0);
    return true;
#endif    
}

long DmTpFt6x06::map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

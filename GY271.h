#ifndef WIRE_H
#include <Wire.h>
#endif

#define COMPASS_ADDRESS 0x0D

#define XOUT 0x00

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001
//Output data rate
#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100
//Measure range
#define RNG_2G          0b00000000
#define RNG_8G          0b00010000
//Over sampling rate
#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

class GY271 {
  public:
  void writeRegister(uint8_t reg, uint8_t val);
  void readData(uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *t);
  void setCtrlRegister(uint8_t overSampling, uint8_t range, uint8_t dataRate, uint8_t mode);
  void softReset();
};

void GY271::writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void GY271::readData(uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *t) {
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(XOUT);
  Wire.endTransmission();
  Wire.requestFrom(COMPASS_ADDRESS, 8);
  *x = (Wire.read() | Wire.read() << 8);
  *y = (Wire.read() | Wire.read() << 8);
  *z = (Wire.read() | Wire.read() << 8);
  *t = (Wire.read() | Wire.read() << 8);
}

void GY271::setCtrlRegister(uint8_t overSampling, uint8_t range, uint8_t dataRate, uint8_t mode) {
  writeRegister(0x09, overSampling | range | dataRate | mode);
}

void GY271::softReset() {
  writeRegister(0x0A, 0b10000000); //Soft reset, restore default value of all registers
  writeRegister(0x0B, 0x01); //SET/RESET Period (0x01 recommendet)
}

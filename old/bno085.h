#ifndef BNO085_H
#define BNO085_H

#include "Particle.h"

// BNO085 I2C address
#define BNO085_ADDRESS 0x4A

// BNO085 Registers
#define REG_DEVID          0x00
#define REG_POWER_CTL      0x2D
#define REG_DATA_FORMAT    0x31
#define REG_DATAX0         0x32

class BNO085 {
public:
BNO085(TwoWire &wirePort = Wire, uint8_t address = BNO085_ADDRESS);

    bool begin();
    void readAccelerometer(int16_t *x, int16_t *y, int16_t *z);
    void readAccelerometer_G(float *x, float *y, float *z);
    void readGyroscope(int16_t *x, int16_t *y, int16_t *z);
    void readGyroscope_rad_s(float *x, float *y, float *z);
    void readMagnetometer(int16_t *x, int16_t *y, int16_t *z);
    void readMagnetometer_µT(float *x, float *y, float *z);

private:
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    TwoWire &wire;
    uint8_t i2cAddress;
};

#endif // BNO085_H

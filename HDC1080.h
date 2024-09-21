#ifndef HDC1080_H
#define HDC1080_H

#include <stdint.h>
#include <stdbool.h>

#define HDC1080_I2C_ADDR 0x40

#define HDC1080_MANUFACTURER_ID 0x5449
#define HDC1080_DEVICE_ID 0x1050

#define HDC1080_RESET_NORMAL 0
#define HDC1080_RESET_SOFTWARE 1

#define HDC1080_HEATER_DISABLED 0
#define HDC1080_HEATER_ENABLED  1

#define HDC1080_ACQ_TEMP_OR_HUM 0
#define HDC1080_ACQ_TEMP_AND_HUM 1

#define HDC1080_BATTERY_GT_2_8V 0
#define HDC1080_BATTERY_LT_2_8V 1

#define HDC1080_TEMP_RESOLUTION_14BIT 0
#define HDC1080_TEMP_RESOLUTION_11BIT 1

#define HDC1080_HUM_RESOLUTION_14BIT 0
#define HDC1080_HUM_RESOLUTION_11BIT 1
#define HDC1080_HUM_RESOLUTION_8BIT 2

#define HDC1080_CONV_TIME_8BIT_US 2500 // 2.5ms
#define HDC1080_CONV_TIME_11BIT_US 3850 // 3.85ms
#define HDC1080_CONV_TIME_14BIT_US 7500 // 7.5ms

struct HDC1080_Configuration {
    unsigned int softwareReset : 1;
    unsigned int heater : 1;
    unsigned int modeOfAcquisition : 1;
    unsigned int batteryStatus : 1;
    unsigned int tempResolution : 1;
    unsigned int humResolution : 2;
};

struct HDC1080_Platform {
    int (*i2cWriteReg)(uint8_t addr7bit, uint8_t regNum, const uint8_t *data, uint8_t length, uint8_t wait);
    int (*i2cReadReg)(uint8_t addr7bit, uint8_t regNum, uint8_t *data, uint8_t length, int timeout);
    int (*i2cRead)(uint8_t addr7bit, uint8_t *data, uint8_t length, int timeout);

    void (*delayUs)(int us);
    void (*debugPrint)(const char *fmt, ...);
};

void HDC1080_Init(const struct HDC1080_Platform *platform);

bool HDC1080_DefInit(unsigned int tempResolution, unsigned int humResolution);
bool HDC1080_SoftwareReset(void);

bool HDC1080_PerformMeasurement(float *temperature, float *humidity);

bool HDC1080_ReadConfiguration(struct HDC1080_Configuration *config);
bool HDC1080_WriteConfiguration(const struct HDC1080_Configuration *config);

bool HDC1080_ReadManufacturerId(uint16_t *manufacturerId);
bool HDC1080_CheckManufacturerId(void);

bool HDC1080_ReadDeviceId(uint16_t *deviceId);
bool HDC1080_CheckDeviceId(void);

bool HDC1080_ReadTemperature(float *temperature);
bool HDC1080_ReadRelHumidity(float *humidity);

bool HDC1080_ReadSerialID(uint8_t *serialID_6Byte);


#endif // HDC1080_H

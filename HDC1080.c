#include "HDC1080.h"

#define HDC1080_REG_TEMPERATURE 0x00
#define HDC1080_REG_HUMIDITY 0x01
#define HDC1080_REG_CONFIGURATION 0x02
#define HDC1080_REG_SERIAL_ID_FIRST 0xFB
#define HDC1080_REG_SERIAL_ID_MID 0xFC
#define HDC1080_REG_SERIAL_ID_LAST 0xFD
#define HDC1080_REG_MANUFACTURER_ID 0xFE
#define HDC1080_REG_DEVICE_ID 0xFF

#define HDC1080_I2C_TIMEOUT 10

static int conversionTime_us[4] = {
    HDC1080_CONV_TIME_14BIT_US,
    HDC1080_CONV_TIME_11BIT_US,
    HDC1080_CONV_TIME_8BIT_US,
    0
};

static struct HDC1080_Configuration sensorConfig;

static struct HDC1080_Platform platform;

static int getConversionTime_us(unsigned int tempResolution, unsigned int humResolution) {
    return conversionTime_us[tempResolution & 3] +
        conversionTime_us[humResolution & 3] +
        5000;
}

static float calcTemperature(const uint8_t data[2]) {
    int rawTemperature = (data[0] << 8) | data[1];
    return (float)rawTemperature / 65536.0f * 165.0f - 40.0f;
}

static float calcHumidity(const uint8_t data[2]) {
    int rawHumidity = (data[0] << 8) | data[1];
    return (float)rawHumidity / 65536.0f * 100.0f;
}

void HDC1080_Init(const struct HDC1080_Platform *platformPtr) {
    platform = *platformPtr;
}

bool HDC1080_DefInit(unsigned int tempResolution, unsigned int humResolution) {
    struct HDC1080_Configuration config = {
        .softwareReset = HDC1080_RESET_NORMAL,
        .heater = HDC1080_HEATER_DISABLED,
        .modeOfAcquisition = HDC1080_ACQ_TEMP_AND_HUM,
        .tempResolution = tempResolution,
        .humResolution = humResolution
    };

    if (!HDC1080_WriteConfiguration(&config)) {
        return false;
    }

    return true;
}

bool HDC1080_SoftwareReset(void) {
    // Read configuration
    struct HDC1080_Configuration config;
    if (!HDC1080_ReadConfiguration(&config)) {
        return false;
    }

    // Set reset bit (the bit self-clears)
    config.softwareReset = HDC1080_RESET_SOFTWARE;

    // Write configuration
    if (!HDC1080_WriteConfiguration(&config)) {
        return false;
    }

    // Wait for reset to be completed
    // NOTE: the time is not specified in the datasheet
    platform.delayUs(10000);

    return true;
}

bool HDC1080_PerformMeasurement(float *temperature, float *humidity) {
    // Trigger the measurement
    int ret = platform.i2cWriteReg(HDC1080_I2C_ADDR, 0x00, 0, 0, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error triggering HDC1080 measurement: %d\r\n", -ret);
        return false;
    }

    // Wait for measurement to complete
    int conversionTime = getConversionTime_us(sensorConfig.tempResolution, sensorConfig.humResolution);
    platform.delayUs(conversionTime);

    // Read temperature and humidity
    uint8_t data[4];
    ret = platform.i2cRead(HDC1080_I2C_ADDR, data, 4, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error reading HDC1080 measurement: %d\r\n", -ret);
        return false;
    }

    // Calculate temperature and humidity
    *temperature = calcTemperature(data);
    *humidity = calcHumidity(data + 2);

    return true;
}

bool HDC1080_ReadConfiguration(struct HDC1080_Configuration *config) {
    uint8_t data[2];
    int ret = platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_CONFIGURATION, data, 2, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error reading HDC1080 configuration: %d\r\n", -ret);
        return false;
    }

    config->softwareReset = (data[0] >> 7) & 1;
    config->heater = (data[0] >> 5) & 1;
    config->modeOfAcquisition = (data[0] >> 4) & 1;
    config->batteryStatus = (data[0] >> 3) & 1;
    config->tempResolution = (data[0] >> 2) & 1;
    config->humResolution = (data[0] >> 0) & 3;

    sensorConfig = *config;

    return true;
}

bool HDC1080_WriteConfiguration(const struct HDC1080_Configuration *config) {
    uint8_t data[2] = {
        (config->softwareReset << 7) |
        (config->heater << 5) |
        (config->modeOfAcquisition << 4) |
        (config->batteryStatus << 3) |
        (config->tempResolution << 2) |
        (config->humResolution << 0),

        0
    };

    int ret = platform.i2cWriteReg(HDC1080_I2C_ADDR, HDC1080_REG_CONFIGURATION, data, 2, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error writing HDC1080 configuration: %d\r\n", -ret);
        return false;
    }

    sensorConfig = *config;

    return true;
}

bool HDC1080_ReadManufacturerId(uint16_t *manufacturerId) {
    uint8_t data[2];
    int ret = platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_MANUFACTURER_ID, data, 2, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error reading HDC1080 manufacturer ID: %d\r\n", -ret);
        return false;
    }

    *manufacturerId = (data[0] << 8) | data[1];
    return true;
}

bool HDC1080_CheckManufacturerId(void) {
    uint16_t manufacturerId;
    if (!HDC1080_ReadManufacturerId(&manufacturerId)) {
        return false;
    }

    if (manufacturerId != HDC1080_MANUFACTURER_ID) {
        platform.debugPrint("HDC1080 manufacturer ID mismatch: 0x%04X != 0x%04X\r\n", manufacturerId, HDC1080_MANUFACTURER_ID);
        return false;
    }

    return true;
}

bool HDC1080_ReadDeviceId(uint16_t *deviceId) {
    uint8_t data[2];
    int ret = platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_DEVICE_ID, data, 2, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error reading HDC1080 device ID: %d\r\n", -ret);
        return false;
    }

    *deviceId = (data[0] << 8) | data[1];
    return true;
}

bool HDC1080_CheckDeviceId(void) {
    uint16_t deviceId;
    if (!HDC1080_ReadDeviceId(&deviceId)) {
        return false;
    }

    if (deviceId != HDC1080_DEVICE_ID) {
        platform.debugPrint("HDC1080 device ID mismatch: 0x%04X != 0x%04X\r\n", deviceId, HDC1080_DEVICE_ID);
        return false;
    }

    return true;
}

bool HDC1080_ReadTemperature(float *temperature) {
    uint8_t data[2];
    int ret = platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_TEMPERATURE, data, 2, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error reading HDC1080 temperature: %d\r\n", -ret);
        return false;
    }

    *temperature = calcTemperature(data);
    return true;
}

bool HDC1080_ReadRelHumidity(float *humidity) {
    uint8_t data[2];
    int ret = platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_HUMIDITY, data, 2, HDC1080_I2C_TIMEOUT);
    if (ret < 0) {
        platform.debugPrint("Error reading HDC1080 humidity: %d\r\n", -ret);
        return false;
    }

    *humidity = calcHumidity(data);
    return true;
}

bool HDC1080_ReadSerialID(uint8_t *serialID_6Byte) {
    bool ok = true;
    
    ok &= platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_SERIAL_ID_FIRST,
        serialID_6Byte, 2, HDC1080_I2C_TIMEOUT) == 0;
    ok &= platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_SERIAL_ID_MID,
        serialID_6Byte + 2, 2, HDC1080_I2C_TIMEOUT) == 0;
    ok &= platform.i2cReadReg(HDC1080_I2C_ADDR, HDC1080_REG_SERIAL_ID_LAST,
        serialID_6Byte + 4, 2, HDC1080_I2C_TIMEOUT) == 0;
    
    if (!ok) {
        platform.debugPrint("Error reading HDC1080 serial ID\r\n");
        return false;
    }

    return true;
}

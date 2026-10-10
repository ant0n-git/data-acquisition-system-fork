#include "SystemConfig.h"

SystemConfig::SystemConfig()
    : i2cDevicePath("/dev/i2c-1"),
      spiDevicePath("/dev/spidev0.0"),
      spiSpeedHz(1000000),
      sensorReadIntervalMs(100),
      deviceName("MHP-VCU"),
      debugEnabled(true)
{
}

std::string SystemConfig::getI2CDevicePath() const
{
    return i2cDevicePath;
}

std::string SystemConfig::getSPIDevicePath() const
{
    return spiDevicePath;
}

unsigned int SystemConfig::getSPISpeedHz() const
{
    return spiSpeedHz;
}

int SystemConfig::getSensorReadIntervalMs() const
{
    return sensorReadIntervalMs;
}

std::string SystemConfig::getDeviceName() const
{
    return deviceName;
}

bool SystemConfig::isDebugEnabled() const
{
    return debugEnabled;
}
#include "VCUController.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

VCUController::VCUController()
    : config(),
      i2cBus(nullptr),
      spiBus(nullptr),
      gyroSensor(nullptr),
      strainSensor(nullptr),
      sensorManager(),
      dataSerialiser(),
      communicationManager(),
      running(false)
{
}

void VCUController::setup()
{
    std::cout << "Starting VCU system..." << std::endl;

    bool communicationReady =
        communicationManager.init();

    if (!communicationReady)
    {
        std::cerr
            << "Warning: communication failed to initialise."
            << std::endl;
    }

    try
    {
        // Temporary direct MPU6050 connection
        // on Raspberry Pi I2C bus 1.
        i2cBus =
            std::make_unique<I2CBus>(
                "/dev/i2c-1"
            );

        gyroSensor =
            std::make_shared<GyroscopeSensor>(
                "gyro_1",
                *i2cBus
            );

        sensorManager.addSensor(
            gyroSensor
        );
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Failed to set up gyroscope: "
            << e.what()
            << std::endl;
    }

    try
    {
        // ADS131M08 strain gauge ADC. The ADS131M08 uses SPI mode 1.
        spiBus =
            std::make_unique<SPIBus>(
                config.getSPIDevicePath(),
                config.getSPISpeedHz(),
                1
            );

        // TODO: Set channels, gauge factor, excitation voltage and
        // PGA gain to match the strain gauge PCB and bridges.
        StrainChannelConfig bridge0;
        bridge0.name = "strain_0";
        bridge0.channel = 0;
        bridge0.gaugeFactor = 2.0f;
        bridge0.bridgeFactor = 1.0f;
        bridge0.excitationVoltage = 3.3f;
        bridge0.pgaGain = 32;

        strainSensor =
            std::make_shared<StrainGaugeSensor>(
                "strain_1",
                *spiBus,
                std::vector<StrainChannelConfig>{bridge0}
            );

        sensorManager.addSensor(
            strainSensor
        );
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Failed to set up strain gauges: "
            << e.what()
            << std::endl;
    }

    bool initSuccess =
        sensorManager.initAll();

    if (!initSuccess)
    {
        std::cerr
            << "Warning: one or more sensors failed to initialise."
            << std::endl;
    }

    if (initSuccess && gyroSensor)
    {
        std::cout
            << "Keep gyroscope still for calibration..."
            << std::endl;

        gyroSensor->calibrateGyroscope();

        std::cout
            << "Gyroscope calibration complete."
            << std::endl;
    }

    if (strainSensor && strainSensor->isInitialised())
    {
        std::cout
            << "Keep strain gauges unloaded for taring..."
            << std::endl;

        strainSensor->tare();
    }

    running = true;

    std::cout
        << "VCU setup complete."
        << std::endl;
}

void VCUController::run()
{
    setup();

    while (running)
    {
        // Read all sensors.
        sensorManager.readAll();

        // Collect all latest readings.
        std::vector<SensorReading> readings =
            sensorManager.getReadings();

        // Convert readings to JSON.
        std::vector<std::string> messages =
            DataSerialiser::toJSON(readings);

        // Send JSON messages.
        communicationManager.sendMessages(
            messages
        );

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                config.getSensorReadIntervalMs()
            )
        );
    }
}
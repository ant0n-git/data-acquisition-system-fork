#pragma once

#include "SPISensor.h"
#include "ADS131M08.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// One full Wheatstone bridge wired to one ADS131M08 differential input.
struct StrainChannelConfig
{
    // Used as the prefix of the telemetry keys, e.g. "frame_left"
    // produces "frame_left_microstrain" and "frame_left_mv".
    std::string name;

    // ADC input, 0-7 (AIN0P/AIN0N ... AIN7P/AIN7N)
    int channel = 0;

    // From the gauge datasheet, typically ~2.0 for foil gauges
    float gaugeFactor = 2.0f;

    // Converts bridge output to strain: strain = Vout / Vex / (GF * bridgeFactor)
    //   1.0                full bridge, 4 active gauges (bending)
    //   (1 + poisson) / 2  full bridge, 2 axial + 2 Poisson gauges
    // A negative value flips the sign if the bridge is wired the other way.
    float bridgeFactor = 1.0f;

    // Bridge excitation voltage
    float excitationVoltage = 3.3f;

    // PGA gain: 1, 2, 4, 8, 16, 32, 64 or 128.
    // Input range is +/- 1.2 V / gain (e.g. +/- 37.5 mV at gain 32).
    uint8_t pgaGain = 32;
};

// Strain gauge sensor that reads full Wheatstone bridges through an
// ADS131M08 ADC over SPI.
class StrainGaugeSensor : public SPISensor {
public:
    StrainGaugeSensor(
        std::string id,
        SPIBus& sharedBus,
        std::vector<StrainChannelConfig> channels,
        ADS131M08::OSR osr = ADS131M08::OSR::OSR_4096,
        int samplesPerReading = 8,
        bool globalChop = false,
        uint32_t clockHz = DEFAULT_CLOCK_HZ
    );

    // CLKIN from the on-board SiT1602BC-21-25E-4.096000G oscillator
    static constexpr uint32_t DEFAULT_CLOCK_HZ = 4096000;

    ~StrainGaugeSensor() override = default;

    bool init() override;
    bool read() override;

    // Records the current bridge outputs as zero strain.
    // The structure must be unloaded while this runs.
    bool tare(int samples = 200);

    bool isInitialised() const;

private:
    ADS131M08 adc;
    std::vector<StrainChannelConfig> channels;
    ADS131M08::OSR osr;
    int samplesPerReading;
    bool globalChop;
    uint32_t clockHz;

    bool initialised;

    // Bridge output in volts at zero strain, per ADC channel
    std::array<double, ADS131M08::NUM_CHANNELS> offsetVolts{};

    // Bit mask of the ADC channels in use, matching the STATUS DRDY bits
    uint16_t channelMask;

    // Averages `samples` fresh conversions for every configured channel.
    bool acquire(
        int samples,
        std::array<double, ADS131M08::NUM_CHANNELS>& averageVolts,
        bool& saturated
    );

    bool validateConfig() const;
};

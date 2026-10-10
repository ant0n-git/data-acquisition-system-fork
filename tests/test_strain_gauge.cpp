// Offline test for the ADS131M08 driver and StrainGaugeSensor.
//
// Runs anywhere (no Raspberry Pi or SPI hardware needed) by replacing
// SPIBus with a simulated ADS131M08 that follows the datasheet framing.
//
// Build from the repository root:
//   g++ -std=c++17 tests/test_strain_gauge.cpp src/hardware/SPIBus.cpp src/hardware/GPIOInput.cpp src/hardware/ADS131M08.cpp src/sensors/SPISensor.cpp src/sensors/analog/StrainGaugeSensor.cpp -Isrc -Isrc/hardware -Isrc/sensors -Isrc/sensors/analog -o test_strain_gauge

#include "ADS131M08.h"
#include "StrainGaugeSensor.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    int failures = 0;

    void check(bool condition, const std::string& message)
    {
        if (!condition)
        {
            std::cerr << "FAIL: " << message << std::endl;
            failures++;
        }
        else
        {
            std::cout << "ok:   " << message << std::endl;
        }
    }

    bool near(double a, double b, double tolerance)
    {
        return std::fabs(a - b) <= tolerance;
    }
}

// Simulated ADS131M08 behind an SPIBus.
class FakeADS131M08 : public SPIBus
{
public:
    std::array<uint16_t, 64> registers{};

    // Differential input voltage on each channel
    std::array<double, ADS131M08::NUM_CHANNELS> inputVolts{};

    // Voltage on the REFIN pin, used when EXTREF_EN is set
    double refinVolts = 1.25;

    bool corruptNextCrc = false;
    bool present = true;
    int framesBetweenConversions = 0;
    int transfers = 0;

    FakeADS131M08()
    {
        powerOnReset();
    }

    bool openBus() override { return true; }
    bool isOpen() const override { return true; }

    bool transfer(
        const std::vector<uint8_t>& tx,
        std::vector<uint8_t>& rx) override
    {
        rx.assign(tx.size(), present ? 0x00 : 0xFF);
        transfers++;

        if (!present)
        {
            return true;
        }

        if (tx.size() != ADS131M08::FRAME_BYTES)
        {
            return false;
        }

        // A new conversion is ready every (framesBetweenConversions + 1) frames
        bool fresh = framesUntilConversion == 0;

        // Output frame: response to the previous command, then data.
        // The response to NULL is the current STATUS register.
        uint16_t response = pendingResponse;

        if (lastWasNull)
        {
            response = static_cast<uint16_t>(0x0500 | (fresh ? 0x00FF : 0x0000));
        }

        putWord(rx, 0, response);

        for (int ch = 0; ch < ADS131M08::NUM_CHANNELS; ++ch)
        {
            putCode(rx, ch + 1, codeFor(ch));
        }

        std::size_t crcOffset = (ADS131M08::FRAME_WORDS - 1) * 3;
        uint16_t crc = ADS131M08::crc16(rx.data(), crcOffset);

        if (corruptNextCrc)
        {
            crc ^= 0x1;
            corruptNextCrc = false;
        }

        putWord(rx, ADS131M08::FRAME_WORDS - 1, crc);

        framesUntilConversion = fresh
            ? framesBetweenConversions
            : framesUntilConversion - 1;

        // Decode this frame's command; its response goes in the next frame
        uint16_t command = static_cast<uint16_t>((tx[0] << 8) | tx[1]);
        lastWasNull = command == ADS131M08::CMD_NULL;

        if (command == ADS131M08::CMD_RESET)
        {
            powerOnReset();
            pendingResponse = ADS131M08::RESET_RESPONSE;
        }
        else if ((command & 0xE000) == 0xA000)
        {
            uint8_t address = (command >> 7) & 0x3F;
            pendingResponse = registers[address];
        }
        else if ((command & 0xE000) == 0x6000)
        {
            uint8_t address = (command >> 7) & 0x3F;
            uint16_t value = static_cast<uint16_t>((tx[3] << 8) | tx[4]);

            if (address != ADS131M08::REG_ID && address != ADS131M08::REG_STATUS)
            {
                registers[address] = value;
            }

            pendingResponse = static_cast<uint16_t>(0x4000 | (address << 7));
        }

        return true;
    }

    // A conversion completes now (DRDY falls)
    void convertNow()
    {
        framesUntilConversion = 0;
    }

    uint8_t gainOf(int channel) const
    {
        uint16_t reg = registers[channel < 4 ? 4 : 5];
        return static_cast<uint8_t>(1u << ((reg >> ((channel % 4) * 4)) & 0x7));
    }

private:
    uint16_t pendingResponse = 0;
    bool lastWasNull = false;
    int framesUntilConversion = 0;

    void powerOnReset()
    {
        registers.fill(0);
        registers[ADS131M08::REG_ID] = 0x2800;
        registers[ADS131M08::REG_STATUS] = 0x0500;
        registers[ADS131M08::REG_MODE] = 0x0510;
        registers[ADS131M08::REG_CLOCK] = 0xFF0E;
        registers[ADS131M08::REG_CFG] = 0x0600;
    }

    int32_t codeFor(int channel) const
    {
        bool external = registers[ADS131M08::REG_CLOCK] & ADS131M08::CLOCK_EXTREF_EN;
        double vref = external ? 0.96 * refinVolts : 1.2;
        double lsb = vref / (gainOf(channel) * 8388608.0);
        double code = std::round(inputVolts[channel] / lsb);

        if (code > 8388607.0) code = 8388607.0;
        if (code < -8388608.0) code = -8388608.0;

        return static_cast<int32_t>(code);
    }

    static void putWord(std::vector<uint8_t>& rx, std::size_t word, uint16_t value)
    {
        rx[word * 3] = static_cast<uint8_t>(value >> 8);
        rx[word * 3 + 1] = static_cast<uint8_t>(value & 0xFF);
        rx[word * 3 + 2] = 0;
    }

    static void putCode(std::vector<uint8_t>& rx, std::size_t word, int32_t code)
    {
        uint32_t raw = static_cast<uint32_t>(code) & 0xFFFFFF;

        rx[word * 3] = static_cast<uint8_t>(raw >> 16);
        rx[word * 3 + 1] = static_cast<uint8_t>(raw >> 8);
        rx[word * 3 + 2] = static_cast<uint8_t>(raw);
    }
};

// Simulated DRDY pin wired to a FakeADS131M08
class FakeDrdy : public GPIOInput
{
public:
    explicit FakeDrdy(FakeADS131M08& adc) : adc(adc) {}

    bool available = true;
    bool stuck = false;
    int edges = 0;

    bool openLine() override { return available; }
    bool isOpen() const override { return false; }
    void discardEdges() override {}

    int waitForFallingEdge(int) override
    {
        if (stuck)
        {
            return 0;
        }

        edges++;
        adc.convertNow();
        return 1;
    }

private:
    FakeADS131M08& adc;
};

static void testCrc()
{
    const char* text = "123456789";

    // CRC-16/CCITT-FALSE check value
    check(
        ADS131M08::crc16(reinterpret_cast<const uint8_t*>(text), 9) == 0x29B1,
        "CRC-16-CCITT matches the standard check value"
    );
}

static void testConversion()
{
    check(near(ADS131M08::codeToVolts(0x7FFFFF, 1), 1.2, 1e-6), "full scale at gain 1 is 1.2 V");
    check(near(ADS131M08::codeToVolts(-0x800000, 32), -0.0375, 1e-7), "negative full scale at gain 32 is -37.5 mV");
    check(near(ADS131M08::effectiveReference(true, 1.25f), 1.2, 1e-6), "REFIN 1.25 V gives a 1.2 V reference");
    check(near(ADS131M08::effectiveReference(false, 1.25f), 1.2, 1e-6), "internal reference is 1.2 V");
    check(near(ADS131M08::dataRateHz(4096000, ADS131M08::OSR::OSR_4096), 500.0, 1e-9), "4.096 MHz CLKIN at OSR 4096 gives 500 SPS");
    check(ADS131M08::isValidGain(128) && !ADS131M08::isValidGain(3) && !ADS131M08::isValidGain(0), "gain validation");
}

static void testDriverRegisters()
{
    FakeADS131M08 fake;
    ADS131M08 adc(fake);

    check(adc.reset(), "reset returns 0xFF28");

    uint16_t id = 0;
    check(adc.readRegister(ADS131M08::REG_ID, id) && id == 0x2800, "ID register reads 0x2800");

    check(adc.setChannelGain(5, 64), "set channel 5 gain");
    check(fake.gainOf(5) == 64 && fake.gainOf(4) == 1, "gain written to the right field of GAIN2");

    check(adc.setOversampling(ADS131M08::OSR::OSR_4096), "set OSR");
    check(fake.registers[ADS131M08::REG_CLOCK] == 0xFF16, "CLOCK register = 0xFF16 for OSR 4096");

    check(adc.setExternalReference(true) && fake.registers[ADS131M08::REG_CLOCK] == 0xFF56, "EXTREF_EN set in CLOCK");
    check(adc.setOversampling(ADS131M08::OSR::OSR_1024) && fake.registers[ADS131M08::REG_CLOCK] == 0xFF4E, "changing OSR keeps EXTREF_EN");
    check(adc.setGlobalChop(true) && fake.registers[ADS131M08::REG_CFG] == 0x0700, "global chop sets GC_EN");
}

static void testStrainSensor()
{
    FakeADS131M08 fake;
    fake.framesBetweenConversions = 2;

    StrainChannelConfig left;
    left.name = "left";
    left.channel = 0;
    left.gaugeFactor = 2.0f;
    left.excitationVoltage = 3.3f;
    left.pgaGain = 32;

    StrainChannelConfig right = left;
    right.name = "right";
    right.channel = 3;
    right.pgaGain = 128;

    StrainGaugeSensor sensor("strain_test", fake, {left, right});

    check(!sensor.read(), "read fails before init");
    check(sensor.init(), "sensor init");
    check(fake.registers[ADS131M08::REG_CLOCK] & ADS131M08::CLOCK_EXTREF_EN, "external reference enabled by default");
    check(fake.gainOf(0) == 32 && fake.gainOf(3) == 128, "per-channel gains applied");

    // Bridge imbalance at no load
    fake.inputVolts[0] = 0.0012;
    fake.inputVolts[3] = -0.0004;

    check(sensor.tare(20), "tare");

    // Apply 500 microstrain on the left bridge and -250 on the right:
    // Vout = strain * GF * Vex
    fake.inputVolts[0] = 0.0012 + 500e-6 * 2.0 * 3.3;
    fake.inputVolts[3] = -0.0004 + -250e-6 * 2.0 * 3.3;

    check(sensor.read(), "read after load");

    const SensorReading& reading = sensor.getLastReading();

    check(reading.isValid, "reading is valid");
    check(near(reading.values.at("left_microstrain"), 500.0, 0.5), "left = 500 microstrain");
    check(near(reading.values.at("right_microstrain"), -250.0, 0.5), "right = -250 microstrain");
    check(near(reading.values.at("left_mv"), 4.5, 0.01), "left bridge output = 4.5 mV");

    // One corrupted frame should be skipped, not fail the read
    fake.corruptNextCrc = true;
    check(sensor.read() && reading.isValid, "single CRC error is tolerated");

    // Drive the gain 128 input past its +/- 9.375 mV range
    fake.inputVolts[3] = 0.02;
    check(sensor.read() && !reading.isValid, "saturated input marks reading invalid");
}

// Bridge excited from the same 2.5 V rail that feeds the REFIN divider
static void testRatiometric()
{
    FakeADS131M08 fake;

    StrainChannelConfig config;
    config.name = "gauge";
    config.excitationVoltage = 2.5f;

    StrainGaugeSensor sensor("strain_ratio", fake, {config});

    check(sensor.init() && sensor.tare(20), "ratiometric init and tare");

    // The 2.5 V rail is actually 2% low: excitation and REFIN both drop
    double rail = 2.45;
    fake.refinVolts = rail / 2.0;
    fake.inputVolts[0] = 800e-6 * 2.0 * rail;

    check(sensor.read(), "ratiometric read");
    check(
        near(sensor.getLastReading().values.at("gauge_microstrain"), 800.0, 1.0),
        "2.5 V rail error cancels out (800 microstrain)"
    );
}

static void testDrdy()
{
    FakeADS131M08 fake;

    // Conversions only happen when the DRDY pin fires
    fake.framesBetweenConversions = 1000000;

    FakeDrdy drdy(fake);

    StrainChannelConfig config;
    config.name = "gauge";

    StrainGaugeSensor sensor("strain_drdy", fake, {config}, StrainGaugeADCSettings(), &drdy);

    check(sensor.init() && sensor.isUsingDrdy(), "init with DRDY pin");

    fake.inputVolts[0] = 300e-6 * 2.0 * 2.5;

    int transfersBefore = fake.transfers;
    int edgesBefore = drdy.edges;

    check(sensor.read(), "read using DRDY");
    check(fake.transfers - transfersBefore == 8, "one SPI frame per sample with DRDY (8 samples)");
    check(drdy.edges - edgesBefore == 8, "one DRDY edge per sample");
    check(
        near(sensor.getLastReading().values.at("gauge_microstrain"), 300.0, 0.5),
        "DRDY reading = 300 microstrain"
    );

    drdy.stuck = true;
    check(!sensor.read() && !sensor.getLastReading().isValid, "read fails if DRDY never falls");
}

static void testDrdyUnavailable()
{
    FakeADS131M08 fake;
    fake.framesBetweenConversions = 2;

    FakeDrdy drdy(fake);
    drdy.available = false;

    StrainChannelConfig config;
    config.name = "gauge";

    StrainGaugeSensor sensor("strain_nodrdy", fake, {config}, StrainGaugeADCSettings(), &drdy);

    check(sensor.init() && !sensor.isUsingDrdy(), "falls back to polling if DRDY cannot be opened");
    check(sensor.read(), "read by polling after fallback");
}

static void testMissingDevice()
{
    FakeADS131M08 fake;
    fake.present = false;

    StrainChannelConfig config;
    config.name = "gauge";

    StrainGaugeSensor sensor("strain_missing", fake, {config});

    check(!sensor.init(), "init fails when no device answers");
}

static void testNoConversions()
{
    FakeADS131M08 fake;
    fake.framesBetweenConversions = 1000000;

    StrainChannelConfig config;
    config.name = "gauge";

    StrainGaugeSensor sensor("strain_noclock", fake, {config});

    check(!sensor.init(), "init times out when no conversions arrive (no CLKIN)");
}

static void testBadConfig()
{
    FakeADS131M08 fake;

    StrainChannelConfig a;
    a.name = "a";
    a.channel = 2;

    StrainChannelConfig b = a;
    b.name = "b";

    StrainGaugeSensor duplicate("strain_dup", fake, {a, b});
    check(!duplicate.init(), "duplicate channel rejected");

    a.pgaGain = 3;
    StrainGaugeSensor badGain("strain_gain", fake, {a});
    check(!badGain.init(), "invalid gain rejected");
}

int main()
{
    testCrc();
    testConversion();
    testDriverRegisters();
    testStrainSensor();
    testRatiometric();
    testDrdy();
    testDrdyUnavailable();
    testMissingDevice();
    testNoConversions();
    testBadConfig();

    if (failures > 0)
    {
        std::cerr << failures << " test(s) failed" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "All strain gauge tests passed" << std::endl;
    return EXIT_SUCCESS;
}

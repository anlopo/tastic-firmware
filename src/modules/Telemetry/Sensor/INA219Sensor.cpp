#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_INA219.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "INA219Sensor.h"
#include "TelemetrySensor.h"
#include <Adafruit_INA219.h>
#include <algorithm>
#include <cmath>
#include <cstddef>

// External declaration for INA219 addresses found during scan
#if !MESHTASTIC_EXCLUDE_I2C
#ifndef ARCH_PORTDUINO
#include <Wire.h>
#endif
extern std::vector<std::pair<uint8_t, TwoWire *>> ina219Addresses;
#endif

#ifndef INA219_MULTIPLIER
#define INA219_MULTIPLIER 1.0f
#endif

INA219Sensor::INA219Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_INA219, "INA219") {}

INA219Sensor::~INA219Sensor()
{
    // Clean up allocated INA219 instances
    for (auto *ina : ina219Instances) {
        delete ina;
    }
    ina219Instances.clear();
    ina219Addrs.clear();
}

bool INA219Sensor::initOne(uint8_t addr, TwoWire *wire)
{
    // Skip addresses already initialized so retries don't leak instances.
    if (std::find(ina219Addrs.begin(), ina219Addrs.end(), addr) != ina219Addrs.end()) {
        return true;
    }

    LOG_DEBUG("Attempting to initialize INA219 at address 0x%x", addr);
    Adafruit_INA219 *ina = new Adafruit_INA219(addr);
    if (ina->begin(wire)) {
        ina219Instances.push_back(ina);
        ina219Addrs.push_back(addr);
        LOG_INFO("Initialized INA219 at address 0x%x", addr);
        return true;
    }
    LOG_WARN("Failed to initialize INA219 at address 0x%x", addr);
    delete ina;
    return false;
}

int32_t INA219Sensor::runOnce()
{
    LOG_INFO("Init sensor: %s", sensorName);

#if !MESHTASTIC_EXCLUDE_I2C
    // Try to initialize every INA219 the scanner found. initOne() skips
    // addresses that are already up, so missing/failed chips are retried on
    // every runOnce() cycle until they come back.
    for (const auto &addrPair : ina219Addresses) {
        initOne(addrPair.first, addrPair.second);
    }
#endif

    // Legacy fallback: if nothing was provided by the scanner but the
    // single-sensor map has an entry (older boards / pre-scan paths), init it.
    if (ina219Instances.empty()) {
        auto &entry = nodeTelemetrySensorsMap[sensorType];
        if (entry.first > 0) {
            initOne(entry.first, entry.second);
        }
    }

    status = !ina219Instances.empty();
    return initI2CSensor();
}

void INA219Sensor::setup() {}

struct _INA219Measurement INA219Sensor::getMeasurement(size_t channel)
{
    struct _INA219Measurement measurement;
    measurement.voltage = 0.0f;
    measurement.current = 0.0f;
    measurement.valid = false;

    if (channel < ina219Instances.size() && ina219Instances[channel] != nullptr) {
        LOG_DEBUG("Reading from INA219 instance %zu (address 0x%x)", channel, ina219Addrs[channel]);
        float voltage = ina219Instances[channel]->getBusVoltage_V();
        float current = ina219Instances[channel]->getCurrent_mA() * INA219_MULTIPLIER;

        bool vOk = !isnan(voltage) && !isinf(voltage);
        bool iOk = !isnan(current) && !isinf(current);
        LOG_DEBUG("Raw INA219 read: voltage=%.3f (ok=%d), current=%.3f (ok=%d)", voltage, vOk, current, iOk);

        if (vOk)
            measurement.voltage = voltage;
        else
            LOG_WARN("Invalid voltage reading from INA219 at address 0x%x: %.3f", ina219Addrs[channel], voltage);

        if (iOk)
            measurement.current = current;
        else
            LOG_WARN("Invalid current reading from INA219 at address 0x%x: %.3f", ina219Addrs[channel], current);

        measurement.valid = vOk && iOk;
    } else {
        LOG_WARN("Invalid channel index %zu or null pointer (total instances: %zu)", channel, ina219Instances.size());
    }

    return measurement;
}

bool INA219Sensor::getEnvironmentMetrics(meshtastic_Telemetry *measurement)
{
    if (ina219Instances.empty()) {
        return false;
    }

    // When multiple INA219 instances are present, only power telemetry
    // consumes them — environment metrics come from the other env sensors
    // (temperature/humidity/etc.). Returning false here is enough: the env
    // aggregator in EnvironmentTelemetry.cpp already short-circuits other
    // sensors correctly (`valid = valid || get_metrics`).
    if (ina219Instances.size() > 1) {
        return false;
    }

    // Use first channel for environment metrics (single INA219 case).
    struct _INA219Measurement m = getMeasurement(0);
    if (!m.valid) {
        return false;
    }

    measurement->variant.environment_metrics.has_voltage = true;
    measurement->variant.environment_metrics.has_current = true;
    measurement->variant.environment_metrics.voltage = m.voltage;
    measurement->variant.environment_metrics.current = m.current;

    return true;
}

bool INA219Sensor::getPowerMetrics(meshtastic_Telemetry *measurement)
{
    if (ina219Instances.empty()) {
        return false;
    }

    // Map INA219 instances to channels based on address:
    //   ch1 = 0x40, ch2 = 0x41, ch3 = 0x43
    size_t ch1_idx = SIZE_MAX, ch2_idx = SIZE_MAX, ch3_idx = SIZE_MAX;
    for (size_t i = 0; i < ina219Addrs.size(); i++) {
        if (ina219Addrs[i] == 0x40)
            ch1_idx = i;
        else if (ina219Addrs[i] == 0x41)
            ch2_idx = i;
        else if (ina219Addrs[i] == 0x43)
            ch3_idx = i;
    }

    bool anySet = false;

    if (ch1_idx != SIZE_MAX) {
        struct _INA219Measurement m = getMeasurement(ch1_idx);
        if (m.valid) {
            measurement->variant.power_metrics.has_ch1_voltage = true;
            measurement->variant.power_metrics.has_ch1_current = true;
            measurement->variant.power_metrics.ch1_voltage = m.voltage;
            measurement->variant.power_metrics.ch1_current = m.current;
            anySet = true;
        }
    }

    if (ch2_idx != SIZE_MAX) {
        struct _INA219Measurement m = getMeasurement(ch2_idx);
        if (m.valid) {
            measurement->variant.power_metrics.has_ch2_voltage = true;
            measurement->variant.power_metrics.has_ch2_current = true;
            measurement->variant.power_metrics.ch2_voltage = m.voltage;
            measurement->variant.power_metrics.ch2_current = m.current;
            anySet = true;
        }
    }

    if (ch3_idx != SIZE_MAX) {
        struct _INA219Measurement m = getMeasurement(ch3_idx);
        if (m.valid) {
            measurement->variant.power_metrics.has_ch3_voltage = true;
            measurement->variant.power_metrics.has_ch3_current = true;
            measurement->variant.power_metrics.ch3_voltage = m.voltage;
            measurement->variant.power_metrics.ch3_current = m.current;
            anySet = true;
        }
    }

    return anySet;
}

bool INA219Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    switch (measurement->which_variant) {
    case meshtastic_Telemetry_environment_metrics_tag:
        return getEnvironmentMetrics(measurement);

    case meshtastic_Telemetry_power_metrics_tag:
        return getPowerMetrics(measurement);
    }

    return false;
}

uint16_t INA219Sensor::getBusVoltageMv()
{
    if (ina219Instances.empty() || ina219Instances[0] == nullptr) {
        return 0;
    }
    return lround(ina219Instances[0]->getBusVoltage_V() * 1000);
}

int16_t INA219Sensor::getCurrentMa()
{
    if (ina219Instances.empty() || ina219Instances[0] == nullptr) {
        return 0;
    }
    return lround(ina219Instances[0]->getCurrent_mA());
}

bool INA219Sensor::hasSensor()
{
    if (!ina219Instances.empty()) {
        return true;
    }
#if !MESHTASTIC_EXCLUDE_I2C
    if (!ina219Addresses.empty()) {
        return true;
    }
#endif
    return TelemetrySensor::hasSensor();
}

#endif
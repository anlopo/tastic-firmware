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
}

int32_t INA219Sensor::runOnce()
{
    LOG_INFO("Init sensor: %s", sensorName);
    
    // Check if we have any INA219 addresses from the scan
    if (ina219Addresses.empty() && !hasSensor()) {
        return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
    }
    
    // Initialize all INA219 instances found during scan
    if (ina219Instances.empty() && !ina219Addresses.empty()) {
        LOG_INFO("Initializing %d INA219 instances", ina219Addresses.size());
        for (const auto &addrPair : ina219Addresses) {
            uint8_t addr = addrPair.first;
            TwoWire *wire = addrPair.second;
            
            LOG_DEBUG("Attempting to initialize INA219 at address 0x%x", addr);
            // Allocate on heap to avoid copying issues
            Adafruit_INA219 *ina = new Adafruit_INA219(addr);
            if (ina->begin(wire)) {
                ina219Instances.push_back(ina);
                ina219Addrs.push_back(addr);
                LOG_INFO("Initialized INA219 at address 0x%x", addr);
            } else {
                LOG_WARN("Failed to initialize INA219 at address 0x%x", addr);
                delete ina;
            }
        }
        LOG_INFO("Total INA219 instances initialized: %d", ina219Instances.size());
        
        // If no instances were initialized but we have the old single sensor method, try that
        if (ina219Instances.empty() && hasSensor()) {
            Adafruit_INA219 *ina = new Adafruit_INA219(nodeTelemetrySensorsMap[sensorType].first);
            if (ina->begin(nodeTelemetrySensorsMap[sensorType].second)) {
                ina219Instances.push_back(ina);
                ina219Addrs.push_back(nodeTelemetrySensorsMap[sensorType].first);
                LOG_INFO("Initialized single INA219 at address 0x%x", nodeTelemetrySensorsMap[sensorType].first);
            } else {
                delete ina;
            }
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
    
    if (channel < ina219Instances.size() && ina219Instances[channel] != nullptr) {
        LOG_DEBUG("Reading from INA219 instance %d (address 0x%x)", channel, ina219Addrs[channel]);
        // Safely read from INA219, handle potential I2C errors
        float voltage = ina219Instances[channel]->getBusVoltage_V();
        float current = ina219Instances[channel]->getCurrent_mA() * INA219_MULTIPLIER;
        
        LOG_DEBUG("Raw INA219 read: voltage=%.3f, current=%.3f (NaN=%d, Inf=%d)", 
                  voltage, current, isnan(voltage), isinf(voltage));
        
        // Check for valid readings (INA219 returns NaN or invalid values on error)
        if (!isnan(voltage) && !isinf(voltage)) {
            measurement.voltage = voltage;
        } else {
            LOG_WARN("Invalid voltage reading from INA219 at address 0x%x: %.3f", ina219Addrs[channel], voltage);
        }
        if (!isnan(current) && !isinf(current)) {
            measurement.current = current;
        } else {
            LOG_WARN("Invalid current reading from INA219 at address 0x%x: %.3f", ina219Addrs[channel], current);
        }
    } else {
        LOG_WARN("Invalid channel index %d or null pointer (total instances: %d)", channel, ina219Instances.size());
    }
    
    return measurement;
}

bool INA219Sensor::getEnvironmentMetrics(meshtastic_Telemetry *measurement)
{
    if (ina219Instances.empty()) {
        return false;
    }
    
    // When multiple INA219 instances are present, use them only for power telemetry
    // (multi-channel), not for environment telemetry (single-channel)
    if (ina219Instances.size() > 1) {
        return false;
    }
    
    // Use first channel for environment metrics (single INA219 case)
    struct _INA219Measurement m = getMeasurement(0);
    
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
    // Channel 1 = address 0x40
    // Channel 2 = address 0x41
    // Channel 3 = address 0x43
    
    LOG_DEBUG("getPowerMetrics: Found %d INA219 instances with addresses:", ina219Addrs.size());
    for (size_t i = 0; i < ina219Addrs.size(); i++) {
        LOG_DEBUG("  Instance %d: address 0x%x", i, ina219Addrs[i]);
    }
    
    // Find indices for each channel address
    size_t ch1_idx = SIZE_MAX, ch2_idx = SIZE_MAX, ch3_idx = SIZE_MAX;
    for (size_t i = 0; i < ina219Addrs.size(); i++) {
        if (ina219Addrs[i] == 0x40) {
            ch1_idx = i;
        } else if (ina219Addrs[i] == 0x41) {
            ch2_idx = i;
        } else if (ina219Addrs[i] == 0x43) {
            ch3_idx = i;
        }
    }
    
    LOG_DEBUG("Channel mapping: ch1_idx=%d, ch2_idx=%d, ch3_idx=%d", 
              ch1_idx == SIZE_MAX ? -1 : (int)ch1_idx,
              ch2_idx == SIZE_MAX ? -1 : (int)ch2_idx,
              ch3_idx == SIZE_MAX ? -1 : (int)ch3_idx);
    
    // Read from channel 1 (0x40)
    if (ch1_idx != SIZE_MAX) {
        struct _INA219Measurement m = getMeasurement(ch1_idx);
        measurement->variant.power_metrics.has_ch1_voltage = true;
        measurement->variant.power_metrics.has_ch1_current = true;
        measurement->variant.power_metrics.ch1_voltage = m.voltage;
        measurement->variant.power_metrics.ch1_current = m.current;
    }
    
    // Read from channel 2 (0x41)
    if (ch2_idx != SIZE_MAX) {
        struct _INA219Measurement m = getMeasurement(ch2_idx);
        measurement->variant.power_metrics.has_ch2_voltage = true;
        measurement->variant.power_metrics.has_ch2_current = true;
        measurement->variant.power_metrics.ch2_voltage = m.voltage;
        measurement->variant.power_metrics.ch2_current = m.current;
    }
    
    // Read from channel 3 (0x43)
    if (ch3_idx != SIZE_MAX) {
        struct _INA219Measurement m = getMeasurement(ch3_idx);
        LOG_DEBUG("Channel 3 (0x43) measurement: voltage=%.3f, current=%.3f", m.voltage, m.current);
        measurement->variant.power_metrics.has_ch3_voltage = true;
        measurement->variant.power_metrics.has_ch3_current = true;
        measurement->variant.power_metrics.ch3_voltage = m.voltage;
        measurement->variant.power_metrics.ch3_current = m.current;
    } else {
        LOG_DEBUG("Channel 3 (0x43) not found - ch3_idx is SIZE_MAX");
    }
    
    return true;
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
    // Check if we have initialized instances or addresses from scan
    if (!ina219Instances.empty()) {
        return true;
    }
#if !MESHTASTIC_EXCLUDE_I2C
    if (!ina219Addresses.empty()) {
        return true;
    }
#endif
    // Fall back to base class check
    return TelemetrySensor::hasSensor();
}

#endif
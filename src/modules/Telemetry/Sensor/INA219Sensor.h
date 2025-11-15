#pragma once

#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_INA219.h>)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "CurrentSensor.h"
#include "TelemetrySensor.h"
#include "VoltageSensor.h"
#include <Adafruit_INA219.h>
#include <vector>

class INA219Sensor : public TelemetrySensor, VoltageSensor, CurrentSensor
{
  private:
    // Support for multiple INA219 instances (up to 3 for channels 1, 2, 3)
    // Use pointers to avoid copying issues with Adafruit_INA219 objects
    std::vector<Adafruit_INA219 *> ina219Instances;
    std::vector<uint8_t> ina219Addrs;

    // Get measurement from a specific channel (0-based index)
    struct _INA219Measurement getMeasurement(size_t channel);

  protected:
    virtual void setup() override;
    bool getEnvironmentMetrics(meshtastic_Telemetry *measurement);
    bool getPowerMetrics(meshtastic_Telemetry *measurement);

  public:
    INA219Sensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual uint16_t getBusVoltageMv() override;
    virtual int16_t getCurrentMa() override;
    bool hasSensor();
    
    // Destructor to clean up allocated INA219 instances
    ~INA219Sensor();
};

struct _INA219Measurement {
    float voltage;
    float current;
};

#endif
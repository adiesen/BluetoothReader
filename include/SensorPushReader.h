#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace ble {

struct SensorPushReading {
    double temperatureC = 0;
    double relativeHumidity = 0;
    std::optional<double> pressurePa;
    std::optional<uint16_t> batteryMillivolts;
};

// Connects to a second-generation SensorPush device and reads its GATT values.
bool readSensorPush(const std::string& address, bool addressIsRandom,
                    SensorPushReading& reading, std::string& error);

} // namespace ble
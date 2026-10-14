#include "SensorPushReader.h"

#include <dbus/dbus.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

namespace ble {
namespace {

constexpr char kBluezService[] = "org.bluez";
constexpr char kObjectManagerInterface[] = "org.freedesktop.DBus.ObjectManager";
constexpr char kPropertiesInterface[] = "org.freedesktop.DBus.Properties";
constexpr char kAdapterInterface[] = "org.bluez.Adapter1";
constexpr char kDeviceInterface[] = "org.bluez.Device1";
constexpr char kGattServiceInterface[] = "org.bluez.GattService1";
constexpr char kGattCharacteristicInterface[] = "org.bluez.GattCharacteristic1";
constexpr char kSensorPushServiceUuid[] = "ef090000-11d6-42ba-93b8-9dd7ec090ab0";
constexpr char kTemperatureUuid[] = "ef090080-11d6-42ba-93b8-9dd7ec090aa9";
constexpr char kHumidityUuid[] = "ef090081-11d6-42ba-93b8-9dd7ec090aa9";
constexpr char kPressureUuid[] = "ef090082-11d6-42ba-93b8-9dd7ec090aa9";
constexpr char kBatteryUuid[] = "ef090007-11d6-42ba-93b8-9dd7ec090aa9";

using MessagePtr = std::unique_ptr<DBusMessage, decltype(&dbus_message_unref)>;
using ConnectionPtr = std::unique_ptr<DBusConnection, decltype(&dbus_connection_unref)>;

struct DeviceInfo {
    std::string path;
    std::string address;
    std::string adapterPath;
    bool connected = false;
    bool servicesResolved = false;
};

struct ServiceInfo {
    std::string devicePath;
    std::string uuid;
};

struct CharacteristicInfo {
    std::string servicePath;
    std::string uuid;
};

struct BluezSnapshot {
    std::map<std::string, DeviceInfo> devices;
    std::map<std::string, ServiceInfo> services;
    std::map<std::string, CharacteristicInfo> characteristics;
};

std::string normalize(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

uint16_t readLe16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

int32_t readLe32Signed(const uint8_t* data) {
    uint32_t value = static_cast<uint32_t>(data[0]) |
                     (static_cast<uint32_t>(data[1]) << 8) |
                     (static_cast<uint32_t>(data[2]) << 16) |
                     (static_cast<uint32_t>(data[3]) << 24);
    int32_t result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

MessagePtr sendMessage(DBusConnection* connection, MessagePtr request,
                       std::string& error, std::string* errorName = nullptr) {
    DBusError dbusError;
    dbus_error_init(&dbusError);
    DBusMessage* reply = dbus_connection_send_with_reply_and_block(
        connection, request.get(), 15000, &dbusError);
    if (reply == nullptr) {
        error = dbus_error_is_set(&dbusError) && dbusError.message != nullptr
                    ? dbusError.message
                    : "BlueZ D-Bus method call failed";
        if (errorName != nullptr && dbusError.name != nullptr) {
            *errorName = dbusError.name;
        }
        dbus_error_free(&dbusError);
        return MessagePtr(nullptr, &dbus_message_unref);
    }
    dbus_error_free(&dbusError);
    return MessagePtr(reply, &dbus_message_unref);
}

MessagePtr callNoArgs(DBusConnection* connection, const std::string& path,
                      const char* interface, const char* method,
                      std::string& error, std::string* errorName = nullptr) {
    MessagePtr request(dbus_message_new_method_call(
                           kBluezService, path.c_str(), interface, method),
                       &dbus_message_unref);
    if (!request) {
        error = "Failed to allocate BlueZ D-Bus request";
        return MessagePtr(nullptr, &dbus_message_unref);
    }
    return sendMessage(connection, std::move(request), error, errorName);
}

bool getStringProperty(DBusMessageIter properties, const char* wanted,
                       std::string& result) {
    while (dbus_message_iter_get_arg_type(&properties) != DBUS_TYPE_INVALID) {
        if (dbus_message_iter_get_arg_type(&properties) == DBUS_TYPE_DICT_ENTRY) {
            DBusMessageIter entry;
            dbus_message_iter_recurse(&properties, &entry);
            const char* name = nullptr;
            dbus_message_iter_get_basic(&entry, &name);
            if (name != nullptr && dbus_message_iter_next(&entry) &&
                std::strcmp(name, wanted) == 0 &&
                dbus_message_iter_get_arg_type(&entry) == DBUS_TYPE_VARIANT) {
                DBusMessageIter variant;
                dbus_message_iter_recurse(&entry, &variant);
                int type = dbus_message_iter_get_arg_type(&variant);
                if (type == DBUS_TYPE_STRING || type == DBUS_TYPE_OBJECT_PATH) {
                    const char* value = nullptr;
                    dbus_message_iter_get_basic(&variant, &value);
                    if (value != nullptr) {
                        result = value;
                        return true;
                    }
                }
            }
        }
        dbus_message_iter_next(&properties);
    }
    return false;
}

bool getBooleanProperty(DBusMessageIter properties, const char* wanted,
                        bool& result) {
    while (dbus_message_iter_get_arg_type(&properties) != DBUS_TYPE_INVALID) {
        if (dbus_message_iter_get_arg_type(&properties) == DBUS_TYPE_DICT_ENTRY) {
            DBusMessageIter entry;
            dbus_message_iter_recurse(&properties, &entry);
            const char* name = nullptr;
            dbus_message_iter_get_basic(&entry, &name);
            if (name != nullptr && dbus_message_iter_next(&entry) &&
                std::strcmp(name, wanted) == 0 &&
                dbus_message_iter_get_arg_type(&entry) == DBUS_TYPE_VARIANT) {
                DBusMessageIter variant;
                dbus_message_iter_recurse(&entry, &variant);
                if (dbus_message_iter_get_arg_type(&variant) == DBUS_TYPE_BOOLEAN) {
                    dbus_bool_t value = false;
                    dbus_message_iter_get_basic(&variant, &value);
                    result = value != false;
                    return true;
                }
            }
        }
        dbus_message_iter_next(&properties);
    }
    return false;
}

bool parseManagedObjects(DBusMessage* reply, BluezSnapshot& snapshot,
                         std::string& error) {
    DBusMessageIter root;
    if (!dbus_message_iter_init(reply, &root) ||
        dbus_message_iter_get_arg_type(&root) != DBUS_TYPE_ARRAY) {
        error = "Malformed BlueZ managed-objects response";
        return false;
    }

    DBusMessageIter objects;
    dbus_message_iter_recurse(&root, &objects);
    while (dbus_message_iter_get_arg_type(&objects) != DBUS_TYPE_INVALID) {
        DBusMessageIter objectEntry;
        dbus_message_iter_recurse(&objects, &objectEntry);
        const char* objectPathValue = nullptr;
        dbus_message_iter_get_basic(&objectEntry, &objectPathValue);
        std::string objectPath = objectPathValue == nullptr ? "" : objectPathValue;
        if (dbus_message_iter_next(&objectEntry) &&
            dbus_message_iter_get_arg_type(&objectEntry) == DBUS_TYPE_ARRAY) {
            DBusMessageIter interfaces;
            dbus_message_iter_recurse(&objectEntry, &interfaces);
            while (dbus_message_iter_get_arg_type(&interfaces) != DBUS_TYPE_INVALID) {
                DBusMessageIter interfaceEntry;
                dbus_message_iter_recurse(&interfaces, &interfaceEntry);
                const char* interfaceValue = nullptr;
                dbus_message_iter_get_basic(&interfaceEntry, &interfaceValue);
                std::string interfaceName = interfaceValue == nullptr ? "" : interfaceValue;
                if (dbus_message_iter_next(&interfaceEntry) &&
                    dbus_message_iter_get_arg_type(&interfaceEntry) == DBUS_TYPE_ARRAY) {
                    DBusMessageIter properties;
                    dbus_message_iter_recurse(&interfaceEntry, &properties);
                    if (interfaceName == kDeviceInterface) {
                        DeviceInfo device;
                        device.path = objectPath;
                        getStringProperty(properties, "Address", device.address);
                        getStringProperty(properties, "Adapter", device.adapterPath);
                        getBooleanProperty(properties, "Connected", device.connected);
                        getBooleanProperty(properties, "ServicesResolved", device.servicesResolved);
                        if (!device.address.empty()) {
                            snapshot.devices[objectPath] = std::move(device);
                        }
                    } else if (interfaceName == kGattServiceInterface) {
                        ServiceInfo service;
                        getStringProperty(properties, "Device", service.devicePath);
                        getStringProperty(properties, "UUID", service.uuid);
                        if (!service.uuid.empty()) {
                            snapshot.services[objectPath] = std::move(service);
                        }
                    } else if (interfaceName == kGattCharacteristicInterface) {
                        CharacteristicInfo characteristic;
                        getStringProperty(properties, "Service", characteristic.servicePath);
                        getStringProperty(properties, "UUID", characteristic.uuid);
                        if (!characteristic.uuid.empty()) {
                            snapshot.characteristics[objectPath] = std::move(characteristic);
                        }
                    }
                }
                dbus_message_iter_next(&interfaces);
            }
        }
        dbus_message_iter_next(&objects);
    }
    return true;
}

bool getManagedObjects(DBusConnection* connection, BluezSnapshot& snapshot,
                       std::string& error) {
    auto reply = callNoArgs(connection, "/", kObjectManagerInterface,
                            "GetManagedObjects", error);
    return reply && parseManagedObjects(reply.get(), snapshot, error);
}

bool findDevice(const BluezSnapshot& snapshot, const std::string& address,
                const std::string& adapterPath, DeviceInfo& result) {
    const std::string wantedAddress = normalize(address);
    for (const auto& item : snapshot.devices) {
        if (normalize(item.second.address) == wantedAddress &&
            item.second.adapterPath == adapterPath) {
            result = item.second;
            return true;
        }
    }
    return false;
}

bool findSensorCharacteristics(const BluezSnapshot& snapshot,
                               const std::string& devicePath,
                               std::string& temperaturePath,
                               std::string& humidityPath,
                               std::string& pressurePath,
                               std::string& batteryPath) {
    std::string servicePath;
    for (const auto& item : snapshot.services) {
        if (item.second.devicePath == devicePath &&
            normalize(item.second.uuid) == kSensorPushServiceUuid) {
            servicePath = item.first;
            break;
        }
    }
    if (servicePath.empty()) {
        return false;
    }

    for (const auto& item : snapshot.characteristics) {
        if (item.second.servicePath != servicePath) {
            continue;
        }
        const std::string uuid = normalize(item.second.uuid);
        if (uuid == kTemperatureUuid) {
            temperaturePath = item.first;
        } else if (uuid == kHumidityUuid) {
            humidityPath = item.first;
        } else if (uuid == kPressureUuid) {
            pressurePath = item.first;
        } else if (uuid == kBatteryUuid) {
            batteryPath = item.first;
        }
    }
    return !temperaturePath.empty() && !humidityPath.empty();
}

bool writeCharacteristic(DBusConnection* connection, const std::string& path,
                         const std::vector<uint8_t>& value, std::string& error) {
    MessagePtr request(dbus_message_new_method_call(
                           kBluezService, path.c_str(), kGattCharacteristicInterface,
                           "WriteValue"),
                       &dbus_message_unref);
    if (!request) {
        error = "Failed to allocate BlueZ GATT write request";
        return false;
    }

    DBusMessageIter arguments;
    DBusMessageIter bytes;
    DBusMessageIter options;
    dbus_message_iter_init_append(request.get(), &arguments);
    if (!dbus_message_iter_open_container(&arguments, DBUS_TYPE_ARRAY,
                                         DBUS_TYPE_BYTE_AS_STRING, &bytes)) {
        error = "Failed to build BlueZ GATT write request";
        return false;
    }
    for (uint8_t byte : value) {
        if (!dbus_message_iter_append_basic(&bytes, DBUS_TYPE_BYTE, &byte)) {
            error = "Failed to build BlueZ GATT write request";
            return false;
        }
    }
    if (!dbus_message_iter_close_container(&arguments, &bytes) ||
        !dbus_message_iter_open_container(&arguments, DBUS_TYPE_ARRAY,
                                          "{sv}", &options) ||
        !dbus_message_iter_close_container(&arguments, &options)) {
        error = "Failed to build BlueZ GATT write options";
        return false;
    }
    return static_cast<bool>(sendMessage(connection, std::move(request), error));
}

bool readCharacteristic(DBusConnection* connection, const std::string& path,
                        std::vector<uint8_t>& value, std::string& error) {
    MessagePtr request(dbus_message_new_method_call(
                           kBluezService, path.c_str(), kGattCharacteristicInterface,
                           "ReadValue"),
                       &dbus_message_unref);
    if (!request) {
        error = "Failed to allocate BlueZ GATT read request";
        return false;
    }

    DBusMessageIter arguments;
    DBusMessageIter options;
    dbus_message_iter_init_append(request.get(), &arguments);
    if (!dbus_message_iter_open_container(&arguments, DBUS_TYPE_ARRAY,
                                          "{sv}", &options) ||
        !dbus_message_iter_close_container(&arguments, &options)) {
        error = "Failed to build BlueZ GATT read options";
        return false;
    }

    auto reply = sendMessage(connection, std::move(request), error);
    if (!reply) {
        return false;
    }
    DBusMessageIter response;
    if (!dbus_message_iter_init(reply.get(), &response) ||
        dbus_message_iter_get_arg_type(&response) != DBUS_TYPE_ARRAY) {
        error = "Malformed BlueZ GATT read response";
        return false;
    }
    DBusMessageIter bytes;
    dbus_message_iter_recurse(&response, &bytes);
    value.clear();
    while (dbus_message_iter_get_arg_type(&bytes) != DBUS_TYPE_INVALID) {
        if (dbus_message_iter_get_arg_type(&bytes) != DBUS_TYPE_BYTE) {
            error = "Malformed BlueZ GATT characteristic value";
            return false;
        }
        uint8_t byte = 0;
        dbus_message_iter_get_basic(&bytes, &byte);
        value.push_back(byte);
        dbus_message_iter_next(&bytes);
    }
    return true;
}

bool readScaledValue(DBusConnection* connection, const std::string& path,
                     double& result, std::string& error) {
    std::vector<uint8_t> value;
    if (!readCharacteristic(connection, path, value, error)) {
        return false;
    }
    if (value.size() != 4) {
        error = "Unexpected SensorPush characteristic value size";
        return false;
    }
    result = readLe32Signed(value.data()) / 100.0;
    return true;
}

} // namespace

bool readSensorPush(const std::string& address, bool addressIsRandom,
                    SensorPushReading& reading, std::string& error,
                    int deviceId) {
    static_cast<void>(addressIsRandom);
    if (deviceId < 0) {
        deviceId = 0;
    }
    const std::string adapterPath = "/org/bluez/hci" + std::to_string(deviceId);

    DBusError busError;
    dbus_error_init(&busError);
    DBusConnection* rawConnection = dbus_bus_get(DBUS_BUS_SYSTEM, &busError);
    if (rawConnection == nullptr) {
        error = dbus_error_is_set(&busError) && busError.message != nullptr
                    ? busError.message
                    : "Failed to connect to the system D-Bus";
        dbus_error_free(&busError);
        return false;
    }
    dbus_error_free(&busError);
    ConnectionPtr connection(rawConnection, &dbus_connection_unref);
    dbus_connection_set_exit_on_disconnect(connection.get(), false);

    BluezSnapshot snapshot;
    if (!getManagedObjects(connection.get(), snapshot, error)) {
        return false;
    }

    DeviceInfo device;
    bool deviceFound = findDevice(snapshot, address, adapterPath, device);
    bool discoveryStarted = false;
    if (!deviceFound || !device.connected) {
        std::string discoveryError;
        std::string discoveryErrorName;
        auto discoveryReply = callNoArgs(connection.get(), adapterPath,
                                         kAdapterInterface, "StartDiscovery",
                                         discoveryError, &discoveryErrorName);
        if (!discoveryReply && discoveryErrorName != "org.bluez.Error.InProgress") {
            error = "BlueZ discovery failed: " + discoveryError;
            return false;
        }
        discoveryStarted = static_cast<bool>(discoveryReply);

        const auto discoveryReadyAt = std::chrono::steady_clock::now() +
                                      std::chrono::seconds(3);
        const auto discoveryDeadline = std::chrono::steady_clock::now() +
                                       std::chrono::seconds(15);
        while (std::chrono::steady_clock::now() < discoveryDeadline) {
            snapshot = {};
            if (!getManagedObjects(connection.get(), snapshot, error)) {
                break;
            }
            deviceFound = findDevice(snapshot, address, adapterPath, device);
            if (deviceFound && std::chrono::steady_clock::now() >= discoveryReadyAt) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        if (discoveryStarted) {
            std::string ignoredError;
            callNoArgs(connection.get(), adapterPath, kAdapterInterface,
                       "StopDiscovery", ignoredError);
        }
        if (!deviceFound) {
            if (error.empty()) {
                error = "BlueZ did not discover SensorPush device " + address;
            }
            return false;
        }
    }

    bool ownsConnection = false;
    if (!device.connected) {
        std::string connectErrorName;
        auto connectReply = callNoArgs(connection.get(), device.path,
                                       kDeviceInterface, "Connect", error,
                                       &connectErrorName);
        if (!connectReply && connectErrorName != "org.bluez.Error.AlreadyConnected") {
            error = "Failed to connect to SensorPush through BlueZ: " + error;
            return false;
        }
        ownsConnection = static_cast<bool>(connectReply);
    }

    std::string temperaturePath;
    std::string humidityPath;
    std::string pressurePath;
    std::string batteryPath;
    bool characteristicsFound = false;
    const auto serviceDeadline = std::chrono::steady_clock::now() +
                                 std::chrono::seconds(15);
    while (std::chrono::steady_clock::now() < serviceDeadline) {
        snapshot = {};
        if (!getManagedObjects(connection.get(), snapshot, error)) {
            break;
        }
        if (findDevice(snapshot, address, adapterPath, device) &&
            device.servicesResolved) {
            characteristicsFound = findSensorCharacteristics(
                snapshot, device.path, temperaturePath, humidityPath,
                pressurePath, batteryPath);
            if (characteristicsFound) {
                break;
            }
            error = "SensorPush GATT service or required characteristics were not found";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    bool success = characteristicsFound;
    if (success) {
        success = writeCharacteristic(connection.get(), temperaturePath,
                                      {0x01, 0x00, 0x00, 0x00}, error);
    }
    if (success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        success = readScaledValue(connection.get(), temperaturePath,
                                  reading.temperatureC, error);
    }
    if (success) {
        success = readScaledValue(connection.get(), humidityPath,
                                  reading.relativeHumidity, error);
    }
    if (success && !pressurePath.empty()) {
        success = writeCharacteristic(connection.get(), pressurePath,
                                      {0x01, 0x00, 0x00, 0x00}, error);
        if (success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            double pressurePa = 0;
            success = readScaledValue(connection.get(), pressurePath,
                                      pressurePa, error);
            if (success) {
                reading.pressurePa = pressurePa;
            }
        }
    }
    if (success && !batteryPath.empty()) {
        std::vector<uint8_t> value;
        success = readCharacteristic(connection.get(), batteryPath, value, error);
        if (success && value.size() == 4) {
            reading.batteryMillivolts = readLe16(value.data());
        } else if (success) {
            error = "Unexpected SensorPush battery value size";
            success = false;
        }
    }

    if (ownsConnection) {
        std::string ignoredError;
        callNoArgs(connection.get(), device.path, kDeviceInterface,
                   "Disconnect", ignoredError);
    }
    if (!success && error.empty()) {
        error = "Failed to read SensorPush characteristics";
    }
    return success;
}

} // namespace ble
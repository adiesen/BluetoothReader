#include "SensorPushReader.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <map>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace ble {
namespace {

constexpr uint8_t kAttErrorResponse = 0x01;
constexpr uint8_t kAttFindByTypeValueRequest = 0x06;
constexpr uint8_t kAttFindByTypeValueResponse = 0x07;
constexpr uint8_t kAttReadByTypeRequest = 0x08;
constexpr uint8_t kAttReadByTypeResponse = 0x09;
constexpr uint8_t kAttReadRequest = 0x0A;
constexpr uint8_t kAttReadResponse = 0x0B;
constexpr uint8_t kAttWriteRequest = 0x12;
constexpr uint8_t kAttWriteResponse = 0x13;
constexpr uint16_t kAttCid = 0x0004;
constexpr uint16_t kPrimaryServiceUuid = 0x2800;
constexpr uint16_t kCharacteristicUuid = 0x2803;

using Uuid = std::array<uint8_t, 16>;

Uuid uuidFromString(const char* text) {
    Uuid uuid{};
    size_t output = uuid.size();
    int highNibble = -1;
    for (const char* current = text; *current != '\0'; ++current) {
        if (*current == '-') {
            continue;
        }
        const char* digits = "0123456789abcdef";
        const char* digit = std::strchr(digits, static_cast<char>(
            *current >= 'A' && *current <= 'F' ? *current - 'A' + 'a' : *current));
        if (digit == nullptr) {
            continue;
        }
        int value = static_cast<int>(digit - digits);
        if (highNibble < 0) {
            highNibble = value;
        } else if (output > 0) {
            uuid[--output] = static_cast<uint8_t>((highNibble << 4) | value);
            highNibble = -1;
        }
    }
    return uuid;
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

bool exchangeAtt(int socketFd, const std::vector<uint8_t>& request,
                 uint8_t expectedOpcode, std::vector<uint8_t>& response,
                 uint8_t& attError, std::string& error) {
    ssize_t sent = send(socketFd, request.data(), request.size(), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(request.size())) {
        error = std::string("Failed to send GATT request: ") + std::strerror(errno);
        return false;
    }

    std::array<uint8_t, 512> buffer{};
    ssize_t received = recv(socketFd, buffer.data(), buffer.size(), 0);
    if (received < 0) {
        error = std::string("Timed out or failed waiting for GATT response: ") +
                std::strerror(errno);
        return false;
    }
    response.assign(buffer.begin(), buffer.begin() + received);
    if (response.empty()) {
        error = "Sensor disconnected during GATT request";
        return false;
    }
    if (response[0] == kAttErrorResponse) {
        attError = response.size() >= 5 ? response[4] : 0;
        error = "Sensor returned ATT error " + std::to_string(attError);
        return false;
    }
    if (response[0] != expectedOpcode) {
        error = "Unexpected GATT response opcode " + std::to_string(response[0]);
        return false;
    }
    return true;
}

bool discoverService(int socketFd, uint16_t& serviceStart, uint16_t& serviceEnd,
                     std::string& error) {
    const Uuid serviceUuid = uuidFromString("EF090000-11D6-42BA-93B8-9DD7EC090AB0");
    uint16_t handle = 1;
    while (handle != 0) {
        std::vector<uint8_t> request{kAttFindByTypeValueRequest,
                                     static_cast<uint8_t>(handle),
                                     static_cast<uint8_t>(handle >> 8),
                                     0xFF, 0xFF,
                                     static_cast<uint8_t>(kPrimaryServiceUuid),
                                     static_cast<uint8_t>(kPrimaryServiceUuid >> 8)};
        request.insert(request.end(), serviceUuid.begin(), serviceUuid.end());
        std::vector<uint8_t> response;
        uint8_t attError = 0;
        if (!exchangeAtt(socketFd, request, kAttFindByTypeValueResponse,
                         response, attError, error)) {
            if (attError == 0x0A) {
                error = "SensorPush GATT service was not found";
            }
            return false;
        }
        if (response.size() < 5 || (response.size() - 1) % 4 != 0) {
            error = "Malformed GATT service discovery response";
            return false;
        }
        serviceStart = readLe16(response.data() + 1);
        serviceEnd = readLe16(response.data() + 3);
        return true;
    }
    error = "SensorPush GATT service was not found";
    return false;
}

bool discoverCharacteristics(int socketFd, uint16_t serviceStart, uint16_t serviceEnd,
                              std::map<std::string, std::pair<uint16_t, uint8_t>>& found,
                              std::string& error) {
    uint16_t handle = serviceStart;
    while (handle <= serviceEnd) {
        std::vector<uint8_t> request{kAttReadByTypeRequest,
                                     static_cast<uint8_t>(handle),
                                     static_cast<uint8_t>(handle >> 8),
                                     static_cast<uint8_t>(serviceEnd),
                                     static_cast<uint8_t>(serviceEnd >> 8),
                                     static_cast<uint8_t>(kCharacteristicUuid),
                                     static_cast<uint8_t>(kCharacteristicUuid >> 8)};
        std::vector<uint8_t> response;
        uint8_t attError = 0;
        if (!exchangeAtt(socketFd, request, kAttReadByTypeResponse,
                         response, attError, error)) {
            if (attError == 0x0A) {
                return true;
            }
            return false;
        }
        if (response.size() < 2 || response[1] < 7 ||
            (response.size() - 2) % response[1] != 0) {
            error = "Malformed GATT characteristic discovery response";
            return false;
        }
        uint8_t entryLength = response[1];
        uint16_t lastDeclarationHandle = handle;
        for (size_t offset = 2; offset + entryLength <= response.size(); offset += entryLength) {
            lastDeclarationHandle = readLe16(response.data() + offset);
            uint8_t properties = response[offset + 2];
            uint16_t valueHandle = readLe16(response.data() + offset + 3);
            if (entryLength == 21) {
                Uuid uuid{};
                std::copy_n(response.data() + offset + 5, uuid.size(), uuid.begin());
                for (const char* item : {"EF090080-11D6-42BA-93B8-9DD7EC090AA9",
                                         "EF090081-11D6-42BA-93B8-9DD7EC090AA9",
                                         "EF090082-11D6-42BA-93B8-9DD7EC090AA9",
                                         "EF090007-11D6-42BA-93B8-9DD7EC090AA9"}) {
                    if (uuid == uuidFromString(item)) {
                        found[item] = {valueHandle, properties};
                    }
                }
            }
        }
        if (lastDeclarationHandle >= serviceEnd) {
            return true;
        }
        handle = static_cast<uint16_t>(lastDeclarationHandle + 1);
    }
    return true;
}

bool writeTrigger(int socketFd, uint16_t handle, std::string& error) {
    // A nonzero 32-bit value triggers a fresh sensor sample per the API.
    std::vector<uint8_t> request{kAttWriteRequest,
                                 static_cast<uint8_t>(handle),
                                 static_cast<uint8_t>(handle >> 8),
                                 0x01, 0x00, 0x00, 0x00};
    std::vector<uint8_t> response;
    uint8_t attError = 0;
    return exchangeAtt(socketFd, request, kAttWriteResponse,
                       response, attError, error);
}

bool readValue(int socketFd, uint16_t handle, std::vector<uint8_t>& value,
               std::string& error) {
    std::vector<uint8_t> response;
    uint8_t attError = 0;
    if (!exchangeAtt(socketFd,
                     {kAttReadRequest, static_cast<uint8_t>(handle),
                      static_cast<uint8_t>(handle >> 8)},
                     kAttReadResponse, response, attError, error)) {
        return false;
    }
    value.assign(response.begin() + 1, response.end());
    return true;
}

} // namespace

bool readSensorPush(const std::string& address, bool addressIsRandom,
                    SensorPushReading& reading, std::string& error) {
    bdaddr_t bluetoothAddress{};
    if (str2ba(address.c_str(), &bluetoothAddress) != 0) {
        error = "Invalid Bluetooth address: " + address;
        return false;
    }

    int socketFd = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (socketFd < 0) {
        error = std::string("Failed to create Bluetooth socket: ") + std::strerror(errno);
        return false;
    }
    timeval timeout{3, 0};
    setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socketFd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_l2 target{};
    target.l2_family = AF_BLUETOOTH;
    target.l2_psm = 0;
    target.l2_cid = htobs(kAttCid);
    target.l2_bdaddr_type = addressIsRandom ? BDADDR_LE_RANDOM : BDADDR_LE_PUBLIC;
    bacpy(&target.l2_bdaddr, &bluetoothAddress);
    if (connect(socketFd, reinterpret_cast<sockaddr*>(&target), sizeof(target)) < 0) {
        error = std::string("Failed to connect to SensorPush: ") + std::strerror(errno);
        ::close(socketFd);
        return false;
    }

    uint16_t serviceStart = 0;
    uint16_t serviceEnd = 0;
    std::map<std::string, std::pair<uint16_t, uint8_t>> characteristics;
    bool success = discoverService(socketFd, serviceStart, serviceEnd, error) &&
                   discoverCharacteristics(socketFd, serviceStart, serviceEnd,
                                           characteristics, error);
    const std::string temperatureUuid = "EF090080-11D6-42BA-93B8-9DD7EC090AA9";
    const std::string humidityUuid = "EF090081-11D6-42BA-93B8-9DD7EC090AA9";
    const std::string pressureUuid = "EF090082-11D6-42BA-93B8-9DD7EC090AA9";
    const std::string batteryUuid = "EF090007-11D6-42BA-93B8-9DD7EC090AA9";
    if (success && characteristics.find(temperatureUuid) == characteristics.end()) {
        error = "SensorPush temperature characteristic was not found (HT1 is not supported)";
        success = false;
    }
    if (success && characteristics.find(humidityUuid) == characteristics.end()) {
        error = "SensorPush relative humidity characteristic was not found";
        success = false;
    }

    std::vector<uint8_t> value;
    if (success) {
        success = writeTrigger(socketFd, characteristics[temperatureUuid].first, error);
    }
    if (success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        success = readValue(socketFd, characteristics[temperatureUuid].first, value, error);
        if (success && value.size() == 4) {
            reading.temperatureC = readLe32Signed(value.data()) / 100.0;
        } else if (success) {
            error = "Unexpected SensorPush temperature value size";
            success = false;
        }
    }
    if (success) {
        success = readValue(socketFd, characteristics[humidityUuid].first, value, error);
        if (success && value.size() == 4) {
            reading.relativeHumidity = readLe32Signed(value.data()) / 100.0;
        } else if (success) {
            error = "Unexpected SensorPush humidity value size";
            success = false;
        }
    }
    auto pressure = characteristics.find(pressureUuid);
    if (success && pressure != characteristics.end()) {
        success = writeTrigger(socketFd, pressure->second.first, error);
        if (success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            success = readValue(socketFd, pressure->second.first, value, error);
            if (success && value.size() == 4) {
                reading.pressurePa = readLe32Signed(value.data()) / 100.0;
            } else if (success) {
                error = "Unexpected SensorPush pressure value size";
                success = false;
            }
        }
    }
    auto battery = characteristics.find(batteryUuid);
    if (success && battery != characteristics.end()) {
        success = readValue(socketFd, battery->second.first, value, error);
        if (success && value.size() == 4) {
            reading.batteryMillivolts = readLe16(value.data());
        } else if (success) {
            error = "Unexpected SensorPush battery value size";
            success = false;
        }
    }

    ::close(socketFd);
    return success;
}

} // namespace ble
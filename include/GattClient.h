#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ble {

struct GattService {
    uint16_t startHandle = 0;
    uint16_t endHandle = 0;
    std::string uuid;
};

struct GattCharacteristic {
    uint16_t declarationHandle = 0;
    uint16_t valueHandle = 0;
    uint8_t properties = 0;
    std::string uuid;
};

class GattClient {
public:
    explicit GattClient(int deviceId = -1);
    ~GattClient();

    GattClient(const GattClient&) = delete;
    GattClient& operator=(const GattClient&) = delete;

    bool connect(const std::string& address, bool randomAddress = false);
    void disconnect();

    bool discoverServices(std::vector<GattService>& services);
    bool discoverCharacteristics(const GattService& service,
                                 std::vector<GattCharacteristic>& characteristics);
    bool readValue(uint16_t handle, std::vector<uint8_t>& value);

    const std::string& lastError() const { return lastError_; }

private:
    int deviceId_;
    int socketFd_ = -1;
    uint16_t mtu_ = 23;
    std::string lastError_;

    bool request(uint8_t opcode, const std::vector<uint8_t>& payload,
                 std::vector<uint8_t>& response, uint8_t& attError);
    bool exchangeMtu();
};

} // namespace ble
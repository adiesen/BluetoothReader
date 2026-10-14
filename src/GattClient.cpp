#include "GattClient.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/l2cap.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ble {

namespace {

constexpr uint16_t kAttCid = 4;
constexpr uint8_t kAttErrorResponse = 0x01;
constexpr uint8_t kAttExchangeMtuRequest = 0x02;
constexpr uint8_t kAttReadByTypeRequest = 0x08;
constexpr uint8_t kAttReadRequest = 0x0A;
constexpr uint8_t kAttReadBlobRequest = 0x0C;
constexpr uint8_t kAttReadByGroupTypeRequest = 0x10;
constexpr uint8_t kAttAttributeNotFound = 0x0A;

constexpr uint16_t kPrimaryServiceUuid = 0x2800;
constexpr uint16_t kCharacteristicUuid = 0x2803;

void append16(std::vector<uint8_t>& packet, uint16_t value) {
    packet.push_back(static_cast<uint8_t>(value));
    packet.push_back(static_cast<uint8_t>(value >> 8));
}

uint16_t read16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

std::string uuidString(const uint8_t* data, size_t length) {
    if (length == 2) {
        char result[7] = {};
        std::snprintf(result, sizeof(result), "%02X%02X", data[1], data[0]);
        return result;
    }
    if (length != 16) {
        return {};
    }

    char result[37] = {};
    std::snprintf(result, sizeof(result),
                  "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                  data[15], data[14], data[13], data[12], data[11], data[10],
                  data[9], data[8], data[7], data[6], data[5], data[4], data[3],
                  data[2], data[1], data[0]);
    return result;
}

} // namespace

GattClient::GattClient(int deviceId) : deviceId_(deviceId) {}

GattClient::~GattClient() {
    disconnect();
}

bool GattClient::connect(const std::string& address, bool randomAddress) {
    disconnect();

    bdaddr_t remoteAddress;
    if (str2ba(address.c_str(), &remoteAddress) < 0) {
        lastError_ = "Invalid Bluetooth address: " + address;
        return false;
    }

    socketFd_ = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (socketFd_ < 0) {
        lastError_ = std::string("Failed to create ATT socket: ") + std::strerror(errno);
        return false;
    }

    timeval timeout{5, 0};
    setsockopt(socketFd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socketFd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (deviceId_ >= 0) {
        bdaddr_t localAddress;
        if (hci_devba(deviceId_, &localAddress) < 0) {
            lastError_ = std::string("Failed to find HCI device: ") + std::strerror(errno);
            disconnect();
            return false;
        }
        sockaddr_l2 source{};
        source.l2_family = AF_BLUETOOTH;
        source.l2_bdaddr = localAddress;
        source.l2_bdaddr_type = BDADDR_LE_PUBLIC;
        if (::bind(socketFd_, reinterpret_cast<sockaddr*>(&source), sizeof(source)) < 0) {
            lastError_ = std::string("Failed to bind HCI device: ") + std::strerror(errno);
            disconnect();
            return false;
        }
    }

    sockaddr_l2 destination{};
    destination.l2_family = AF_BLUETOOTH;
    destination.l2_bdaddr = remoteAddress;
    destination.l2_cid = htobs(kAttCid);
    destination.l2_bdaddr_type = randomAddress ? BDADDR_LE_RANDOM : BDADDR_LE_PUBLIC;

    if (::connect(socketFd_, reinterpret_cast<sockaddr*>(&destination),
                  sizeof(destination)) < 0) {
        lastError_ = std::string("Failed to connect to BLE device: ") + std::strerror(errno);
        disconnect();
        return false;
    }

    if (!exchangeMtu()) {
        disconnect();
        return false;
    }
    return true;
}

void GattClient::disconnect() {
    if (socketFd_ >= 0) {
        ::close(socketFd_);
        socketFd_ = -1;
    }
    mtu_ = 23;
}

bool GattClient::request(uint8_t opcode, const std::vector<uint8_t>& payload,
                         std::vector<uint8_t>& response, uint8_t& attError) {
    attError = 0;
    response.clear();
    if (socketFd_ < 0) {
        lastError_ = "GATT client is not connected";
        return false;
    }

    std::vector<uint8_t> packet;
    packet.reserve(payload.size() + 1);
    packet.push_back(opcode);
    packet.insert(packet.end(), payload.begin(), payload.end());
    if (send(socketFd_, packet.data(), packet.size(), 0) !=
        static_cast<ssize_t>(packet.size())) {
        lastError_ = std::string("Failed to send ATT request: ") + std::strerror(errno);
        return false;
    }

    uint8_t buffer[517];
    ssize_t length = recv(socketFd_, buffer, sizeof(buffer), 0);
    if (length < 1) {
        lastError_ = std::string("Failed to receive ATT response: ") + std::strerror(errno);
        return false;
    }
    if (buffer[0] == kAttErrorResponse) {
        if (length < 5) {
            lastError_ = "Malformed ATT error response";
            return false;
        }
        attError = buffer[4];
        return true;
    }
    if (buffer[0] != static_cast<uint8_t>(opcode | 0x01)) {
        lastError_ = "Unexpected ATT response opcode";
        return false;
    }
    response.assign(buffer + 1, buffer + length);
    return true;
}

bool GattClient::exchangeMtu() {
    std::vector<uint8_t> response;
    uint8_t attError = 0;
    if (!request(kAttExchangeMtuRequest, {0x05, 0x02}, response, attError)) {
        return false;
    }
    if (attError != 0) {
        if (attError == 0x06) {
            return true;
        }
        lastError_ = "ATT MTU exchange failed with error 0x" + std::to_string(attError);
        return false;
    }
    if (response.size() != 2) {
        lastError_ = "Malformed ATT MTU response";
        return false;
    }
    mtu_ = std::max<uint16_t>(23, std::min<uint16_t>(517, read16(response.data())));
    return true;
}

bool GattClient::discoverServices(std::vector<GattService>& services) {
    services.clear();
    uint16_t start = 1;
    while (start != 0) {
        std::vector<uint8_t> payload;
        append16(payload, start);
        append16(payload, 0xFFFF);
        append16(payload, kPrimaryServiceUuid);

        std::vector<uint8_t> response;
        uint8_t attError = 0;
        if (!request(kAttReadByGroupTypeRequest, payload, response, attError)) {
            return false;
        }
        if (attError != 0) {
            if (attError == kAttAttributeNotFound) {
                break;
            }
            lastError_ = "ATT service discovery failed with error 0x" +
                         std::to_string(attError);
            return false;
        }
        if (response.empty() || (response[0] != 6 && response[0] != 20)) {
            lastError_ = "Malformed ATT service discovery response";
            return false;
        }

        size_t entryLength = response[0];
        if ((response.size() - 1) % entryLength != 0) {
            lastError_ = "Malformed ATT service discovery entry";
            return false;
        }
        for (size_t offset = 1; offset < response.size(); offset += entryLength) {
            GattService service;
            service.startHandle = read16(response.data() + offset);
            service.endHandle = read16(response.data() + offset + 2);
            service.uuid = uuidString(response.data() + offset + 4, entryLength - 4);
            services.push_back(std::move(service));
            start = services.back().endHandle + 1;
        }
    }
    return true;
}

bool GattClient::discoverCharacteristics(
    const GattService& service, std::vector<GattCharacteristic>& characteristics) {
    characteristics.clear();
    uint16_t start = service.startHandle;
    while (start <= service.endHandle) {
        std::vector<uint8_t> payload;
        append16(payload, start);
        append16(payload, service.endHandle);
        append16(payload, kCharacteristicUuid);

        std::vector<uint8_t> response;
        uint8_t attError = 0;
        if (!request(kAttReadByTypeRequest, payload, response, attError)) {
            return false;
        }
        if (attError != 0) {
            if (attError == kAttAttributeNotFound) {
                break;
            }
            lastError_ = "ATT characteristic discovery failed with error 0x" +
                         std::to_string(attError);
            return false;
        }
        if (response.empty() || (response[0] != 7 && response[0] != 21)) {
            lastError_ = "Malformed ATT characteristic discovery response";
            return false;
        }

        size_t entryLength = response[0];
        if ((response.size() - 1) % entryLength != 0) {
            lastError_ = "Malformed ATT characteristic discovery entry";
            return false;
        }
        for (size_t offset = 1; offset < response.size(); offset += entryLength) {
            GattCharacteristic characteristic;
            characteristic.declarationHandle = read16(response.data() + offset);
            characteristic.properties = response[offset + 2];
            characteristic.valueHandle = read16(response.data() + offset + 3);
            characteristic.uuid = uuidString(response.data() + offset + 5,
                                              entryLength - 5);
            characteristics.push_back(std::move(characteristic));
            if (characteristics.back().declarationHandle == 0xFFFF) {
                start = 0;
            } else {
                start = characteristics.back().declarationHandle + 1;
            }
        }
        if (start == 0) {
            break;
        }
    }
    return true;
}

bool GattClient::readValue(uint16_t handle, std::vector<uint8_t>& value) {
    value.clear();
    uint16_t offset = 0;
    while (true) {
        std::vector<uint8_t> payload;
        append16(payload, handle);
        uint8_t opcode = offset == 0 ? kAttReadRequest : kAttReadBlobRequest;
        if (offset != 0) {
            append16(payload, offset);
        }

        std::vector<uint8_t> response;
        uint8_t attError = 0;
        if (!request(opcode, payload, response, attError)) {
            return false;
        }
        if (attError != 0) {
            lastError_ = "ATT read failed with error 0x" + std::to_string(attError);
            return false;
        }
        if (response.empty()) {
            lastError_ = "Malformed ATT read response";
            return false;
        }
        value.insert(value.end(), response.begin(), response.end());
        if (response.size() < static_cast<size_t>(mtu_ - 1)) {
            break;
        }
        offset = static_cast<uint16_t>(value.size());
        if (offset == 0) {
            break;
        }
    }
    return true;
}

} // namespace ble
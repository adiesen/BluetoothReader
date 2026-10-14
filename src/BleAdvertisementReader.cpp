#include "BleAdvertisementReader.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

namespace ble {

namespace {

std::string bdaddrToString(const bdaddr_t& addr) {
    char buf[18] = {0};
    ba2str(&addr, buf);
    return std::string(buf);
}

// Parses the concatenated AD (Advertising Data) structures found in a
// single LE advertising report into the higher-level Advertisement struct.
void parseAdStructures(const uint8_t* data, uint8_t length, Advertisement& out) {
    out.rawData.assign(data, data + length);

    size_t offset = 0;
    while (offset + 1 <= static_cast<size_t>(length)) {
        uint8_t fieldLen = data[offset];
        if (fieldLen == 0 || offset + 1 + fieldLen > static_cast<size_t>(length)) {
            break;
        }
        uint8_t adType = data[offset + 1];
        const uint8_t* value = data + offset + 2;
        size_t valueLen = fieldLen - 1;

        switch (adType) {
            case 0x08: // Shortened Local Name
            case 0x09: // Complete Local Name
                out.name.assign(reinterpret_cast<const char*>(value), valueLen);
                break;
            case 0xFF: // Manufacturer Specific Data
                if (valueLen >= 2) {
                    out.manufacturerId = static_cast<uint16_t>(value[0]) |
                                          (static_cast<uint16_t>(value[1]) << 8);
                    out.manufacturerData.assign(value + 2, value + valueLen);
                }
                break;
            default:
                break;
        }

        offset += 1 + fieldLen;
    }
}

bool decodeGoveeTemperature(Advertisement& adv) {
    if (adv.manufacturerId != 0xEC88) {
        return false;
    }

    const auto& data = adv.manufacturerData;
    if (data.size() == 6) {
        // H5072/H5075: 24-bit signed temperature/humidity value.
        uint32_t encoded = (static_cast<uint32_t>(data[1]) << 16) |
                           (static_cast<uint32_t>(data[2]) << 8) | data[3];
        bool negative = (encoded & 0x800000) != 0;
        encoded &= 0x7FFFFF;
        double temperature = static_cast<double>(encoded / 1000) / 10.0;
        adv.temperatureCelsius = negative ? -temperature : temperature;
    } else if (data.size() >= 7) {
        // H5074: signed little-endian temperature in hundredths of a degree.
        // Some firmware revisions append optional bytes to this payload.
        int16_t encoded = static_cast<int16_t>(data[1] |
                                               (static_cast<uint16_t>(data[2]) << 8));
        adv.temperatureCelsius = static_cast<double>(encoded) / 100.0;
    } else {
        return false;
    }

    adv.hasTemperature = adv.temperatureCelsius >= -40.0 &&
                         adv.temperatureCelsius <= 100.0;
    return adv.hasTemperature;
}

bool decodeGoveeH5127(Advertisement& adv) {
    if (adv.manufacturerId != 0x8843 || adv.manufacturerData.size() != 6) {
        return false;
    }

    // H5127: byte 4 indicates presence and byte 5 indicates motion.
    const auto& data = adv.manufacturerData;
    adv.presenceDetected = data[4] == 1;
    adv.motionDetected = data[5] == 17;
    adv.hasH5127Data = true;
    return true;
}

} // namespace

BleAdvertisementReader::BleAdvertisementReader(int deviceId) : deviceId_(deviceId) {}

BleAdvertisementReader::~BleAdvertisementReader() {
    close();
}

bool BleAdvertisementReader::open() {
    int devId = deviceId_;
    if (devId < 0) {
        devId = hci_get_route(nullptr);
    }
    if (devId < 0) {
        lastError_ = "No available Bluetooth adapter found";
        return false;
    }

    socketFd_ = hci_open_dev(devId);
    if (socketFd_ < 0) {
        lastError_ = std::string("Failed to open HCI device: ") + std::strerror(errno);
        return false;
    }

    if (!setScanParameters()) {
        close();
        return false;
    }

    return true;
}

void BleAdvertisementReader::close() {
    if (socketFd_ >= 0) {
        setScanEnable(false);
        ::close(socketFd_);
        socketFd_ = -1;
    }
}

bool BleAdvertisementReader::setScanParameters() {
    // Passive scan with the maximum interval and window for continuous scanning.
    uint8_t scanType = 0x00;         // passive
    uint16_t interval = htobs(0x4000);
    uint16_t window = htobs(0x4000);
    uint8_t ownType = 0x00;          // public device address
    uint8_t filterPolicy = 0x00;     // accept all advertisements

    if (hci_le_set_scan_parameters(socketFd_, scanType, interval, window,
                                    ownType, filterPolicy, 1000) < 0) {
        lastError_ = std::string("Failed to set LE scan parameters: ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool BleAdvertisementReader::setScanEnable(bool enable) {
    uint8_t enableVal = enable ? 0x01 : 0x00;
    uint8_t filterDup = 0x00; // report duplicates so RSSI updates are visible

    if (hci_le_set_scan_enable(socketFd_, enableVal, filterDup, 1000) < 0) {
        lastError_ = std::string("Failed to set LE scan enable: ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool BleAdvertisementReader::startScan(const AdvertisementCallback& callback) {
    if (socketFd_ < 0) {
        lastError_ = "Device not open; call open() first";
        return false;
    }

    struct hci_filter oldFilter;
    socklen_t oldFilterLen = sizeof(oldFilter);
    if (getsockopt(socketFd_, SOL_HCI, HCI_FILTER, &oldFilter, &oldFilterLen) < 0) {
        lastError_ = std::string("Failed to read HCI filter: ") + std::strerror(errno);
        return false;
    }

    struct hci_filter newFilter;
    hci_filter_clear(&newFilter);
    hci_filter_set_ptype(HCI_EVENT_PKT, &newFilter);
    hci_filter_set_event(EVT_LE_META_EVENT, &newFilter);
    if (setsockopt(socketFd_, SOL_HCI, HCI_FILTER, &newFilter, sizeof(newFilter)) < 0) {
        lastError_ = std::string("Failed to set HCI filter: ") + std::strerror(errno);
        return false;
    }

    if (!setScanEnable(true)) {
        setsockopt(socketFd_, SOL_HCI, HCI_FILTER, &oldFilter, sizeof(oldFilter));
        return false;
    }

    stopRequested_ = false;
    uint8_t buf[HCI_MAX_EVENT_SIZE];

    while (!stopRequested_) {
        ssize_t len = read(socketFd_, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            lastError_ = std::string("Error reading from HCI socket: ") + std::strerror(errno);
            break;
        }

        // Layout: [packet type][hci_event_hdr][evt_le_meta_event][le_advertising_info...][rssi]
        if (len < 1 + HCI_EVENT_HDR_SIZE) {
            continue;
        }
        const uint8_t* ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        auto* meta = reinterpret_cast<const evt_le_meta_event*>(ptr);

        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) {
            continue;
        }

        uint8_t reportCount = meta->data[0];
        const uint8_t* reportPtr = meta->data + 1; // skip num_reports byte
        const uint8_t* packetEnd = buf + len;
        for (uint8_t report = 0; report < reportCount; ++report) {
            if (reportPtr + LE_ADVERTISING_INFO_SIZE + 1 > packetEnd) {
                break;
            }

            auto* info = reinterpret_cast<const le_advertising_info*>(reportPtr);
            size_t reportLength = LE_ADVERTISING_INFO_SIZE + info->length + 1;
            if (reportPtr + reportLength > packetEnd) {
                break;
            }

            Advertisement adv;
            adv.address = bdaddrToString(info->bdaddr);
            adv.addressIsRandom = (info->bdaddr_type == LE_RANDOM_ADDRESS);
            parseAdStructures(info->data, info->length, adv);

            decodeGoveeTemperature(adv);
            decodeGoveeH5127(adv);

            // RSSI is a signed byte appended after the variable-length AD data.
            adv.rssi = static_cast<int8_t>(reportPtr[LE_ADVERTISING_INFO_SIZE +
                                                     info->length]);
            callback(adv);
            reportPtr += reportLength;
        }
    }

    setScanEnable(false);
    setsockopt(socketFd_, SOL_HCI, HCI_FILTER, &oldFilter, sizeof(oldFilter));
    return true;
}

void BleAdvertisementReader::stop() {
    stopRequested_ = true;
}

} // namespace ble

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ble {

// Parsed content of a single BLE advertising report.
struct Advertisement {
    std::string address;         // e.g. "AA:BB:CC:DD:EE:FF"
    bool addressIsRandom = false;
    int rssi = 0;                // dBm, or 127 if unavailable
    std::string name;            // from AD type 0x08/0x09, if present
    std::vector<uint8_t> manufacturerData; // from AD type 0xFF, if present
    uint16_t manufacturerId = 0;
    std::vector<uint8_t> rawData; // full AD payload, unparsed
};

using AdvertisementCallback = std::function<void(const Advertisement&)>;

// Reads BLE (Bluetooth Low Energy) advertisement packets from a local
// Bluetooth adapter using a raw BlueZ HCI socket.
//
// Requires libbluetooth (BlueZ) and CAP_NET_ADMIN / CAP_NET_RAW privileges
// (typically run as root, or grant the binary the capabilities).
class BleAdvertisementReader {
public:
    // deviceId: HCI device index (0 = hci0). -1 lets BlueZ pick a default device.
    explicit BleAdvertisementReader(int deviceId = 0);
    ~BleAdvertisementReader();

    BleAdvertisementReader(const BleAdvertisementReader&) = delete;
    BleAdvertisementReader& operator=(const BleAdvertisementReader&) = delete;

    // Opens the HCI device and configures LE scan parameters.
    // Returns true on success; on failure call lastError() for details.
    bool open();

    // Closes the HCI device, stopping any active scan.
    void close();

    // Enables LE scanning and blocks, invoking `callback` for each
    // advertisement received, until stop() is called from another thread
    // or a signal handler.
    bool startScan(const AdvertisementCallback& callback);

    // Requests that a running startScan() loop return as soon as possible.
    void stop();

    const std::string& lastError() const { return lastError_; }

private:
    int deviceId_;
    int socketFd_ = -1;
    volatile bool stopRequested_ = false;
    std::string lastError_;

    bool setScanParameters();
    bool setScanEnable(bool enable);
};

} // namespace ble

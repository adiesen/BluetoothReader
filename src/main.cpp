#include "BleAdvertisementReader.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>

namespace {

std::atomic<ble::BleAdvertisementReader*> g_reader{nullptr};

void handleSignal(int) {
    if (auto* reader = g_reader.load()) {
        reader->stop();
    }
}

void printAdvertisement(const ble::Advertisement& adv) {
    std::printf("%s (%s)  RSSI=%d dBm", adv.address.c_str(),
                adv.addressIsRandom ? "random" : "public", adv.rssi);
    if (!adv.name.empty()) {
        std::printf("  name=\"%s\"", adv.name.c_str());
    }
    if (!adv.manufacturerData.empty()) {
        std::printf("  mfg=0x%04X data=", adv.manufacturerId);
        for (uint8_t b : adv.manufacturerData) {
            std::printf("%02X", b);
        }
    }
    std::printf("\n");
}

} // namespace

int main(int argc, char** argv) {
    int deviceId = 0;
    if (argc > 1) {
        deviceId = std::atoi(argv[1]);
    }

    ble::BleAdvertisementReader reader(deviceId);
    g_reader = &reader;
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    if (!reader.open()) {
        std::fprintf(stderr, "Failed to open adapter: %s\n", reader.lastError().c_str());
        std::fprintf(stderr, "Note: BLE scanning typically requires root privileges.\n");
        return 1;
    }

    std::printf("Scanning for BLE advertisements on hci%d (press Ctrl+C to stop)...\n", deviceId);
    if (!reader.startScan(printAdvertisement)) {
        std::fprintf(stderr, "Scan failed: %s\n", reader.lastError().c_str());
        return 1;
    }

    reader.close();
    return 0;
}

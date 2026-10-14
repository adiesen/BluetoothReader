#include "BleAdvertisementReader.h"
#include "GattClient.h"
#include "SensorPushReader.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

std::atomic<ble::BleAdvertisementReader*> g_reader{nullptr};

void handleSignal(int) {
    if (auto* reader = g_reader.load()) {
        reader->stop();
    }
}

void printAdvertisement(const ble::Advertisement& adv) {
    std::printf("%s (%s)", adv.address.c_str(),
                adv.addressIsRandom ? "random" : "public");
    if (adv.hasTemperature) {
        std::printf("  temperature=%.1f C", adv.temperatureCelsius);
    }
    if (adv.hasH5127Data) {
        std::printf("  H5127 presence=%s motion=%s",
                    adv.presenceDetected ? "yes" : "no",
                    adv.motionDetected ? "yes" : "no");
    }
    std::printf("  RSSI=%d dBm", adv.rssi);
    if (!adv.name.empty()) {
        std::printf("  name=\"%s\"", adv.name.c_str());
    }
    if (!adv.manufacturerData.empty()) {
        std::printf("  mfg=0x%04X data=", adv.manufacturerId);
        for (uint8_t b : adv.manufacturerData) {
            std::printf("%02X", b);
        }
    }
    std::printf("  advertisement=");
    for (uint8_t b : adv.rawData) {
        std::printf("%02X", b);
    }
    std::printf("\n");
}

void printUsage(const char* program) {
    std::printf("Usage: %s ADDRESS [public|random]\n", program);
    std::printf("       %s --scan [hci-device]\n", program);
    std::printf("       %s --help\n", program);
}

} // namespace

int runGatt(int argc, char** argv) {
    if (argc < 3 || argc > 5) {
        std::fprintf(stderr, "Usage: %s --gatt ADDRESS [public|random] [hci-device]\n", argv[0]);
        return 2;
    }

    bool randomAddress = argc >= 4 && std::string(argv[3]) == "random";
    int deviceId = argc == 5 ? std::atoi(argv[4]) : -1;
    ble::GattClient client(deviceId);
    if (!client.connect(argv[2], randomAddress)) {
        std::fprintf(stderr, "GATT connection failed: %s\n", client.lastError().c_str());
        return 1;
    }

    std::vector<ble::GattService> services;
    if (!client.discoverServices(services)) {
        std::fprintf(stderr, "Service discovery failed: %s\n", client.lastError().c_str());
        return 1;
    }
    for (const auto& service : services) {
        std::printf("service 0x%04X-0x%04X uuid=%s\n", service.startHandle,
                    service.endHandle, service.uuid.c_str());
        std::vector<ble::GattCharacteristic> characteristics;
        if (!client.discoverCharacteristics(service, characteristics)) {
            std::fprintf(stderr, "Characteristic discovery failed: %s\n",
                         client.lastError().c_str());
            return 1;
        }
        for (const auto& characteristic : characteristics) {
            std::printf("  characteristic declaration=0x%04X value=0x%04X properties=0x%02X uuid=%s\n",
                        characteristic.declarationHandle, characteristic.valueHandle,
                        characteristic.properties, characteristic.uuid.c_str());
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--gatt") {
        return runGatt(argc, argv);
    }

    if (argc > 1 && std::string(argv[1]) == "--help") {
        printUsage(argv[0]);
        return 0;
    }
    if (argc == 1 || std::string(argv[1]) == "--scan") {
        if (argc > 3) {
            printUsage(argv[0]);
            return 2;
        }
    } else {
        int addressIndex = 1;
        int addressTypeIndex = 2;
        if (argc > 1 && std::string(argv[1]) == "--read") {
            addressIndex = 2;
            addressTypeIndex = 3;
        }
        if (argc <= addressIndex || argc > addressTypeIndex + 1) {
            printUsage(argv[0]);
            return 2;
        }

        bool isRandom = false;
        if (argc > addressTypeIndex) {
            std::string addressType(argv[addressTypeIndex]);
            if (addressType == "random") {
                isRandom = true;
            } else if (addressType != "public") {
                printUsage(argv[0]);
                return 2;
            }
        }
        ble::SensorPushReading reading;
        std::string error;
        if (!ble::readSensorPush(argv[addressIndex], isRandom, reading, error)) {
            std::fprintf(stderr, "SensorPush read failed: %s\n", error.c_str());
            return 1;
        }
        std::printf("Temperature: %.2f C  Relative humidity: %.2f%%",
                    reading.temperatureC, reading.relativeHumidity);
        if (reading.pressurePa) {
            std::printf("  Pressure: %.2f Pa", *reading.pressurePa);
        }
        if (reading.batteryMillivolts) {
            std::printf("  Battery: %u mV", *reading.batteryMillivolts);
        }
        std::printf("\n");
        return 0;
    }

    int deviceId = 0;
    if (argc == 3) {
        deviceId = std::atoi(argv[2]);
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

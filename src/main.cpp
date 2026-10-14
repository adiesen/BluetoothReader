#include "BleAdvertisementReader.h"
#include "GattClient.h"
#include "SensorPushReader.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <poll.h>
#include <optional>
#include <string>
#include <termios.h>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>

namespace {

std::atomic<bool> g_interrupted{false};

void handleSignal(int) {
    g_interrupted.store(true);
}

void printAdvertisement(const ble::Advertisement& adv) {
    if (!adv.hasSensorPushService) {
        return;
    }

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

struct SensorState {
    ble::Advertisement advertisement;
    std::optional<ble::SensorPushReading> cachedReading;
    std::chrono::system_clock::time_point lastRefresh{};
};

void printSensorReading(const ble::SensorPushReading& reading,
                        std::chrono::system_clock::time_point timestamp) {
    const std::time_t time = std::chrono::system_clock::to_time_t(timestamp);
    std::tm utcTime{};
    gmtime_r(&time, &utcTime);
    char timestampText[32];
    std::strftime(timestampText, sizeof(timestampText), "%Y-%m-%dT%H:%M:%SZ", &utcTime);
    std::printf("Timestamp: %s\n", timestampText);
    std::printf("Temperature: %.2f C  Relative humidity: %.2f%%",
                reading.temperatureC, reading.relativeHumidity);
    if (reading.pressurePa) {
        std::printf("  Pressure: %.2f Pa", *reading.pressurePa);
    }
    if (reading.batteryMillivolts) {
        std::printf("  Battery: %u mV", *reading.batteryMillivolts);
    }
    std::printf("\n");
}

int readAndPrintSensorPush(const std::string& address, bool addressIsRandom,
                           int deviceId = 0) {
    ble::SensorPushReading reading;
    std::string error;
    if (!ble::readSensorPush(address, addressIsRandom, reading, error, deviceId)) {
        std::fprintf(stderr, "SensorPush read failed: %s\n", error.c_str());
        return 1;
    }

    printSensorReading(reading, std::chrono::system_clock::now());
    return 0;
}

int readAndPrintSensorState(SensorState& sensor, int deviceId) {
    ble::SensorPushReading reading;
    std::string error;
    const auto& advertisement = sensor.advertisement;
    if (!ble::readSensorPush(advertisement.address, advertisement.addressIsRandom,
                             reading, error, deviceId)) {
        std::fprintf(stderr, "SensorPush read failed: %s\n", error.c_str());
        if (sensor.cachedReading) {
            std::printf("Using cached reading from the last successful refresh:\n");
            printSensorReading(*sensor.cachedReading, sensor.lastRefresh);
        } else {
            std::printf("No cached sensor reading is available.\n");
        }
        return 1;
    }

    sensor.cachedReading = reading;
    sensor.lastRefresh = std::chrono::system_clock::now();
    printSensorReading(*sensor.cachedReading, sensor.lastRefresh);
    return 0;
}

class TerminalInput {
public:
    ~TerminalInput() {
        restore();
    }

    bool enable(std::string& error) {
        if (!isatty(STDIN_FILENO)) {
            return true;
        }
        if (tcgetattr(STDIN_FILENO, &original_) < 0) {
            error = "Failed to read terminal settings";
            return false;
        }
        termios mode = original_;
        mode.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        mode.c_cc[VMIN] = 0;
        mode.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &mode) < 0) {
            error = "Failed to enable single-key input";
            return false;
        }
        active_ = true;
        return true;
    }

    void restore() {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            active_ = false;
        }
    }

private:
    termios original_{};
    bool active_ = false;
};

int runScan(int deviceId) {
    ble::BleAdvertisementReader reader(deviceId);
    if (!reader.open()) {
        std::fprintf(stderr, "Failed to open adapter: %s\n", reader.lastError().c_str());
        std::fprintf(stderr, "Note: BLE scanning typically requires root privileges.\n");
        return 1;
    }

    std::vector<SensorState> detectedSensors;
    bool newSensorDetected = false;
    auto onAdvertisement = [&](const ble::Advertisement& adv) {
        if (!adv.hasSensorPushService) {
            return;
        }
        auto found = std::find_if(detectedSensors.begin(), detectedSensors.end(),
                                  [&](const SensorState& detected) {
            return detected.advertisement.address == adv.address &&
                   detected.advertisement.addressIsRandom == adv.addressIsRandom;
        });
        if (found != detectedSensors.end()) {
            found->advertisement.rssi = adv.rssi;
            return;
        }
        SensorState sensor;
        sensor.advertisement = adv;
        detectedSensors.push_back(std::move(sensor));
        printAdvertisement(adv);
        newSensorDetected = true;
        reader.stop();
    };

    bool readFailed = false;
    const auto readInterval = std::chrono::seconds(30);
    const auto scanInterval = std::chrono::seconds(120);
    const auto scanDuration = std::chrono::seconds(30);
    auto nextReadAt = std::chrono::steady_clock::time_point::max();
    auto nextScanAt = std::chrono::steady_clock::now();
    auto scanEndsAt = std::chrono::steady_clock::time_point::min();
    bool scanWindowActive = false;
    while (!g_interrupted.load()) {
        auto now = std::chrono::steady_clock::now();
        if (!scanWindowActive && now >= nextScanAt) {
            scanWindowActive = true;
            scanEndsAt = now + scanDuration;
            nextScanAt = now + scanInterval;
            std::printf("Scanning for new SensorPush sensors on hci%d for 30 seconds...\n",
                        deviceId);
        }

        if (scanWindowActive && !newSensorDetected) {
            auto scanUntil = scanEndsAt;
            if (!detectedSensors.empty()) {
                scanUntil = std::min(scanUntil, nextReadAt);
            }
            now = std::chrono::steady_clock::now();
            auto scanWindow = std::chrono::duration_cast<std::chrono::milliseconds>(
                scanUntil - now);
            if (scanWindow > std::chrono::milliseconds::zero() &&
                !reader.startScan(onAdvertisement, scanWindow, &g_interrupted)) {
                std::fprintf(stderr, "Scan failed: %s\n", reader.lastError().c_str());
                reader.close();
                return 1;
            }
            if (g_interrupted.load()) {
                break;
            }
            now = std::chrono::steady_clock::now();
            if (now >= scanEndsAt) {
                scanWindowActive = false;
            }
        }

        now = std::chrono::steady_clock::now();
        const bool periodicReadDue = !detectedSensors.empty() && now >= nextReadAt;
        if (newSensorDetected || periodicReadDue) {
            std::printf("\nReading %zu SensorPush sensor(s)...\n", detectedSensors.size());
            for (auto& sensor : detectedSensors) {
                if (g_interrupted.load()) {
                    break;
                }
                std::printf("%s (%s):\n", sensor.advertisement.address.c_str(),
                            sensor.advertisement.addressIsRandom ? "random" : "public");
                if (readAndPrintSensorState(sensor, deviceId) != 0) {
                    readFailed = true;
                }
            }
            newSensorDetected = false;
            if (!g_interrupted.load()) {
                nextReadAt = std::chrono::steady_clock::now() + readInterval;
            }
            continue;
        }

        if (!scanWindowActive) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    reader.close();
    std::printf("Finished reading SensorPush sensors...\n");
    return readFailed ? 1 : 0;
}

int waitForQuitKey(std::atomic<bool>& scanFinished) {
    TerminalInput terminal;
    std::string error;
    if (!terminal.enable(error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        g_interrupted.store(true);
        return 1;
    }

    std::printf("Press 'q' to stop scanning.\n");
    bool inputClosed = false;
    while (!g_interrupted.load() && !scanFinished.load()) {
        if (inputClosed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        pollfd input{STDIN_FILENO, POLLIN, 0};
        int result = ::poll(&input, 1, 100);
        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::fprintf(stderr, "Failed to read keyboard input: %s\n",
                         std::strerror(errno));
            g_interrupted.store(true);
            return 1;
        }
        if (result == 0) {
            continue;
        }
        if ((input.revents & POLLIN) != 0) {
            char key = '\0';
            ssize_t bytesRead = ::read(STDIN_FILENO, &key, 1);
            if (bytesRead == 1 && (key == 'q' || key == 'Q')) {
                g_interrupted.store(true);
                std::printf("\nStopping scan...\n");
                break;
            }
            if (bytesRead == 0) {
                inputClosed = true;
            }
        }
        if ((input.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
            inputClosed = true;
        }
    }
    terminal.restore();
    return 0;
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
        return readAndPrintSensorPush(argv[addressIndex], isRandom);
    }

    int deviceId = 0;
    if (argc == 3) {
        deviceId = std::atoi(argv[2]);
    }

    g_interrupted.store(false);
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    std::atomic<bool> scanFinished{false};
    int scanResult = 1;
    std::thread scanThread([&] {
        scanResult = runScan(deviceId);
        scanFinished.store(true);
    });
    int inputResult = waitForQuitKey(scanFinished);
    if (inputResult != 0) {
        g_interrupted.store(true);
    }
    scanThread.join();
    return inputResult != 0 ? inputResult : scanResult;
}

# BluetoothReader

A small C++ command-line tool that reads BLE (Bluetooth Low Energy)
advertisement data and GATT services from a local Bluetooth adapter using BlueZ.

## Layout

- `include/BleAdvertisementReader.h` / `src/BleAdvertisementReader.cpp` — reusable
  class that opens an HCI device, enables LE scanning, and parses advertising
  reports (address, RSSI, local name, manufacturer data).
- `include/GattClient.h` / `src/GattClient.cpp` — ATT-over-L2CAP GATT client
  supporting service discovery, characteristic discovery, and value reads.
- `include/SensorPushReader.h` / `src/SensorPushReader.cpp` — SensorPush second-generation
  GATT client for temperature, relative humidity, pressure, and battery voltage.
- `src/main.cpp` — CLI entry point that prints advertisements as they arrive.
- `Makefile` — build rules.
- `scripts/install-deps.sh` — installs the build toolchain and `libbluetooth-dev`.

## Build

```sh
make install-deps   # one-time: installs the C++ toolchain, BlueZ, and D-Bus development files
make                # builds bin/bluetooth_reader
```

## Run

BLE scanning requires elevated privileges:

```sh
sudo bin/bluetooth_reader --scan       # scan on hci0
sudo bin/bluetooth_reader --scan 1     # scan on hci1
```

New SensorPush devices are read immediately. Known sensors are reread every 30 seconds. A 30-second discovery scan runs at startup and every 120 seconds to find new sensors. Press `q` or `Ctrl+C` to exit.

## GATT

Connect to a device and list its services and characteristics:

```sh
sudo bin/bluetooth_reader --gatt AA:BB:CC:DD:EE:FF
sudo bin/bluetooth_reader --gatt AA:BB:CC:DD:EE:FF random
```

The optional `random` argument selects a random peer address. A final HCI device
index can be supplied when more than one adapter is present.

## Read a SensorPush sensor

The SensorPush Bluetooth API exposes readings through GATT characteristics, not
the advertisement payload. First use the scanner above to find the device's
Bluetooth address, then connect and read it:

```sh
sudo bin/bluetooth_reader AA:BB:CC:DD:EE:FF public
```

Use `random` instead of `public` when the address is a random BLE address. The
reader supports SensorPush second-generation devices; the first-generation HT1
does not expose the documented GATT API. Temperature and humidity are returned
in degrees Celsius and percent, respectively. Pressure and battery are printed
when those characteristics are present. During scanning, the latest successful
reading and its refresh timestamp are cached per sensor; a failed refresh prints
that cached snapshot when one is available.

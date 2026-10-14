# BluetoothReader

A small C++ command-line tool that reads BLE (Bluetooth Low Energy)
advertisement data from a local Bluetooth adapter using BlueZ.

## Layout

- `include/BleAdvertisementReader.h` / `src/BleAdvertisementReader.cpp` — reusable
  class that opens an HCI device, enables LE scanning, and parses advertising
  reports (address, RSSI, local name, manufacturer data).
- `include/SensorPushReader.h` / `src/SensorPushReader.cpp` — SensorPush second-generation
  GATT client for temperature, relative humidity, pressure, and battery voltage.
- `src/main.cpp` — CLI entry point that prints advertisements as they arrive.
- `Makefile` — build rules.
- `scripts/install-deps.sh` — installs the build toolchain and `libbluetooth-dev`.

## Build

```sh
make install-deps   # one-time: installs build-essential + libbluetooth-dev
make                # builds bin/bluetooth_reader
```

## Run

BLE scanning requires elevated privileges:

```sh
sudo bin/bluetooth_reader --scan       # scan on hci0
sudo bin/bluetooth_reader --scan 1     # scan on hci1
```

Press `Ctrl+C` to stop scanning.

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
when those characteristics are present.

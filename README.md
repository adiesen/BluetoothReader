# BluetoothReader

A small C++ command-line tool that reads BLE (Bluetooth Low Energy)
advertisement data from a local Bluetooth adapter using BlueZ.

## Layout

- `include/BleAdvertisementReader.h` / `src/BleAdvertisementReader.cpp` — reusable
  class that opens an HCI device, enables LE scanning, and parses advertising
  reports (address, RSSI, local name, manufacturer data).
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
sudo bin/bluetooth_reader        # scan on hci0
sudo bin/bluetooth_reader 1      # scan on hci1
```

Press `Ctrl+C` to stop scanning.

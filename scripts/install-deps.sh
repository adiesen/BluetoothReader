#!/usr/bin/env bash
# Installs build tools and the BlueZ development headers/libraries
# needed to compile BluetoothReader.
set -euo pipefail

if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y build-essential libbluetooth-dev libdbus-1-dev
elif command -v dnf >/dev/null 2>&1; then
    sudo dnf install -y gcc-c++ make bluez-libs-devel dbus-devel
elif command -v pacman >/dev/null 2>&1; then
    sudo pacman -S --needed base-devel bluez-libs dbus
else
    echo "Unsupported package manager. Please install a C++ toolchain and libbluetooth-dev manually." >&2
    exit 1
fi

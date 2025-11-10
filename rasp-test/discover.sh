#!/bin/bash
echo "Scanning I²C devices and drivers..."
for dev in /sys/bus/i2c/devices/*-*; do
    bus=$(basename "$dev" | cut -d- -f1)
    addr=$(basename "$dev" | cut -d- -f2)
    if [ -L "$dev/driver" ]; then
        driver=$(basename "$(readlink "$dev/driver")")
        echo "Bus $bus, address 0x$addr → bound to driver: $driver"
    else
        echo "Bus $bus, address 0x$addr → free (no driver bound)"
    fi
done
# Set up CAN
```
sudo ip link set can0 type can bitrate 1000000 && \
sudo ip link set up can0
```

# Install Python Dependencies
```
python3 -m venv .venv && \
source .venv/bin/activate && \
pip3 install -r requirements.txt
```

# I2C
## Raspberry Pi Pinouts
https://pinout.xyz/

## Raspberry, change configuration, enable I2C
```
sudo raspi-config
```

## Scan I2C Bus
```
i2cdetect -y 1
```
PWM Board should appear at `0x40`
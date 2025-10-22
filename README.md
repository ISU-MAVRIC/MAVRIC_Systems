# Install System Dependecies
```
sudo apt update && sudo apt upgrade -y && \
sudo apt install -y \
  build-essential cmake pkg-config git \
  python3 python3-venv python3-pip \
  python3-dev python3-setuptools python3-wheel \
  can-utils \
  i2c-tools \
  git
```

# Configure eth0 static IP
```
sudo tee -a /etc/dhcpcd.conf > /dev/null <<EOF
interface eth0
# IPv4
static ip_address=192.168.1.9/24
static routers=192.168.1.1

# IPv6
static ip6_address=2001:db8:abcd:1234::50/64

# DNS (both v4 + v6)
static domain_name_servers=8.8.8.8 2001:4860:4860::8888
EOF
```

## Enable SSH
```
sudo raspi-config nonint do_ssh 0
```

# Set up CAN
## CAN Drivers
```
sudo modprobe can && \
sudo modprobe can_raw && \
sudo modprobe gs_usb
```

### Launch Modules at start up
```
sudo tee -a /etc/modules > /dev/null <<EOF
can
can_raw
gs_usb
EOF
```

## CAN0
```
sudo ip link set can0 type can bitrate 1000000 && \
sudo ip link set up can0
```

### Uplink CAN0 on Plug in
```
sudo tee /etc/udev/rules.d/90-canable.rules > /dev/null <<'EOF'
SUBSYSTEM=="net", ACTION=="add", KERNEL=="can*", \
  RUN+="/sbin/ip link set can0 type can bitrate 1000000"
EOF
```



# I2C
## Raspberry Pi Pinouts
https://pinout.xyz/

## Raspberry, change configuration, enable I2C
Enter Configuration Menu
```
sudo raspi-config
```
or use:
```
sudo raspi-config nonint do_i2c 0
```
to enable I2C directly

## Scan I2C Bus
```
i2cdetect -y 1
```
PWM Board should appear at `0x40`


# Install Python Dependencies
Change into `rasp-test` directory first, `cd rasp-test`
```
git clone https://github.com/ISU-MAVRIC/MAVRIC_Systems.git && \
cd MAVRIC_Systems && \
git checkout raspberry-pi-test-andrew && \
cd rasp-test
```

Create Virtual Python Environment and Install Dependecies
```
python3 -m venv .venv && \
source .venv/bin/activate && \
pip3 install -r requirements.txt
```
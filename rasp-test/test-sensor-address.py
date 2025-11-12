from smbus2 import SMBus

bus = SMBus(7)  # use /dev/i2c-7
addr = 0x40

try:
    data = bus.read_byte(addr)
    print(f"Read byte {data} from 0x{addr:02X}")
except OSError as e:
    print("I2C communication failed:", e)
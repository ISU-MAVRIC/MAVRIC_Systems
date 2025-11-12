from smbus2 import SMBus, i2c_msg

bus = SMBus(7)
addr = 0x40

# 0xE7 = read user register (per HTU21D datasheet, which MS8607 humidity chip uses)
bus.write_byte(addr, 0xE7)
data = bus.read_byte(addr)
print(f"User register: 0x{data:02X}")
bus.close()

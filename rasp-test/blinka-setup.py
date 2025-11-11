import board
import busio

print("Attempting to initialize I2C on the FT232H...")

try:
    # This will fail if the FT232H isn't found
    # or if BLINKA_FT232H isn't set.
    i2c = busio.I2C(board.SCL, board.SDA)

    print("I2C Bus Initialized!")
    print("Scanning for devices...")

    # Lock the bus
    while not i2c.try_lock():
        pass

    try:
        # Scan and print results
        devices = i2c.scan()
        if not devices:
            print("No I2C devices found.")
        else:
            print("I2C devices found at (hex):")
            for device in devices:
                print(hex(device))
    
    finally:
        # Always unlock the bus
        i2c.unlock()

except Exception as e:
    print(f"Error: {e}")
    print("Failed to initialize I2C.")
    print("Did you run 'export BLINKA_FT232H=1' first?")
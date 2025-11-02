# CAN Bus Status Message
# Published by the CAN manager with current motor state

uint8 controller_id         # Which motor controller (0-15)
float32 position            # Current position (from status 0x62)
float32 velocity            # Current velocity (from status 0x61, index 0)
float32 current             # Current draw (from status 0x61, index 1)
float32 voltage             # Bus voltage (from status 0x61, index 2)
float32 last_percent_output # Last set percent output command
builtin_interfaces/Time timestamp  # When status was updated

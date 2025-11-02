# CAN Bus Command Message
# Sent by control nodes to command motors via the CAN manager

# Command operation types
uint8 PERCENT_OUTPUT = 1
uint8 VELOCITY_OUTPUT = 2
uint8 POSITION_OUTPUT = 3

uint8 command_type          # Which operation to perform
uint8 controller_id         # Target motor controller (0-15)
float32 value               # Command value (ignored for REQUEST_STATUS)

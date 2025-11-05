# Adafruit ServoKit Control Message
# Support continuous and standard servos

uint8 STANDARD_SERVO = 1
uint8 CONTINUOUS_SERVO = 2

uint8 servo_type          # Type of servo being commanded
uint8 channel             # Target servo channel (0-15)
uint8 value               # Command value (0 to actuation range for standard, -1 to 1 for continuous)
# MAVRIC SparkMax ESC Library

A library for controlling SparkMax Electronic Speed Controllers using python and a CAN Bus.

## IMPORTANT NOTE
*Motor controllers must have ramping enabled if running at more than 50% power, otherwise the motor will try to instantly go to the power level and will stutter.*

## Dependencies

- Python 3 (Not tested on 2.7)
- python-can (https://python-can.readthedocs.io/en/master/)



# Example Control and Encoder Feedback Code

```python
from SparkCAN import SparkBus
import time

#Instantiate SparkBus object
bus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)
#Create a new controller with CAN id 1
sparkESC = bus.init_controller(1)
time.sleep(1)
#Set motor to 50% power
sparkESC.percent_output(.5) #50%
#Prints the velocity 10 times, you should be able to see it ramp up as the motor gets up to speed.
for i in range(10):
    print(sparkESC.velocity)
    time.sleep(0.001)
#wait 5 seconds
time.sleep(5)
#Set motor to 0% power
sparkESC.percent_output(0)
```

## Running Without Hardware / No vcan Device

If the specified CAN interface (e.g. `can0` / `vcan0`) isn't present or python-can can't initialize it, the library automatically enters a lightweight simulation mode. In this mode:

- A dummy in‑memory bus is used so API calls (`send_msg`, controller output methods) do not raise exceptions.
- Heartbeat still iterates but only logs warnings if something goes wrong.
- No real status frames are received, so feedback properties like `velocity` / `position` remain at their initialized values (0) unless you extend the dummy bus to inject frames.

This allows higher-level application code to run on development machines without a physical or virtual CAN device. When a real interface becomes available, simply ensure the CAN device exists before constructing `SparkBus` and hardware communication will resume with no code changes.
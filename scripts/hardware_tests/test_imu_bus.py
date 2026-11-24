import time

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# board.SCL / board.SDA are the Jetson header pins 5 / 3. Which /dev/i2c-N that is depends on the
# JetPack release and on camera overlays (often bus 7 on Orin Nano), so the message below
# reports what actually happened rather than assuming a bus number.
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# If board.SCL does not map to the right bus, open it directly:
# import adafruit_extended_bus
# i2c = adafruit_extended_bus.ExtendedI2C(7)

bno = BNO08X_I2C(i2c, address=0x4A)

# Enable sensors
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)

print("BNO085 initialised on the header I2C bus (address 0x4A).")

# Initialising only proves the chip answered. Read a few samples to prove data is flowing.
for _ in range(5):
    time.sleep(0.2)
    print("accel (m/s^2):", bno.acceleration, " gyro (rad/s):", bno.gyro)

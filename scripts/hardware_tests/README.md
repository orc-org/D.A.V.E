# Hardware Test Scripts

This directory contains standalone hardware diagnostic and verification scripts for the D.A.V.E. rover.

### Scripts

- **`pwmtest.cpp`**: Direct sysfs test for physical pin 32 PWM on Jetson Orin Nano (`/sys/class/pwm/pwmchip3/pwm0`). Generates a 500 Hz square wave.
- **`test_dm556y_steppers.py`**: Stepper motor driver frequency ramping test for DM556Y drivers via Jetson.GPIO hardware PWM.
- **`test_pin12.py`**: Simple 1-second on/off toggle test for physical pin 12 (used for rover lighting / relays).
- **`test_imu_bus.py`**: Direct I2C test for the Adafruit BNO085 IMU on Bus 7 (pins 3 & 5).
- **`test_gps.py`**: Raw NMEA serial listener for GPS GNSS receiver on `/dev/ttyUSB0` at 38400 baud.

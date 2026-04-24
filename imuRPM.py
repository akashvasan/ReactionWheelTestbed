"""
IMU RPM Display
===============
Reads the BNO085 gyroscope and displays all three axes in RPM.
Spin the platform by hand to identify which axis corresponds to
platform rotation, then update PLATFORM_AXIS below.

Wiring:
  BNO085 SDA -> Pin 3 (GPIO2)
  BNO085 SCL -> Pin 5 (GPIO3)
  BNO085 PS1 -> Pin 17 (3.3V)
"""

import time
import math
from collections import deque
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

RAD_S_TO_RPM  = 60.0 / (2.0 * math.pi)
SMOOTH_WINDOW = 5   # number of samples to average

i2c = busio.I2C(board.SCL, board.SDA)
imu = BNO08X_I2C(i2c, address=0x4A)
imu.enable_feature(BNO_REPORT_GYROSCOPE)

buf = deque(maxlen=SMOOTH_WINDOW)

# IMU mounted at -45° in XZ plane: axis = (-1/√2, 0, 1/√2)
# Signed projection: omega = (gz - gx) / √2
AXIS_SCALE = 1.0 / math.sqrt(2)

print("Spin the platform EXACTLY ONE full revolution, then stop.")
print("The 'Total °' column should read ~360 for one revolution.")
print("Press Ctrl+C to reset and exit.\n")
print(f"{'gx (rad/s)':>12}  {'gz (rad/s)':>12}  {'Signed RPM':>12}  {'Total °':>10}")
print("-" * 56)

total_angle_deg = 0.0
last_time = time.monotonic()

try:
    while True:
        time.sleep(0.05)
        now = time.monotonic()
        dt = now - last_time
        last_time = now

        gx, gy, gz = imu.gyro  # rad/s

        signed_rad_s = (gx - gz) * AXIS_SCALE
        buf.append(signed_rad_s * RAD_S_TO_RPM)
        smoothed_rpm = sum(buf) / len(buf)

        # Integrate angle: rad/s × dt → radians → degrees
        total_angle_deg += math.degrees(signed_rad_s * dt)

        print(
            f"\r{gx:>12.3f}  {gz:>12.3f}  {smoothed_rpm:>12.2f}  {total_angle_deg:>10.1f}",
            end="", flush=True
        )
except KeyboardInterrupt:
    print("\nDone.")

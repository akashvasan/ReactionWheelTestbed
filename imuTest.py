"""
IMU Test — BNO085 at 0x4A on I2C bus 1

Existing wiring (do not change):
  GPIO18 (Pin 12) -> RPWM  (forward PWM)   Wire T
  GPIO19 (Pin 35) -> LPWM  (reverse PWM)   Wire U
  GPIO23 (Pin 16) -> R_EN  (speed PWM)     Wire V
  GPIO24 (Pin 18) -> L_EN  (speed PWM)     Wire W
  GPIO17 (Pin 11) -> Encoder Ch A          Wire X
  GPIO27 (Pin 13) -> Encoder Ch B          Wire Y

IMU wiring:
  BNO085 VIN -> Pin 2  (5V) or Pin 1 (3.3V)
  BNO085 GND -> Pin 6  (GND)
  BNO085 SDA -> Pin 3  (GPIO2)
  BNO085 SCL -> Pin 5  (GPIO3)
  BNO085 PS1 -> Pin 17 (3.3V)  -- required for I2C mode
"""

import board, busio, time
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE, BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR

i2c = busio.I2C(board.SCL, board.SDA)
imu = BNO08X_I2C(i2c, address=0x4A)

imu.enable_feature(BNO_REPORT_GYROSCOPE)
imu.enable_feature(BNO_REPORT_ACCELEROMETER)
imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("BNO085 detected — reading IMU data. Press Ctrl+C to stop.\n")
print(f"{'Gx(°/s)':>10} {'Gy(°/s)':>10} {'Gz(°/s)':>10}   {'Ax(m/s²)':>10} {'Ay(m/s²)':>10} {'Az(m/s²)':>10}")

try:
    while True:
        gx, gy, gz = imu.gyro          # rad/s → convert to °/s
        ax, ay, az = imu.acceleration  # m/s²
        print(
            f"\r{gx*57.296:10.2f} {gy*57.296:10.2f} {gz*57.296:10.2f}"
            f"   {ax:10.3f} {ay:10.3f} {az:10.3f}",
            end="", flush=True
        )
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nDone.")

"""
Checkpoint 2 — Platform speed control via IMU feedback
=======================================================
User enters a desired platform rotational speed (RPM).
A PD controller adjusts the motor duty cycle until the IMU-measured
platform speed matches the request.

Wiring:
  GPIO18 (Pin 12) -> RPWM  Wire T
  GPIO19 (Pin 35) -> LPWM  Wire U
  GPIO23 (Pin 16) -> R_EN  Wire V
  GPIO24 (Pin 18) -> L_EN  Wire W
  GPIO17 (Pin 11) -> Encoder Ch A  Wire X
  GPIO27 (Pin 13) -> Encoder Ch B  Wire Y

  BNO085 SDA -> Pin 3 (GPIO2)
  BNO085 SCL -> Pin 5 (GPIO3)
  BNO085 PS1 -> Pin 17 (3.3V)

Usage:
  python3 checkpoint2.py
  Enter platform RPM when prompted (positive = CCW, negative = CW).
  Press Ctrl+C to stop.
"""

import time
import math
import threading

import board
import busio
import lgpio
from gpiozero import PWMOutputDevice, OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------
KP            = 0.4    # proportional gain
KD            = 0.05   # derivative gain
FEEDFORWARD   = 10.0   # base duty % to overcome friction (sign follows target)
SAMPLE_PERIOD = 0.05   # seconds between control updates (20 Hz)
MAX_DUTY      = 20.0   # hard cap on motor duty cycle (%)
SAFETY_RPM    = 50.0   # emergency stop if platform exceeds this speed

# Conversion: gyroscope gives rad/s; 1 rad/s = 60/(2π) RPM
RAD_S_TO_RPM  = 60.0 / (2.0 * math.pi)
# IMU mounted at -45° in XZ plane: signed projection = (gz - gx) / √2
AXIS_SCALE    = 1.0 / math.sqrt(2)

# ---------------------------------------------------------------------------
# Hardware init — wiring matches simpleSpin.py
# ---------------------------------------------------------------------------
factory = LGPIOFactory()

rpwm = OutputDevice(18, initial_value=True,  pin_factory=factory)
lpwm = OutputDevice(19, initial_value=False, pin_factory=factory)
r_en = PWMOutputDevice(23, frequency=1000,   pin_factory=factory)
l_en = PWMOutputDevice(24, frequency=1000,   pin_factory=factory)

i2c = busio.I2C(board.SCL, board.SDA)
imu = BNO08X_I2C(i2c, address=0x4A)
imu.enable_feature(BNO_REPORT_GYROSCOPE)

# ---------------------------------------------------------------------------
# Encoder — same as simpleSpin.py
# ---------------------------------------------------------------------------
COUNTS_PER_REV = 28 * 2 * 5.0   # both edges on Ch A × gear ratio = 280

enc_count = 0
enc_lock  = threading.Lock()
_gpio     = lgpio.gpiochip_open(0)

def _on_edge(chip, pin, level, tick):
    global enc_count
    b = lgpio.gpio_read(_gpio, 27)
    with enc_lock:
        if level == 1:
            enc_count += 1 if b == 0 else -1
        else:
            enc_count += 1 if b == 1 else -1

lgpio.gpio_claim_alert(_gpio, 17, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(_gpio, 27, lgpio.SET_PULL_UP)
_enc_cb = lgpio.callback(_gpio, 17, lgpio.BOTH_EDGES, _on_edge)

# ---------------------------------------------------------------------------
# Motor helper — same scheme as simpleSpin.py
# RPWM/LPWM select direction; R_EN/L_EN set speed via PWM
# ---------------------------------------------------------------------------
def set_duty(duty_pct: float):
    duty_pct = max(-MAX_DUTY, min(MAX_DUTY, duty_pct))
    duty = abs(duty_pct) / 100.0
    if duty_pct > 0:
        rpwm.on()
        lpwm.off()
    elif duty_pct < 0:
        rpwm.off()
        lpwm.on()
    else:
        rpwm.off()
        lpwm.off()
    r_en.value = duty
    l_en.value = duty

def stop_motor():
    rpwm.off()
    lpwm.off()
    r_en.value = 0
    l_en.value = 0

# ---------------------------------------------------------------------------
# IMU helper — read platform rotational speed in RPM
# gz is the yaw axis (platform spinning on its vertical axis).
# If the wrong axis spins, change gz to gx or gy below.
# ---------------------------------------------------------------------------
def platform_rpm_from_imu() -> float:
    gx, gy, gz = imu.gyro   # rad/s
    return (gx - gz) * AXIS_SCALE * RAD_S_TO_RPM

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    try:
        target_rpm = float(input("Enter desired platform RPM (e.g. 10, -10): "))
    except ValueError:
        print("Invalid input — exiting.")
        return

    print(f"\nTarget: {target_rpm:.1f} RPM  |  Kp={KP}  Kd={KD}")
    print(f"{'Target RPM':>12}  {'IMU RPM':>12}  {'Motor RPM':>12}  {'Error':>10}  {'Duty %':>8}")
    print("-" * 62)

    prev_error  = 0.0
    prev_time   = time.monotonic()
    last_count  = 0

    try:
        while True:
            time.sleep(SAMPLE_PERIOD)
            now = time.monotonic()
            dt  = now - prev_time
            prev_time = now

            # --- Measure platform speed from IMU ---
            measured_rpm = platform_rpm_from_imu()

            # --- Measure motor/wheel speed from encoder ---
            with enc_lock:
                count = enc_count
            wheel_rpm = ((count - last_count) / COUNTS_PER_REV) / (dt / 60.0)
            last_count = count

            # Safety stop
            if abs(measured_rpm) > SAFETY_RPM:
                stop_motor()
                print(f"\n\nSAFETY STOP: IMU read {measured_rpm:.1f} RPM (limit {SAFETY_RPM} RPM)")
                break

            # --- PD control (negative: positive duty → CW → negative IMU) ---
            error      = target_rpm - measured_rpm
            derivative = (error - prev_error) / dt if dt > 0 else 0.0
            prev_error = error

            ff   = -math.copysign(FEEDFORWARD, target_rpm) if target_rpm != 0 else 0.0
            duty = -(KP * error + KD * derivative) + ff

            set_duty(duty)

            print(
                f"\r{target_rpm:>12.1f}  {measured_rpm:>12.2f}  {wheel_rpm:>12.1f}  {error:>10.2f}  {duty:>8.1f}",
                end="", flush=True
            )

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        stop_motor()
        _enc_cb.cancel()
        lgpio.gpiochip_close(_gpio)
        rpwm.close()
        lpwm.close()
        r_en.close()
        l_en.close()
        print("Done.")

if __name__ == "__main__":
    main()

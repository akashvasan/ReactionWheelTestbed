"""
Demo — Perturbation rejection (return-to-zero)
===============================================
The testbed targets 0 RPM at all times.  When a mentor spins the platform,
the PD controller drives the reaction wheel to bring the platform back to
rest as quickly as possible.

Wiring:
  GPIO25 (Pin 22) -> RPWM  Wire T
  GPIO22 (Pin 15) -> LPWM  Wire U
  GPIO23 (Pin 16) -> R_EN  Wire V
  GPIO24 (Pin 18) -> L_EN  Wire W
  GPIO17 (Pin 11) -> Encoder Ch A  Wire X
  GPIO27 (Pin 13) -> Encoder Ch B  Wire Y

  BNO085 SDA -> Pin 3 (GPIO2)
  BNO085 SCL -> Pin 5 (GPIO3)
  BNO085 PS1 -> Pin 17 (3.3V)

Usage:
  python3 demo_perturbation.py
  Press Ctrl+C to stop.
"""

import time
import math
import threading
import csv

import board
import busio
import lgpio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------
KP            = 1    # proportional gain
KD            = 0.0    # derivative disabled — KD causes sign flip during fast deceleration
K_WHEEL_BRAKE = 0.05   # wheel momentum damping gain (used when platform is near zero)
DEADBAND_RPM  = 0.3    # ignore error smaller than this to avoid fighting IMU noise
SAMPLE_PERIOD = 0.05   # seconds between control updates (20 Hz)
MAX_DUTY      = 50.0   # hard cap on motor duty cycle (%)
SAFETY_RPM    = 150.0  # emergency stop if testbed platform exceeds this speed
MOTOR_SAFETY_RPM = 120.0  # emergency stop if reaction wheel motor exceeds this speed

TARGET_RPM    = 0.0    # always hold the platform at rest

# Conversion: gyroscope gives rad/s; 1 rad/s = 60/(2π) RPM
RAD_S_TO_RPM  = 60.0 / (2.0 * math.pi)
# IMU mounted at -45° in XZ plane: signed projection = (gx - gz) / √2
AXIS_SCALE    = 1.0 / math.sqrt(2)

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------
RPWM_PIN = 25   # Pin 22
LPWM_PIN = 22   # Pin 15
R_EN_PIN = 23   # Pin 16
L_EN_PIN = 24   # Pin 18
PWM_FREQ = 1000

i2c = busio.I2C(board.SCL, board.SDA)
imu = BNO08X_I2C(i2c, address=0x4A)
imu.enable_feature(BNO_REPORT_GYROSCOPE)

# ---------------------------------------------------------------------------
# Encoder
# ---------------------------------------------------------------------------
COUNTS_PER_REV = 28 * 2 * 5.0   # both edges on Ch A × gear ratio = 280

enc_count = 0
enc_lock  = threading.Lock()
_gpio = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(_gpio, RPWM_PIN, 0)
lgpio.gpio_claim_output(_gpio, LPWM_PIN, 0)
lgpio.gpio_claim_output(_gpio, R_EN_PIN, 0)
lgpio.gpio_claim_output(_gpio, L_EN_PIN, 0)

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
# Motor helpers
# ---------------------------------------------------------------------------
def set_duty(duty_pct: float):
    duty_pct = max(-MAX_DUTY, min(MAX_DUTY, duty_pct))
    pwm = abs(duty_pct)
    if duty_pct > 0:
        lgpio.gpio_write(_gpio, RPWM_PIN, 1)
        lgpio.gpio_write(_gpio, LPWM_PIN, 0)
        lgpio.tx_pwm(_gpio, R_EN_PIN, PWM_FREQ, pwm)
        lgpio.gpio_write(_gpio, L_EN_PIN, 1)
    elif duty_pct < 0:
        lgpio.gpio_write(_gpio, RPWM_PIN, 0)
        lgpio.gpio_write(_gpio, LPWM_PIN, 1)
        lgpio.gpio_write(_gpio, R_EN_PIN, 1)
        lgpio.tx_pwm(_gpio, L_EN_PIN, PWM_FREQ, pwm)
    else:
        lgpio.gpio_write(_gpio, RPWM_PIN, 0)
        lgpio.gpio_write(_gpio, LPWM_PIN, 0)
        lgpio.tx_pwm(_gpio, R_EN_PIN, PWM_FREQ, 0)
        lgpio.tx_pwm(_gpio, L_EN_PIN, PWM_FREQ, 0)

def stop_motor():
    lgpio.gpio_write(_gpio, RPWM_PIN, 0)
    lgpio.gpio_write(_gpio, LPWM_PIN, 0)
    lgpio.tx_pwm(_gpio, R_EN_PIN, PWM_FREQ, 0)
    lgpio.tx_pwm(_gpio, L_EN_PIN, PWM_FREQ, 0)

# ---------------------------------------------------------------------------
# IMU helper
# ---------------------------------------------------------------------------
def platform_rpm_from_imu():
    gx, gy, gz = imu.gyro   # rad/s
    return (gx - gz) * AXIS_SCALE * RAD_S_TO_RPM, gx, gz

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    log_path = "/home/fri/Desktop/ReactionWheelTestbed/perturbation_log.csv"
    print("Perturbation-rejection controller starting.")
    print(f"Target: {TARGET_RPM:.0f} RPM  |  Kp={KP}  Kd={KD}  Platform limit={SAFETY_RPM} RPM  Motor limit={MOTOR_SAFETY_RPM} RPM")
    print(f"Logging to {log_path}")
    print("Spin the platform — it will return to 0 RPM automatically.")
    print("Press Ctrl+C to stop.\n")
    print(f"{'IMU RPM':>10}  {'Wheel RPM':>10}  {'Error':>8}  {'Duty %':>8}  {'State':>8}")
    print("-" * 55)

    prev_error = 0.0
    prev_time  = time.monotonic()
    last_count = 0
    t0         = time.monotonic()

    log_file = open(log_path, "w", newline="")
    logger   = csv.writer(log_file)
    logger.writerow(["t_s", "gx", "gz", "imu_rpm", "wheel_rpm", "error", "duty", "state"])

    try:
        while True:
            time.sleep(SAMPLE_PERIOD)
            now = time.monotonic()
            dt  = now - prev_time
            prev_time = now

            measured_rpm, gx, gz = platform_rpm_from_imu()

            with enc_lock:
                count = enc_count
            wheel_rpm = ((count - last_count) / COUNTS_PER_REV) / (dt / 60.0)
            last_count = count

            # Safety stops
            if abs(measured_rpm) > SAFETY_RPM:
                stop_motor()
                print(f"\n\nSAFETY STOP: platform {measured_rpm:.1f} RPM (limit {SAFETY_RPM} RPM)")
                break
            if abs(wheel_rpm) > MOTOR_SAFETY_RPM:
                stop_motor()
                print(f"\n\nSAFETY STOP: motor {wheel_rpm:.1f} RPM (limit {MOTOR_SAFETY_RPM} RPM)")
                break

            error      = TARGET_RPM - measured_rpm   # = -measured_rpm
            derivative = (error - prev_error) / dt if dt > 0 else 0.0
            prev_error = error

            if abs(error) < DEADBAND_RPM:
                # Platform near zero — brake the wheel to bleed off stored momentum
                duty  = -K_WHEEL_BRAKE * wheel_rpm
                state = "  HOLD"
            else:
                duty  = -(KP * error + KD * derivative)
                state = "ACTIVE"

            set_duty(duty)

            logger.writerow([
                f"{now - t0:.3f}", f"{gx:.4f}", f"{gz:.4f}",
                f"{measured_rpm:.3f}", f"{wheel_rpm:.3f}",
                f"{error:.3f}", f"{duty:.3f}", state.strip()
            ])
            print(
                f"\r{measured_rpm:>10.2f}  {wheel_rpm:>10.1f}  {error:>8.2f}  {duty:>8.1f}  {state:>8}",
                end="", flush=True
            )

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        log_file.close()
        stop_motor()
        _enc_cb.cancel()
        lgpio.gpiochip_close(_gpio)
        print("Done.")

if __name__ == "__main__":
    main()

"""
Camera Tracking — Color-based reaction wheel controller
=========================================================
Detects a lime-green target in the camera frame and drives the
reaction wheel to keep it horizontally centered.

Error  = (centroid_x - frame_center_x) / frame_center_x   [-1.0 … +1.0]
Duty   = KP * error   (clamped to ±MAX_DUTY)

Run with: /usr/bin/python3 cameraTracking.py
Press Ctrl+C to stop.
"""

import time
import threading
import csv
import cv2
import numpy as np
import lgpio
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------
KP            = 20.0   # proportional gain (scales [-1,1] error to duty %)
KD            = 1.0    # derivative gain (dampens oscillation)
MAX_DUTY      = 50.0   # hard cap on motor duty (%)
DEADBAND      = 0.03   # ignore error smaller than this fraction of half-width
SAMPLE_PERIOD = 0.05   # seconds between control updates (20 Hz)
SEARCH_DUTY   = 15.0   # duty cycle (%) while scanning for target

# Bright yellow HSV range (OpenCV: H 0-180, S 0-255, V 0-255)
HSV_LOWER = np.array([ 20, 180, 180])
HSV_UPPER = np.array([ 35, 255, 255])

MIN_BLOB_AREA = 500    # pixels — ignore tiny detections

# ---------------------------------------------------------------------------
# Pin definitions
# ---------------------------------------------------------------------------
RPWM_PIN = 25
LPWM_PIN = 22
R_EN_PIN = 23
L_EN_PIN = 24
PWM_FREQ = 1000

# ---------------------------------------------------------------------------
# Motor init
# ---------------------------------------------------------------------------
_gpio = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(_gpio, RPWM_PIN, 0)
lgpio.gpio_claim_output(_gpio, LPWM_PIN, 0)
lgpio.gpio_claim_output(_gpio, R_EN_PIN, 0)
lgpio.gpio_claim_output(_gpio, L_EN_PIN, 0)

def set_duty(duty_pct: float):
    duty_pct = max(-MAX_DUTY, min(MAX_DUTY, duty_pct))
    pwm = abs(duty_pct)
    if duty_pct > 0:
        lgpio.gpio_write(_gpio, RPWM_PIN, 1)
        lgpio.gpio_write(_gpio, LPWM_PIN, 0)
        lgpio.tx_pwm(_gpio, R_EN_PIN, PWM_FREQ, pwm)
        lgpio.tx_pwm(_gpio, L_EN_PIN, PWM_FREQ, 100)
    elif duty_pct < 0:
        lgpio.gpio_write(_gpio, RPWM_PIN, 0)
        lgpio.gpio_write(_gpio, LPWM_PIN, 1)
        lgpio.tx_pwm(_gpio, R_EN_PIN, PWM_FREQ, 100)
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
# Camera init
# ---------------------------------------------------------------------------
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)
print("Camera ready.")

FRAME_W, FRAME_H = 640, 480
CX = FRAME_W // 2   # frame center x

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    log_path = "/home/fri/Desktop/ReactionWheelTestbed/tracking_log.csv"
    print(f"Camera tracking started. KP={KP}  MAX_DUTY={MAX_DUTY}%  Deadband={DEADBAND}")
    print(f"Logging to {log_path}")
    print("Hold the lime-green target in view. Press Ctrl+C to stop.\n")
    print(f"{'centroid_x':>12}  {'error':>8}  {'duty %':>8}  {'state':>10}")
    print("-" * 50)

    log_file = open(log_path, "w", newline="")
    logger   = csv.writer(log_file)
    logger.writerow(["t_s", "centroid_x", "error", "duty", "state"])
    t0         = time.monotonic()
    prev_error = 0.0
    prev_time  = time.monotonic()

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

            # Find largest blob
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centroid_x = None
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) >= MIN_BLOB_AREA:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        centroid_x = int(M["m10"] / M["m00"])

            now = time.monotonic()
            dt  = now - prev_time
            prev_time = now

            if centroid_x is None:
                duty       = SEARCH_DUTY
                set_duty(duty)
                state      = " SEARCH"
                error      = 0.0
                prev_error = 0.0
            else:
                error      = (centroid_x - CX) / CX   # -1.0 to +1.0
                derivative = (error - prev_error) / dt if dt > 0 else 0.0
                prev_error = error
                if abs(error) < DEADBAND:
                    stop_motor()
                    state = "CENTERED"
                    duty  = 0.0
                else:
                    duty = KP * error + KD * derivative
                    set_duty(duty)
                    state = "TRACKING"

            logger.writerow([f"{now - t0:.3f}", centroid_x, f"{error:.4f}", f"{duty:.2f}", state])
            print(
                f"\r{str(centroid_x):>12}  {error:>8.3f}  {duty:>8.1f}  {state:>10}",
                end="", flush=True
            )

            time.sleep(SAMPLE_PERIOD)

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        log_file.close()
        stop_motor()
        lgpio.gpiochip_close(_gpio)
        picam2.stop()
        print("Done.")

if __name__ == "__main__":
    main()

"""
Reaction Wheel Motor Controller (Pi 5 compatible)
===================================================
Uses lgpio directly — no gpiozero layer.

Wiring (from wire table):
  GPIO25 (Pin 22) -> RPWM          (forward direction)  Wire T
  GPIO22 (Pin 15) -> LPWM          (reverse direction)  Wire U
  GPIO23 (Pin 16) -> R_EN          (right enable PWM)   Wire V
  GPIO24 (Pin 18) -> L_EN          (left enable PWM)    Wire W
  GPIO17 (Pin 11) -> Encoder Ch A                       Wire X
  GPIO27 (Pin 13) -> Encoder Ch B                       Wire Y
"""

import time
import threading
import lgpio

# --- Pin Definitions (BCM numbering) ---
RPWM_PIN  = 25   # Pin 22 — plain GPIO (GPIO18/19 unusable on Pi 5 via lgpio)
LPWM_PIN  = 22   # Pin 15 — plain GPIO
R_EN_PIN  = 23
L_EN_PIN  = 24
ENC_A_PIN = 17
ENC_B_PIN = 27

# --- Motor / Gearbox Constants ---
PWM_FREQ          = 1000   # Hz
GEAR_RATIO        = 5.0
MOTOR_MAX_RPM     = 6000
FLYWHEEL_MAX_RPM  = MOTOR_MAX_RPM / GEAR_RATIO   # 1200 RPM
COUNTS_PER_REV    = 28 * 2 * GEAR_RATIO          # 280 counts per flywheel rev
RPM_SAMPLE_PERIOD = 0.1

# --- SET DUTY CYCLE HERE ---
# Range: -100.0 to +100.0  (positive = forward, negative = reverse)
TARGET_DUTY = 10.0


class ReactionWheel:
    def __init__(self):
        self._gpio = lgpio.gpiochip_open(0)

        lgpio.gpio_claim_output(self._gpio, RPWM_PIN, 0)
        lgpio.gpio_claim_output(self._gpio, LPWM_PIN, 0)
        lgpio.gpio_claim_output(self._gpio, R_EN_PIN, 0)
        lgpio.gpio_claim_output(self._gpio, L_EN_PIN, 0)

        # Encoder pins
        lgpio.gpio_claim_alert(self._gpio, ENC_A_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self._gpio, ENC_B_PIN, lgpio.SET_PULL_UP)
        self._enc_cb = lgpio.callback(self._gpio, ENC_A_PIN, lgpio.BOTH_EDGES, self._encoder_callback)

        # State
        self._encoder_count = 0
        self._last_count    = 0
        self._last_time     = time.monotonic()
        self._measured_rpm  = 0.0
        self._lock          = threading.Lock()
        self._duty          = 0.0

        self._running = True
        self._rpm_thread = threading.Thread(target=self._rpm_loop, daemon=True)
        self._rpm_thread.start()

        print(f"Reaction wheel ready. Max flywheel speed: ±{FLYWHEEL_MAX_RPM:.0f} RPM")

    def _encoder_callback(self, chip, gpio, level, tick):
        b = lgpio.gpio_read(self._gpio, ENC_B_PIN)
        with self._lock:
            if level == 1:
                self._encoder_count += 1 if b == 0 else -1
            else:
                self._encoder_count += 1 if b == 1 else -1

    def _rpm_loop(self):
        while self._running:
            time.sleep(RPM_SAMPLE_PERIOD)
            now = time.monotonic()
            with self._lock:
                delta_counts = self._encoder_count - self._last_count
                self._last_count = self._encoder_count
            delta_time = now - self._last_time
            self._last_time = now
            self._measured_rpm = (delta_counts / COUNTS_PER_REV) / (delta_time / 60.0)

    def _set_speed(self, duty: float):
        duty = max(-100.0, min(100.0, duty))
        self._duty = duty
        pwm = abs(duty)

        if duty > 0:
            lgpio.gpio_write(self._gpio, RPWM_PIN, 1)
            lgpio.gpio_write(self._gpio, LPWM_PIN, 0)
            lgpio.tx_pwm(self._gpio, R_EN_PIN, PWM_FREQ, pwm)
            lgpio.gpio_write(self._gpio, L_EN_PIN, 1)
        elif duty < 0:
            lgpio.gpio_write(self._gpio, RPWM_PIN, 0)
            lgpio.gpio_write(self._gpio, LPWM_PIN, 1)
            lgpio.gpio_write(self._gpio, R_EN_PIN, 1)
            lgpio.tx_pwm(self._gpio, L_EN_PIN, PWM_FREQ, pwm)
        else:
            lgpio.gpio_write(self._gpio, RPWM_PIN, 0)
            lgpio.gpio_write(self._gpio, LPWM_PIN, 0)
            lgpio.tx_pwm(self._gpio, R_EN_PIN, PWM_FREQ, 0)
            lgpio.tx_pwm(self._gpio, L_EN_PIN, PWM_FREQ, 0)

    def stop(self):
        self._set_speed(0)

    def cleanup(self):
        self._running = False
        self._set_speed(0)
        self._enc_cb.cancel()
        lgpio.gpiochip_close(self._gpio)
        print("\nGPIO cleaned up.")

    def run(self, duty: float):
        direction = "FWD" if duty >= 0 else "REV"
        print(f"\nDuty: {duty:+.1f}%  ({direction})")
        print("-" * 60)
        self._set_speed(duty)
        print("Press Ctrl+C to stop\n")
        try:
            while True:
                self._print_status()
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n\nStopping...")
            self.stop()
            time.sleep(2.0)

    def _print_status(self):
        measured  = self._measured_rpm
        duty      = self._duty
        direction = "FWD" if duty >= 0 else "REV"
        with self._lock:
            enc = self._encoder_count
        print(
            f"\r{direction}  duty={duty:+6.1f}%  "
            f"ACTUAL: {measured:7.1f} RPM  "
            f"enc={enc}",
            end="", flush=True
        )


if __name__ == "__main__":
    wheel = ReactionWheel()
    try:
        wheel.run(TARGET_DUTY)
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        wheel.cleanup()

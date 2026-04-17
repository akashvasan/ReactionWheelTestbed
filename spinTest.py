"""
Reaction Wheel Motor Controller (Pi 5 compatible)
===================================================
Uses gpiozero + lgpio backend.

Wiring (from wire table):
  GPIO18 (Pin 12) -> RPWM          (forward PWM)    Wire T
  GPIO19 (Pin 35) -> LPWM          (reverse PWM)    Wire U
  GPIO23 (Pin 16) -> R_EN          (right enable)   Wire V
  GPIO24 (Pin 18) -> L_EN          (left enable)    Wire W
  GPIO17 (Pin 11) -> Encoder Ch A                   Wire X
  GPIO27 (Pin 13) -> Encoder Ch B                   Wire Y
"""

import time
import threading
from gpiozero import PWMOutputDevice, OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
import lgpio

factory = LGPIOFactory()

# --- Pin Definitions (BCM numbering) ---
RPWM_PIN = 18
LPWM_PIN = 19
R_EN_PIN = 23
L_EN_PIN = 24
ENC_A_PIN = 17
ENC_B_PIN = 27

# --- Motor / Gearbox Constants ---
PWM_FREQ          = 1000   # Hz
GEAR_RATIO        = 5.0    # 5:1 reduction
MOTOR_MAX_RPM     = 6000   # REV HD Hex Motor free-speed RPM
FLYWHEEL_MAX_RPM  = MOTOR_MAX_RPM / GEAR_RATIO   # 1200 RPM
COUNTS_PER_REV    = 28 * 2 * GEAR_RATIO          # 280 counts per flywheel revolution (both edges on Ch A)
RPM_SAMPLE_PERIOD = 0.1    # seconds between RPM/PID updates

# --- PD Gains (tune these) ---
KP = 0.05   # proportional — keep low to avoid oscillation on inertial load
KD = 0.01   # derivative — damps overshoot

TARGET_RPM = -25  # <-- SET YOUR DESIRED FLYWHEEL RPM HERE (±1200 max, negative = reverse)


class ReactionWheel:
    def __init__(self):
        # Motor outputs
        self.rpwm = PWMOutputDevice(RPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.lpwm = PWMOutputDevice(LPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.r_en = OutputDevice(R_EN_PIN, initial_value=True, pin_factory=factory)
        self.l_en = OutputDevice(L_EN_PIN, initial_value=True, pin_factory=factory)

        # Encoder state
        self._encoder_count = 0
        self._last_count    = 0
        self._last_time     = time.monotonic()
        self._measured_rpm  = 0.0
        self._lock          = threading.Lock()

        # PD state
        self._target_rpm  = 0.0
        self._prev_error  = 0.0
        self._duty        = 0.0

        # Set up encoder GPIO via lgpio directly
        self._gpio = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_alert(self._gpio, ENC_A_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self._gpio, ENC_B_PIN, lgpio.SET_PULL_UP)
        self._enc_cb = lgpio.callback(self._gpio, ENC_A_PIN, lgpio.BOTH_EDGES, self._encoder_callback)

        # Background thread: measure RPM + run PID
        self._running = True
        self._rpm_thread = threading.Thread(target=self._rpm_loop, daemon=True)
        self._rpm_thread.start()

        print(f"Reaction wheel ready. Max flywheel speed: {FLYWHEEL_MAX_RPM:.0f} RPM")

    # ------------------------------------------------------------------
    # Encoder callback — called on every edge of Channel A
    # ------------------------------------------------------------------
    def _encoder_callback(self, chip, gpio, level, tick):
        b = lgpio.gpio_read(self._gpio, ENC_B_PIN)
        with self._lock:
            if level == 1:
                self._encoder_count += 1 if b == 0 else -1
            else:
                self._encoder_count += 1 if b == 1 else -1

    # ------------------------------------------------------------------
    # Background thread — measures RPM and updates PID every sample period
    # ------------------------------------------------------------------
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

            # PD with feedforward
            error = self._target_rpm - self._measured_rpm
            derivative = (error - self._prev_error) / delta_time
            self._prev_error = error

            feedforward = (self._target_rpm / FLYWHEEL_MAX_RPM) * 100.0
            duty = feedforward + KP * error + KD * derivative
            self._set_speed(duty)

    # ------------------------------------------------------------------
    # Core speed setter — duty cycle -100 to +100
    # ------------------------------------------------------------------
    def _set_speed(self, speed: float):
        speed = max(-100.0, min(100.0, speed))
        self._duty = speed
        duty = abs(speed) / 100.0

        if speed > 0:
            self.rpwm.value = duty
            self.lpwm.value = 0
        elif speed < 0:
            self.rpwm.value = 0
            self.lpwm.value = duty
        else:
            self.rpwm.value = 0
            self.lpwm.value = 0

    def set_target_rpm(self, rpm: float):
        self._target_rpm = max(-FLYWHEEL_MAX_RPM, min(FLYWHEEL_MAX_RPM, rpm))
        self._prev_error = 0.0

    def stop(self):
        self.set_target_rpm(0)

    def cleanup(self):
        self._running = False
        self._set_speed(0)
        self._enc_cb.cancel()
        self.rpwm.close()
        self.lpwm.close()
        self.r_en.close()
        self.l_en.close()
        lgpio.gpiochip_close(self._gpio)
        print("\nGPIO cleaned up.")

    # ------------------------------------------------------------------
    # Run: set target and hold, printing live status
    # ------------------------------------------------------------------
    def run(self, target_rpm: float):
        print(f"\nTarget: {target_rpm:.0f} RPM  (max: ±{FLYWHEEL_MAX_RPM:.0f} RPM)")
        print(f"PD gains — Kp={KP}  Kd={KD}")
        print("-" * 60)
        self.set_target_rpm(target_rpm)
        print("Press Ctrl+C to stop\n")
        try:
            while True:
                self._print_status()
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n\nStopping...")
            self.stop()
            time.sleep(2.0)

    # ------------------------------------------------------------------
    # Terminal output
    # ------------------------------------------------------------------
    def _print_status(self):
        target    = self._target_rpm
        measured  = self._measured_rpm
        pct       = abs(self._duty)
        direction = "FWD" if target >= 0 else "REV"
        with self._lock:
            enc = self._encoder_count
        print(
            f"\r{direction}  duty={pct:5.1f}%  "
            f"TARGET: {target:7.1f} RPM  "
            f"ACTUAL: {measured:7.1f} RPM  "
            f"enc={enc}",
            end="", flush=True
        )


# --- Entry point ---
if __name__ == "__main__":
    wheel = ReactionWheel()
    try:
        wheel.run(TARGET_RPM)
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        wheel.cleanup()

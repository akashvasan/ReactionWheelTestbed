"""
Reaction Wheel Motor Controller (Pi 5 compatible)
===================================================
Uses gpiozero + lgpio backend.

Run first: sudo systemctl start pigpiod

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
COUNTS_PER_REV    = 28 * GEAR_RATIO              # 140 counts per flywheel revolution (quadrature 4x = 560)
RPM_SAMPLE_PERIOD = 0.1    # seconds between RPM calculations

TARGET_RPM = 600  # <-- SET YOUR DESIRED FLYWHEEL RPM HERE (±1200 max, negative = reverse)


class ReactionWheel:
    def __init__(self):
        # Motor outputs
        self.rpwm = PWMOutputDevice(RPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.lpwm = PWMOutputDevice(LPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.r_en = OutputDevice(R_EN_PIN, initial_value=True, pin_factory=factory)
        self.l_en = OutputDevice(L_EN_PIN, initial_value=True, pin_factory=factory)

        # Encoder state
        self._encoder_count = 0
        self._last_count = 0
        self._last_time = time.monotonic()
        self._measured_rpm = 0.0
        self._lock = threading.Lock()

        # Set up encoder GPIO via lgpio directly
        self._gpio = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(self._gpio, ENC_A_PIN)
        lgpio.gpio_claim_input(self._gpio, ENC_B_PIN)
        lgpio.gpio_claim_alert(self._gpio, ENC_A_PIN, lgpio.BOTH_EDGES)
        lgpio.callback(self._gpio, ENC_A_PIN, lgpio.BOTH_EDGES, self._encoder_callback)

        # Background thread to compute RPM
        self._running = True
        self._rpm_thread = threading.Thread(target=self._rpm_loop, daemon=True)
        self._rpm_thread.start()

        self._speed = 0
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
    # Background thread — recalculates RPM every RPM_SAMPLE_PERIOD
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
            # counts / counts_per_rev / time_in_min = RPM
            self._measured_rpm = (delta_counts / COUNTS_PER_REV) / (delta_time / 60.0)

    # ------------------------------------------------------------------
    # Core speed setter (motor duty cycle, -100 to +100)
    # ------------------------------------------------------------------
    def set_speed(self, speed: float):
        speed = max(-100.0, min(100.0, speed))
        self._speed = speed
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

    def set_flywheel_rpm(self, rpm: float):
        rpm = max(-FLYWHEEL_MAX_RPM, min(FLYWHEEL_MAX_RPM, rpm))
        self.set_speed((rpm / FLYWHEEL_MAX_RPM) * 100.0)

    def ramp_to_rpm(self, target_rpm: float, ramp_duration: float = 1.5):
        """Smoothly ramp to a target RPM."""
        start_rpm = self.commanded_rpm
        steps = 50
        delay = ramp_duration / steps
        for i in range(steps + 1):
            interp = start_rpm + (target_rpm - start_rpm) * (i / steps)
            self.set_flywheel_rpm(interp)
            time.sleep(delay)

    def run(self, target_rpm: float, ramp_duration: float = 1.5):
        """
        Ramp to target_rpm and hold, printing live status.
        Press Ctrl+C to stop.
        """
        print(f"\nTarget: {target_rpm:.0f} RPM  (max: ±{FLYWHEEL_MAX_RPM:.0f} RPM)")
        print("-" * 55)

        self.ramp_to_rpm(target_rpm, ramp_duration)

        print("\nHolding speed — press Ctrl+C to stop\n")
        try:
            while True:
                self._print_status()
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n\nStopping...")
            self.ramp_to_rpm(0, ramp_duration=1.0)

    def stop(self):
        self.set_speed(0)

    def cleanup(self):
        self._running = False
        self.stop()
        self.rpwm.close()
        self.lpwm.close()
        self.r_en.close()
        self.l_en.close()
        lgpio.gpiochip_close(self._gpio)
        print("\nGPIO cleaned up.")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def commanded_rpm(self) -> float:
        """RPM calculated from duty cycle (what we asked for)."""
        return (self._speed / 100.0) * FLYWHEEL_MAX_RPM

    @property
    def measured_rpm(self) -> float:
        """Actual RPM measured from encoder."""
        return self._measured_rpm

    @property
    def speed_percent(self) -> float:
        return abs(self._speed)

    # ------------------------------------------------------------------
    # Terminal output
    # ------------------------------------------------------------------
    def _print_status(self):
        cmd_rpm = self.commanded_rpm
        meas_rpm = self.measured_rpm
        pct = self.speed_percent
        direction = "FWD" if cmd_rpm >= 0 else "REV"
        bar_len = int(pct / 2)
        bar = "█" * bar_len + "░" * (50 - bar_len)
        print(
            f"\r{direction} |{bar}| {pct:5.1f}%  "
            f"CMD: {abs(cmd_rpm):6.1f} RPM  "
            f"ACTUAL: {abs(meas_rpm):6.1f} RPM",
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
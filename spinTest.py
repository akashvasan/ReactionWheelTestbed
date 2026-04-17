"""
Reaction Wheel Motor Controller (Pi 5 compatible)
===================================================
Uses gpiozero + lgpio backend.

Run first: sudo pigpiod  (or lgpiod if using lgpio)

Wiring (from wire table):
  GPIO18 (Pin 12) -> RPWM   (forward PWM)   Wire T
  GPIO19 (Pin 35) -> LPWM   (reverse PWM)   Wire U
  GPIO23 (Pin 16) -> R_EN   (right enable)  Wire V
  GPIO24 (Pin 18) -> L_EN   (left enable)   Wire W
"""

import time
from gpiozero import PWMOutputDevice, OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

factory = LGPIOFactory()

# --- Pin Definitions (BCM numbering) ---
RPWM_PIN = 18
LPWM_PIN = 19
R_EN_PIN = 23
L_EN_PIN = 24

PWM_FREQ       = 1000   # Hz
GEAR_RATIO     = 5.0    # 5:1 reduction
MOTOR_MAX_RPM  = 6000   # REV HD Hex Motor free-speed RPM
FLYWHEEL_MAX_RPM = MOTOR_MAX_RPM / GEAR_RATIO  # 1200 RPM


class ReactionWheel:
    def __init__(self):
        self.rpwm = PWMOutputDevice(RPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.lpwm = PWMOutputDevice(LPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.r_en = OutputDevice(R_EN_PIN, initial_value=True, pin_factory=factory)
        self.l_en = OutputDevice(L_EN_PIN, initial_value=True, pin_factory=factory)

        self._speed = 0  # motor duty cycle, -100 to +100
        print(f"Reaction wheel ready. Max flywheel speed: {FLYWHEEL_MAX_RPM:.0f} RPM")

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

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def set_flywheel_rpm(self, rpm: float):
        """
        Set desired flywheel RPM directly.
        Positive = forward, negative = reverse.
        Clamped to ±FLYWHEEL_MAX_RPM.
        """
        rpm = max(-FLYWHEEL_MAX_RPM, min(FLYWHEEL_MAX_RPM, rpm))
        motor_speed_pct = (rpm / FLYWHEEL_MAX_RPM) * 100.0
        self.set_speed(motor_speed_pct)

    def run(self, target_rpm: float, ramp_duration: float = 1.5):
        """
        Ramp to target_rpm and hold, printing live status to terminal.
        Press Ctrl+C to stop.
        """
        print(f"\nTarget: {target_rpm:.0f} RPM  (max: ±{FLYWHEEL_MAX_RPM:.0f} RPM)")
        print("-" * 40)

        # Ramp up smoothly
        start_rpm = self.flywheel_rpm
        steps = 50
        delay = ramp_duration / steps
        for i in range(steps + 1):
            interp = start_rpm + (target_rpm - start_rpm) * (i / steps)
            self.set_flywheel_rpm(interp)
            self._print_status()
            time.sleep(delay)

        # Hold and keep printing
        print("\nHolding speed — press Ctrl+C to stop\n")
        try:
            while True:
                self._print_status()
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n\nStopping...")
            self.ramp_to_rpm(0, ramp_duration=1.0)

    def ramp_to_rpm(self, target_rpm: float, ramp_duration: float = 1.5):
        """Smoothly ramp to a target RPM without holding."""
        start_rpm = self.flywheel_rpm
        steps = 50
        delay = ramp_duration / steps
        for i in range(steps + 1):
            interp = start_rpm + (target_rpm - start_rpm) * (i / steps)
            self.set_flywheel_rpm(interp)
            time.sleep(delay)

    def stop(self):
        self.set_speed(0)

    def cleanup(self):
        self.stop()
        self.rpwm.close()
        self.lpwm.close()
        self.r_en.close()
        self.l_en.close()
        print("GPIO cleaned up.")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def flywheel_rpm(self) -> float:
        return (self._speed / 100.0) * FLYWHEEL_MAX_RPM

    @property
    def speed_percent(self) -> float:
        return abs(self._speed)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _print_status(self):
        rpm = self.flywheel_rpm
        pct = self.speed_percent
        direction = "FWD" if rpm >= 0 else "REV"
        bar_len = int(pct / 2)  # 50 chars = 100%
        bar = "█" * bar_len + "░" * (50 - bar_len)
        print(f"\r{direction} |{bar}| {pct:5.1f}%  {abs(rpm):6.1f} RPM", end="", flush=True)


# --- Entry point ---
if __name__ == "__main__":
    wheel = ReactionWheel()

    TARGET_RPM = 600  # <-- set your desired flywheel RPM here (±1200 max, negative=reverse)

    try:
        wheel.run(TARGET_RPM)
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        wheel.cleanup()
"""
Reaction Wheel Motor Controller (Pi 5 compatible)
===================================================
Uses gpiozero + pigpio backend instead of RPi.GPIO,
which does not support the Raspberry Pi 5.

Run first: sudo pigpiod

Wiring (from wire table):
  GPIO18 (Pin 12) -> RPWM   (forward PWM)   Wire T
  GPIO19 (Pin 35) -> LPWM   (reverse PWM)   Wire U
  GPIO23 (Pin 16) -> R_EN   (right enable)  Wire V
  GPIO24 (Pin 18) -> L_EN   (left enable)   Wire W

Speed: -100 to +100 (negative = reverse, 0 = stop)
"""

import time
from gpiozero import PWMOutputDevice, OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()  # uses pigpio backend

# --- Pin Definitions (BCM numbering) ---
RPWM_PIN = 18
LPWM_PIN = 19
R_EN_PIN = 23
L_EN_PIN = 24

PWM_FREQ = 1000  # Hz


class ReactionWheel:
    def __init__(self):
        self.rpwm = PWMOutputDevice(RPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.lpwm = PWMOutputDevice(LPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
        self.r_en = OutputDevice(R_EN_PIN, initial_value=True, pin_factory=factory)
        self.l_en = OutputDevice(L_EN_PIN, initial_value=True, pin_factory=factory)

        self._speed = 0
        print("Reaction wheel initialized.")

    def set_speed(self, speed: float):
        """
        Set motor speed.
        speed: -100.0 to +100.0
          positive -> forward (RPWM)
          negative -> reverse (LPWM)
          0        -> stop
        """
        speed = max(-100.0, min(100.0, speed))  # clamp
        self._speed = speed
        duty = abs(speed) / 100.0  # gpiozero uses 0.0 - 1.0

        if speed > 0:
            self.rpwm.value = duty
            self.lpwm.value = 0
        elif speed < 0:
            self.rpwm.value = 0
            self.lpwm.value = duty
        else:
            self.rpwm.value = 0
            self.lpwm.value = 0

    def stop(self):
        """Stop the motor immediately."""
        self.set_speed(0)
        print("Motor stopped.")

    def ramp_to(self, target_speed: float, duration: float = 1.0, steps: int = 50):
        """
        Smoothly ramp from current speed to target over `duration` seconds.
        """
        start_speed = self._speed
        delay = duration / steps
        for i in range(steps + 1):
            interp = start_speed + (target_speed - start_speed) * (i / steps)
            self.set_speed(interp)
            time.sleep(delay)

    def cleanup(self):
        """Release GPIO resources."""
        self.stop()
        self.rpwm.close()
        self.lpwm.close()
        self.r_en.close()
        self.l_en.close()
        print("GPIO cleaned up.")

    @property
    def speed(self):
        return self._speed


# --- Simple demo ---
if __name__ == "__main__":
    wheel = ReactionWheel()

    try:
        print("Ramping to 50% forward...")
        wheel.ramp_to(50, duration=1.5)
        time.sleep(2)

        print("Ramping to full speed...")
        wheel.ramp_to(100, duration=1.0)
        time.sleep(2)

        print("Ramping to stop...")
        wheel.ramp_to(0, duration=1.5)
        time.sleep(1)

        print("Ramping to 60% reverse...")
        wheel.ramp_to(-60, duration=1.5)
        time.sleep(2)

        print("Ramping to stop...")
        wheel.ramp_to(0, duration=1.5)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        wheel.cleanup()
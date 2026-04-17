"""
Reaction Wheel Motor Controller
================================
Hardware: Raspberry Pi 5 -> H-Bridge Motor Driver -> REV HD Hex Motor

Wiring (from wire table):
  GPIO18 (Pin 12) -> RPWM   (right/forward PWM)
  GPIO19 (Pin 35) -> LPWM   (left/reverse PWM)
  GPIO23 (Pin 16) -> R_EN   (right enable)
  GPIO24 (Pin 18) -> L_EN   (left enable)

Speed: -100 to +100 (negative = reverse, 0 = stop)
"""

import RPi.GPIO as GPIO
import time

# --- Pin Definitions (BCM numbering) ---
RPWM_PIN = 18   # Forward PWM  (Wire T)
LPWM_PIN = 19   # Reverse PWM  (Wire U)
R_EN_PIN = 23   # Right enable (Wire V)
L_EN_PIN = 24   # Left enable  (Wire W)

PWM_FREQ = 1000  # Hz — 1kHz is safe for most H-bridges


class ReactionWheel:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Set all pins as outputs
        for pin in (RPWM_PIN, LPWM_PIN, R_EN_PIN, L_EN_PIN):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # Enable both H-bridge channels
        GPIO.output(R_EN_PIN, GPIO.HIGH)
        GPIO.output(L_EN_PIN, GPIO.HIGH)

        # Set up PWM on both direction pins
        self.rpwm = GPIO.PWM(RPWM_PIN, PWM_FREQ)
        self.lpwm = GPIO.PWM(LPWM_PIN, PWM_FREQ)
        self.rpwm.start(0)
        self.lpwm.start(0)

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

        if speed > 0:
            self.rpwm.ChangeDutyCycle(speed)
            self.lpwm.ChangeDutyCycle(0)
        elif speed < 0:
            self.rpwm.ChangeDutyCycle(0)
            self.lpwm.ChangeDutyCycle(-speed)
        else:
            self.rpwm.ChangeDutyCycle(0)
            self.lpwm.ChangeDutyCycle(0)

    def stop(self):
        """Stop the motor immediately."""
        self.set_speed(0)
        print("Motor stopped.")

    def ramp_to(self, target_speed: float, duration: float = 1.0, steps: int = 50):
        """
        Smoothly ramp from current speed to target_speed over `duration` seconds.
        Avoids mechanical shock on the reaction wheel bearing.
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
        self.rpwm.stop()
        self.lpwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up.")

    @property
    def speed(self):
        return self._speed


# --- Simple demo / manual control ---
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
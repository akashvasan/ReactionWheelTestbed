import time
from gpiozero import PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

factory = LGPIOFactory()
rpwm = PWMOutputDevice(18, frequency=1000, pin_factory=factory)

for speed in [0.25, 0.5, 0.75, 1.0, 0.0]:
    print(f"Duty cycle: {speed*100:.0f}%")
    rpwm.value = speed
    time.sleep(3)

rpwm.close()
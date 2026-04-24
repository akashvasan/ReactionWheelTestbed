from gpiozero import PWMOutputDevice, OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
import lgpio, time, threading

# Wiring:
#   GPIO18 (Pin 12) -> RPWM  (forward direction)  Wire T
#   GPIO19 (Pin 35) -> LPWM  (reverse direction)  Wire U
#   GPIO23 (Pin 16) -> R_EN  (speed PWM)          Wire V
#   GPIO24 (Pin 18) -> L_EN  (speed PWM)          Wire W
#   GPIO17 (Pin 11) -> Encoder Ch A               Wire X
#   GPIO27 (Pin 13) -> Encoder Ch B               Wire Y

DUTY_CYCLE      = 0.1    # 0.0 to 1.0
COUNTS_PER_REV  = 28 * 2 * 5.0   # both edges on Ch A * gear ratio = 280
SAMPLE_PERIOD   = 0.2             # seconds between RPM prints

factory = LGPIOFactory()

rpwm = OutputDevice(18, initial_value=True, pin_factory=factory)
lpwm = OutputDevice(19, initial_value=False, pin_factory=factory)
r_en = PWMOutputDevice(23, frequency=1000, pin_factory=factory)
l_en = PWMOutputDevice(24, frequency=1000, pin_factory=factory)

# --- Encoder ---
enc_count = 0
enc_lock  = threading.Lock()
gpio      = lgpio.gpiochip_open(0)

def on_edge(chip, pin, level, tick):
    global enc_count
    b = lgpio.gpio_read(gpio, 27)
    with enc_lock:
        if level == 1:
            enc_count += 1 if b == 0 else -1
        else:
            enc_count += 1 if b == 1 else -1

lgpio.gpio_claim_alert(gpio, 17, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(gpio, 27, lgpio.SET_PULL_UP)
enc_cb = lgpio.callback(gpio, 17, lgpio.BOTH_EDGES, on_edge)

# --- Start motor ---
r_en.value = DUTY_CYCLE
l_en.value = DUTY_CYCLE
print(f"Running at {DUTY_CYCLE*100:.0f}% duty — press Ctrl+C to stop\n")

last_count = 0
last_time  = time.monotonic()

try:
    while True:
        time.sleep(SAMPLE_PERIOD)
        now = time.monotonic()
        with enc_lock:
            count = enc_count
        rpm = ((count - last_count) / COUNTS_PER_REV) / ((now - last_time) / 60.0)
        last_count = count
        last_time  = now
        print(f"\rduty={DUTY_CYCLE*100:.0f}%  RPM={rpm:7.1f}  enc={count}", end="", flush=True)
except KeyboardInterrupt:
    pass

r_en.value = 0
l_en.value = 0
enc_cb.cancel()
lgpio.gpiochip_close(gpio)
rpwm.close(); lpwm.close(); r_en.close(); l_en.close()
print("\nStopped.")

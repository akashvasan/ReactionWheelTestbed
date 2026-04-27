"""
Quick test: verify lgpio.tx_pwm produces correct duty on GPIO18/19.
Measure GPIO18 with a multimeter (DC voltage) — at 10% duty on 3.3V, 
you should read ~0.33V. At 50% you should read ~1.65V.
No motor connected needed.
"""
import lgpio, time

RPWM_PIN = 18
LPWM_PIN = 19

gpio = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(gpio, RPWM_PIN, 0)
lgpio.gpio_claim_output(gpio, LPWM_PIN, 0)

for duty in [10, 25, 50]:
    print(f"GPIO18 (RPWM) at {duty}% duty — measure now. Waiting 5s...")
    lgpio.tx_pwm(gpio, RPWM_PIN, 1000, duty)
    time.sleep(5)
    lgpio.tx_pwm(gpio, RPWM_PIN, 1000, 0)
    print("  GPIO18 off.")
    time.sleep(1)

lgpio.gpiochip_close(gpio)
print("Done.")

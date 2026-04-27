import time
import lgpio

RPWM_PIN = 25
LPWM_PIN = 22
R_EN_PIN = 23
L_EN_PIN = 24

PWM_FREQ = 1000


def stop_motor(h):
    lgpio.tx_pwm(h, RPWM_PIN, PWM_FREQ, 0)
    lgpio.tx_pwm(h, LPWM_PIN, PWM_FREQ, 0)
    lgpio.gpio_write(h, RPWM_PIN, 0)
    lgpio.gpio_write(h, LPWM_PIN, 0)


h = lgpio.gpiochip_open(0)

try:
    lgpio.gpio_claim_output(h, RPWM_PIN, 0)
    lgpio.gpio_claim_output(h, LPWM_PIN, 0)
    lgpio.gpio_claim_output(h, R_EN_PIN, 1)
    lgpio.gpio_claim_output(h, L_EN_PIN, 1)

    lgpio.gpio_write(h, R_EN_PIN, 1)
    lgpio.gpio_write(h, L_EN_PIN, 1)

    print("Testing forward...")
    stop_motor(h)
    time.sleep(1)

    lgpio.tx_pwm(h, RPWM_PIN, PWM_FREQ, 25)
    lgpio.tx_pwm(h, LPWM_PIN, PWM_FREQ, 0)
    time.sleep(3)

    print("Stopping...")
    stop_motor(h)
    time.sleep(2)

    print("Testing reverse...")
    lgpio.tx_pwm(h, RPWM_PIN, PWM_FREQ, 0)
    lgpio.tx_pwm(h, LPWM_PIN, PWM_FREQ, 25)
    time.sleep(3)

    print("Stopping...")
    stop_motor(h)
    time.sleep(1)

finally:
    stop_motor(h)
    lgpio.gpio_write(h, R_EN_PIN, 0)
    lgpio.gpio_write(h, L_EN_PIN, 0)
    lgpio.gpiochip_close(h)
    print("GPIO cleaned up.")
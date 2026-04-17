import lgpio, time

ENC_A = 17
ENC_B = 27

gpio = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(gpio, ENC_A, lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(gpio, ENC_B, lgpio.SET_PULL_UP)

print("Monitoring GPIO17 (A) and GPIO27 (B) — press Ctrl+C to stop.")
print("Spin the wheel NOW and watch for changes...\n")

prev_a = lgpio.gpio_read(gpio, ENC_A)
prev_b = lgpio.gpio_read(gpio, ENC_B)
changes = 0
print(f"Initial: A={prev_a}  B={prev_b}")

try:
    while True:
        a = lgpio.gpio_read(gpio, ENC_A)
        b = lgpio.gpio_read(gpio, ENC_B)
        if a != prev_a or b != prev_b:
            changes += 1
            print(f"  CHANGE #{changes}: A={a}  B={b}")
            prev_a, prev_b = a, b
except KeyboardInterrupt:
    pass

print(f"\nStopped. Total changes: {changes}")
lgpio.gpiochip_close(gpio)

import lgpio, time

gpio = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(gpio, 17)  # Encoder A
lgpio.gpio_claim_input(gpio, 27)  # Encoder B

print("Spin the wheel by hand and watch for changes...")
for _ in range(20):
    a = lgpio.gpio_read(gpio, 17)
    b = lgpio.gpio_read(gpio, 27)
    print(f"A={a}  B={b}")
    time.sleep(0.2)

lgpio.gpiochip_close(gpio)
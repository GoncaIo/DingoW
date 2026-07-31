"""MicroPython diagnostic — upload to Pico as main.py via Thonny or mpremote."""
from machine import Pin, UART
import time

uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
led = Pin(25, Pin.OUT)

ENCODER_PINS = [(0, 1), (2, 3), (4, 5), (6, 7)]

pins_a = [Pin(pa, Pin.IN, Pin.PULL_UP) for pa, pb in ENCODER_PINS]
pins_b = [Pin(pb, Pin.IN, Pin.PULL_UP) for pa, pb in ENCODER_PINS]

QE_LOOKUP = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]
positions = [0, 0, 0, 0]
states = [(pins_a[i].value() << 1) | pins_b[i].value() for i in range(4)]

def make_cb(idx):
    def cb(pin):
        new = (pins_a[idx].value() << 1) | pins_b[idx].value()
        positions[idx] += QE_LOOKUP[(states[idx] << 2) | new]
        states[idx] = new
    return cb

cbs = [make_cb(i) for i in range(4)]
for i in range(4):
    pins_a[i].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=cbs[i])
    pins_b[i].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=cbs[i])

uart.write("DIAG READY\n")
print("Encoder diagnostic running...")

led_state = False
while True:
    line = "POS {},{},{},{} GPIO {},{},{},{},{},{},{},{}\n".format(
        positions[0], positions[1], positions[2], positions[3],
        pins_a[0].value(), pins_b[0].value(),
        pins_a[1].value(), pins_b[1].value(),
        pins_a[2].value(), pins_b[2].value(),
        pins_a[3].value(), pins_b[3].value(),
    )
    uart.write(line)
    print(line, end="")

    led_state = not led_state
    led.value(led_state)
    time.sleep_ms(500)

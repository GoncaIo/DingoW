"""MicroPython script for Raspberry Pi Pico."""

from machine import Pin, PWM, UART
import time

PI5_POWER_PIN = 22
_pi5_pin = Pin(PI5_POWER_PIN, Pin.IN)  # start high-Z
time.sleep_ms(500)                      # let Pi 5 PMIC initialize
_pi5_pin = Pin(PI5_POWER_PIN, Pin.OUT, value=0)  # pull LOW = press button
time.sleep_ms(200)
_pi5_pin = Pin(PI5_POWER_PIN, Pin.IN)  # release back to high-Z

# ── Configuration ─────────────────────────────────────────────────────

COUNTS_PER_REV = 14200
CONTROL_HZ = 20
CONTROL_INTERVAL_MS = 1000 // CONTROL_HZ
MAX_RPM = 20.0
PWM_FREQ = 1000
SUPPLY_VOLTAGE = 15.0
MOTOR_VOLTAGE = 12.0
PWM_MAX = int(MOTOR_VOLTAGE / SUPPLY_VOLTAGE * 255)  # 204

ENCODER_PINS = [(0, 1), (2, 3), (4, 5), (6, 7)]
MOTOR_PINS = [
    {"rpwm": 10, "lpwm": 11},
    {"rpwm": 12, "lpwm": 13},
    {"rpwm": 14, "lpwm": 15},
    {"rpwm": 16, "lpwm": 17},
]
ENABLE_PIN = 18

# Quadrature lookup table: index = (old_state << 2) | new_state
QE_LOOKUP = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]


# ── Encoder ───────────────────────────────────────────────────────────

class Encoder:
    def __init__(self, pin_a_num, pin_b_num):
        self.pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)
        self.position = 0
        self._state = (self.pin_a.value() << 1) | self.pin_b.value()
        self.pin_a.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb
        )
        self.pin_b.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb
        )

    def _cb(self, pin):
        new_state = (self.pin_a.value() << 1) | self.pin_b.value()
        self.position += QE_LOOKUP[(self._state << 2) | new_state]
        self._state = new_state

    def reset(self):
        self.position = 0


# ── IBT2 Driver ───────────────────────────────────────────────────────

class IBT2:
    def __init__(self, rpwm_pin, lpwm_pin, freq=PWM_FREQ):
        self.rpwm = PWM(Pin(rpwm_pin))
        self.lpwm = PWM(Pin(lpwm_pin))
        self.rpwm.freq(freq)
        self.lpwm.freq(freq)
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)
        self.current = 0

    def set_pwm(self, value):
        value = max(-PWM_MAX, min(PWM_MAX, value))

        if self.current != 0 and (
            (self.current > 0 and value < 0)
            or (self.current < 0 and value > 0)
        ):
            self.stop()
            time.sleep_ms(10)

        self.current = value

        if value == 0:
            self.stop()
        elif value > 0:
            self.lpwm.duty_u16(0)
            self.rpwm.duty_u16(int((value / 255) * 65535))
        else:
            self.rpwm.duty_u16(0)
            self.lpwm.duty_u16(int((abs(value) / 255) * 65535))

    def stop(self):
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)
        self.current = 0


# ── PID Controller ────────────────────────────────────────────────────

class PID:
    def __init__(self, kp=0.2, ki=0.05, kd=0.01, max_integral=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral

        d_term = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * d_term
        if out > PWM_MAX:
            return float(PWM_MAX)
        elif out < -PWM_MAX:
            return float(-PWM_MAX)
        return out

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0


# ── Init ──────────────────────────────────────────────────────────────

uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
led = Pin(25, Pin.OUT)

en_pin = Pin(ENABLE_PIN, Pin.OUT)
en_pin.value(1)

encoders = [Encoder(a, b) for a, b in ENCODER_PINS]
motors = [IBT2(p["rpwm"], p["lpwm"]) for p in MOTOR_PINS]
pids = [PID() for _ in range(4)]

targets = [0.0, 0.0, 0.0, 0.0]
ff_gain = 13.7
last_positions = [0, 0, 0, 0]
rpms = [0.0, 0.0, 0.0, 0.0]
pwm_outputs = [0, 0, 0, 0]
enabled = True


# ── Command Parser ────────────────────────────────────────────────────

def handle_command(cmd):
    global targets, ff_gain, enabled

    if not cmd:
        return

    tag = cmd[0]

    if tag == "T":
        parts = cmd[2:].split(",") if len(cmd) > 2 else []
        if len(parts) == 1:
            v = float(parts[0])
            v = max(-MAX_RPM, min(MAX_RPM, v))
            for i in range(4):
                if (targets[i] > 0) != (v > 0) or (targets[i] < 0) != (v < 0):
                    pids[i].reset()
                targets[i] = v
        elif len(parts) == 4:
            for i in range(4):
                v = max(-MAX_RPM, min(MAX_RPM, float(parts[i])))
                if (targets[i] > 0) != (v > 0) or (targets[i] < 0) != (v < 0):
                    pids[i].reset()
                targets[i] = v
        uart.write("OK T\n")

    elif tag == "S":
        for i in range(4):
            targets[i] = 0.0
            pids[i].reset()
            motors[i].stop()
        uart.write("OK S\n")

    elif tag == "X":
        for i in range(4):
            targets[i] = 0.0
            pids[i].reset()
            motors[i].stop()
        en_pin.value(0)
        enabled = False
        uart.write("OK X\n")

    elif tag == "G":
        en_pin.value(1)
        enabled = True
        uart.write("OK G\n")

    elif tag == "R":
        for enc in encoders:
            enc.reset()
        for i in range(4):
            last_positions[i] = 0
            rpms[i] = 0.0
        uart.write("OK R\n")

    elif tag == "P":
        parts = cmd[2:].split(",")
        if len(parts) == 3:
            kp, ki, kd = float(parts[0]), float(parts[1]), float(parts[2])
            for pid in pids:
                pid.kp = kp
                pid.ki = ki
                pid.kd = kd
                pid.reset()
            uart.write("OK P\n")

    elif tag == "F":
        ff_gain = float(cmd[2:])
        uart.write("OK F\n")

    elif tag == "?":
        send_status()


def send_status():
    parts = []
    for i in range(4):
        parts.append(
            "{:.1f},{:.1f},{},{}".format(
                targets[i], rpms[i], pwm_outputs[i], encoders[i].position
            )
        )
    uart.write("D " + ",".join(parts) + "\n")


# ── Main Loop ─────────────────────────────────────────────────────────

last_control_time = time.ticks_ms()
led_state = False
rx_buf = ""

while True:
    # Read UART commands
    if uart.any():
        try:
            raw = uart.read(uart.any())
            if raw:
                rx_buf += raw.decode()
                while "\n" in rx_buf:
                    line, rx_buf = rx_buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        try:
                            handle_command(line)
                        except Exception as e:
                            uart.write("ERR {}\n".format(e))
        except Exception:
            pass

    now = time.ticks_ms()
    dt_ms = time.ticks_diff(now, last_control_time)

    if dt_ms >= CONTROL_INTERVAL_MS:
        dt_s = dt_ms / 1000.0

        for i in range(4):
            pos = encoders[i].position
            delta = pos - last_positions[i]
            rpms[i] = (delta / COUNTS_PER_REV) / (dt_s / 60.0) if dt_s > 0 else 0.0
            last_positions[i] = pos

            if enabled:
                ff = ff_gain * targets[i]
                fb = pids[i].update(targets[i], rpms[i], dt_s)
                pwm_val = int(ff + fb)
                if pwm_val > PWM_MAX:
                    pwm_val = PWM_MAX
                elif pwm_val < -PWM_MAX:
                    pwm_val = -PWM_MAX
                motors[i].set_pwm(pwm_val)
                pwm_outputs[i] = pwm_val
            else:
                pwm_outputs[i] = 0

        send_status()
        last_control_time = now

        led_state = not led_state
        led.value(led_state)

    time.sleep_ms(1)

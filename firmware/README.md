# Firmware

Microcontroller code that runs outside the ROS workspace, on the Pico the Pi 5
talks to over serial.

## `pico/` — Raspberry Pi Pico (wheel drive)

MicroPython. Runs the low-level wheel loop so the Pi 5 only has to send target
speeds: quadrature encoder decoding on interrupts, PID with feed-forward, and
PWM generation for the four IBT-2 motor drivers.

| File | Purpose |
|---|---|
| `pico_motor_controller.py` | Main firmware. Flash as `main.py` on the Pico. |
| `pico_encoder_diag.py` | Standalone bring-up test: reads the encoders and echoes counts over UART, blinking the on-board LED. Useful for confirming wiring before flashing the real firmware. |

Flashing (either file, one at a time):

```bash
mpremote connect /dev/ttyACM0 fs cp pico_motor_controller.py :main.py
```

or open the file in Thonny and *Save as → Raspberry Pi Pico → `main.py`*.

The Pi-side counterpart is
[`HardwareInterface_Pico.py`](../software/dingo_ws/src/dingo_hardware_interfacing/dingo_servo_interfacing/src/dingo_servo_interfacing/HardwareInterface_Pico.py),
which opens the serial port and publishes wheel feedback to ROS. The serial
protocol (`T`/`S`/`X`/`G`/`R`/`P`/`F` commands) is documented in the header of
`pico_motor_controller.py`.
